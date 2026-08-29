// front_camera_vision.cpp
// Headless YOLOv26 segmentation inference node for D455 camera
// Publishes DetectionArray for external visualization nodes

#include <rclcpp/rclcpp.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "snappy_interfaces/msg/detection_array.hpp"
#include "snappy_interfaces/msg/object_detection.hpp"
#include "snappy_interfaces/msg/bounding_box2_d.hpp"
#include "snappy_interfaces/msg/polygon2_d.hpp"
#include <geometry_msgs/msg/point32.hpp>
#include "snappy_computer_vision/detection_classes.hpp"


using namespace std::chrono_literals;

std::vector<std::string> class_names_ = snappy_computer_vision::kFrontDetectionClasses;

class FrontCameraVision : public rclcpp::Node
{
public:
    explicit FrontCameraVision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("front_camera_vision", options)
    {
        // Declare parameters
        inference_hz_ = this->declare_parameter<double>("inference_hz", 0.0);
        conf_threshold_ = this->declare_parameter<double>("conf_threshold", 0.5);
        engine_path_ = this->declare_parameter<std::string>(
            "engine_path", "/home/kraken/Desktop/ffc_rs_26.engine");
        num_classes_ = this->declare_parameter<int>("num_classes", 8);
        video_capture_enabled_ = this->declare_parameter<bool>("video_capture_enabled", true);
        video_capture_dir_ = this->declare_parameter<std::string>(
            "video_capture_directory", "/home/snappy_data/front_camera");
        video_capture_fps_ = this->declare_parameter<double>("video_capture_fps", 30.0);

        RCLCPP_INFO(get_logger(), "Front Camera Vision (YOLOv26 Seg) initializing...");
        RCLCPP_INFO(get_logger(), "  inference_hz: %.1f", inference_hz_);
        RCLCPP_INFO(get_logger(), "  conf_threshold: %.2f", conf_threshold_);
        RCLCPP_INFO(get_logger(), "  engine_path: %s", engine_path_.c_str());
        RCLCPP_INFO(get_logger(), "  video_capture_enabled: %s", video_capture_enabled_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  video_capture_directory: %s", video_capture_dir_.c_str());

        if (video_capture_enabled_) {
            start_video_capture();
        }

        init_tensorrt();

        // Subscribe to D455 color and aligned depth feeds together so objects get range estimate from the same frame
        color_sub_.subscribe(
            this, "/d455/d455/color/image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        depth_sub_.subscribe(
            this, "/d455/d455/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10));
        sync_->connectInput(color_sub_, depth_sub_);
        sync_->registerCallback(
            std::bind(&FrontCameraVision::image_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Publish detections
        detection_pub_ = this->create_publisher<snappy_interfaces::msg::DetectionArray>(
            "/d455/detections", 10);

        // Start inference worker
        worker_running_ = true;
        worker_thread_ = std::thread(&FrontCameraVision::inference_worker, this);

        RCLCPP_INFO(get_logger(), "Front Camera Vision ready");
    }

    ~FrontCameraVision() override
    {
        {
            std::lock_guard<std::mutex> lock(video_mutex_);
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(get_logger(), "Video capture stopped: %s", video_path_.c_str());
            }
        }

        worker_running_ = false;
        queue_cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        if (context_) { delete context_; context_ = nullptr; }
        if (engine_) { delete engine_; engine_ = nullptr; }
        if (runtime_) { delete runtime_; runtime_ = nullptr; }

        if (device_input_) cudaFree(device_input_);
        if (device_output0_) cudaFree(device_output0_);
        if (device_output1_) cudaFree(device_output1_);
        if (host_output0_) cudaFreeHost(host_output0_);
        if (host_output1_) cudaFreeHost(host_output1_);
        if (stream_) cudaStreamDestroy(stream_);
    }

private:
    // ---- Data types --------------------------------------------------------

    struct InferenceJob {
        cv::Mat image;
        cv::Mat depth_image;
        std::string depth_encoding;
        rclcpp::Time timestamp;
    };

    struct Detection {
        float x1, y1, x2, y2;
        float confidence;
        int class_id;
        float distance_m = -1.0f;
        std::vector<cv::Point2f> mask_polygon;
    };

    struct Anchor {
        float x, y, w, h;
        float confidence;
        int class_id;
        std::vector<float> mask_coeffs;
    };

    // ---- ROS callbacks -----------------------------------------------------

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;

    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
    {
        const auto now = this->get_clock()->now();
        if (last_inference_time_.has_value()) {
            const double elapsed = (now - *last_inference_time_).seconds();
            if (inference_hz_ > 0.0 && elapsed < (1.0 / inference_hz_)) {
                return;
            }
        }
        last_inference_time_ = now;

        try {
            auto color_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
            auto depth_ptr = cv_bridge::toCvShare(depth_msg);

            InferenceJob job;
            job.image = color_ptr->image.clone();
            job.depth_image = depth_ptr->image.clone();
            job.depth_encoding = depth_msg->encoding;
            job.timestamp = rclcpp::Time(color_msg->header.stamp.sec, color_msg->header.stamp.nanosec);

            if (video_capture_enabled_) {
                std::lock_guard<std::mutex> video_lock(video_mutex_);
                if (video_writer_.isOpened()) {
                    video_writer_ << job.image;
                }
            }

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                // Drop oldest frame if queue is full (never accumulate latency)
                while (job_queue_.size() >= 2) {
                    job_queue_.pop_front();
                }
                job_queue_.push_back(std::move(job));
            }
            queue_cv_.notify_one();
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    // ---- Inference worker thread -------------------------------------------

    void inference_worker()
    {
        while (worker_running_) {
            InferenceJob job;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] {
                    return !worker_running_ || !job_queue_.empty();
                });
                if (!worker_running_ && job_queue_.empty()) break;
                job = std::move(job_queue_.front());
                job_queue_.pop_front();
            }

            if (job.image.empty()) continue;

            float inference_ms = 0.0f;
            auto detections = run_inference(job.image, inference_ms);
            // depth/distance of object calculated by a median of the depth pixels within the segmented object
            for (auto & det : detections) {
                det.distance_m = estimate_distance_m(job.depth_image, job.depth_encoding, det);
            }
            // Save a frame for later review, at most once every 5 seconds.
            const double frame_s = job.timestamp.seconds();
            if (frame_s - last_save_s_ >= 5.0) {
                last_save_s_ = frame_s;
                save_image(job.image, job.timestamp);
            }
            publish_detections(detections, job.timestamp, inference_ms);
        }
    }

    void start_video_capture()
    {
        std::filesystem::create_directories(video_capture_dir_, std::error_code{});

        const auto now = std::chrono::system_clock::now();
        const auto tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
        localtime_r(&tt, &tm_buf);

        std::ostringstream timestamp;
        timestamp << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        video_path_ = (std::filesystem::path(video_capture_dir_) / ("front_camera_" + timestamp.str() + ".mp4")).string();

        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        video_writer_ = cv::VideoWriter(video_path_, fourcc, video_capture_fps_, cv::Size(848, 480));
        if (!video_writer_.isOpened()) {
            RCLCPP_WARN(get_logger(), "Failed to open video writer at %s", video_path_.c_str());
            video_capture_enabled_ = false;
            return;
        }

        RCLCPP_INFO(get_logger(), "Video capture enabled: %s", video_path_.c_str());
    }

    // take image and save to desktop
    void save_image(const cv::Mat &image, const rclcpp::Time &timestamp)
    {
        // imwrite does NOT create directories and returns false (never throws)
        // on a bad path -- so the dir must exist first, and we check the result.
        static const std::string dir = "/ros2_ws/snappy_inference/d455";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        const std::string path =
            dir + "/front_camera_" + std::to_string(timestamp.seconds()) + ".jpg";
        if (!cv::imwrite(path, image)) {
            RCLCPP_WARN(get_logger(), "save_image: failed to write %s", path.c_str());
        }
    }

    // ---- Core inference pipeline -------------------------------------------

    std::vector<Detection> run_inference(const cv::Mat & image, float & inference_ms)
    {
        auto t0 = std::chrono::steady_clock::now();

        const int orig_h = image.rows;
        const int orig_w = image.cols;

        // Letterbox resize
        cv::Mat resized;
        float scale;
        int pad_x, pad_y;
        letterbox(image, resized, scale, pad_x, pad_y);

        // BGR -> RGB, scale to [0,1], HWC -> CHW (NCHW blob) in one vectorized
        // call. `resized` is already letterboxed to input size, so the Size arg
        // is a no-op resize; swapRB handles BGR->RGB. `blob` must stay alive
        // until the stream sync below (cudaMemcpyAsync only queues the copy).
        cv::Mat blob = cv::dnn::blobFromImage(
            resized, 1.0 / 255.0, cv::Size(input_w_, input_h_),
            cv::Scalar(), /*swapRB=*/true, /*crop=*/false);

        cudaMemcpyAsync(device_input_, blob.ptr<float>(),
                        blob.total() * sizeof(float),
                        cudaMemcpyHostToDevice, stream_);

        context_->enqueueV3(stream_);

        cudaMemcpyAsync(host_output0_, device_output0_,
                        output0_size_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);
        cudaMemcpyAsync(host_output1_, device_output1_,
                        output1_size_ * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_);

        cudaStreamSynchronize(stream_);

        auto detections = parse_segmentation_output(
            host_output0_, host_output1_, orig_w, orig_h, scale, pad_x, pad_y);

        auto t1 = std::chrono::steady_clock::now();
        inference_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                             "Inference: %.2f ms (%.1f FPS), %zu detections",
                             inference_ms, 1000.0f / inference_ms, detections.size());

        return detections;
    }

    // ---- Preprocessing -----------------------------------------------------

    void letterbox(const cv::Mat & src, cv::Mat & dst,
                   float & scale, int & pad_x, int & pad_y)
    {
        scale = std::min(static_cast<float>(input_w_) / src.cols,
                         static_cast<float>(input_h_) / src.rows);
        int new_w = static_cast<int>(src.cols * scale);
        int new_h = static_cast<int>(src.rows * scale);
        pad_x = (input_w_ - new_w) / 2;
        pad_y = (input_h_ - new_h) / 2;

        cv::resize(src, dst, cv::Size(new_w, new_h));
        cv::copyMakeBorder(dst, dst, pad_y, pad_y, pad_x, pad_x,
                           cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    }

    // ---- YOLOv26 segmentation post-processing ------------------------------

    std::vector<Detection> parse_segmentation_output(
        const float * output0, const float * output1,
        int orig_w, int orig_h, float scale, int pad_x, int pad_y)
    {
        // YOLO26 is NMS-free / end-to-end: output0 layout is
        //   [1, num_det, 4 + 1 + 1 + 32]  e.g. (1, 300, 38)
        // per detection: [x1, y1, x2, y2, conf, class_id, m0..m31]
        //   - boxes are xyxy in letterboxed INPUT pixel space (0..input_w/h)
        //   - detections are already filtered + sorted by conf (no manual NMS)
        // output1 (proto masks) layout: [1, 32, mask_h_, mask_w_]
        const int num_det = num_detections_;          // out0_dims.d[1], e.g. 300
        const int attrs   = det_attrs_;               // out0_dims.d[2], e.g. 38
        const int num_mask_coeffs = attrs - 6;        // 38 - 6 = 32
        constexpr float mask_threshold = 0.5f;

        std::vector<Detection> detections;

        for (int i = 0; i < num_det; ++i) {
            const int offset = i * attrs;
            const float conf = output0[offset + 4];

            // Sorted by confidence desc: once below threshold, the rest are too.
            if (conf < static_cast<float>(conf_threshold_)) break;

            const int class_id = static_cast<int>(std::lround(output0[offset + 5]));

            Detection det;
            det.x1 = std::clamp((output0[offset + 0] - pad_x) / scale, 0.0f, static_cast<float>(orig_w));
            det.y1 = std::clamp((output0[offset + 1] - pad_y) / scale, 0.0f, static_cast<float>(orig_h));
            det.x2 = std::clamp((output0[offset + 2] - pad_x) / scale, 0.0f, static_cast<float>(orig_w));
            det.y2 = std::clamp((output0[offset + 3] - pad_y) / scale, 0.0f, static_cast<float>(orig_h));
            det.confidence = conf;
            det.class_id = class_id;

            std::vector<float> coeffs(num_mask_coeffs);
            for (int k = 0; k < num_mask_coeffs; ++k) {
                coeffs[k] = output0[offset + 6 + k];
            }
            det.mask_polygon = decode_mask_polygon(coeffs, output1, det,
                                                    orig_w, orig_h, pad_x, pad_y,
                                                    mask_threshold);

            detections.push_back(std::move(det));
        }

        // Debug: dump the top detection's raw values so the box format can be
        // verified (xyxy vs xywh) against ground truth.
        if (num_det > 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "raw det0: [%.1f %.1f %.1f %.1f] conf=%.3f cls=%.1f | kept=%zu",
                output0[0], output0[1], output0[2], output0[3],
                output0[4], output0[5], detections.size());
        }

        return detections;
    }

    // ---- Mask decoding: proto matmul -> sigmoid -> threshold -> contour ----

    std::vector<cv::Point2f> decode_mask_polygon(
        const std::vector<float> & mask_coeffs,
        const float * proto,
        const Detection & det,
        int orig_w, int orig_h,
        int pad_x, int pad_y,
        float mask_threshold)
    {
        const int num_mask_coeffs = static_cast<int>(mask_coeffs.size());

        // coeffs . proto -> low-res mask, then sigmoid.
        // proto is [num_mask_coeffs, mask_h_*mask_w_]; coeffs is [1, num_mask_coeffs].
        // A single GEMM (BLAS) replaces the per-pixel triple loop -- the dominant
        // CPU cost per detection.
        const cv::Mat proto_mat(num_mask_coeffs, mask_h_ * mask_w_, CV_32F,
                                const_cast<float *>(proto));
        const cv::Mat coeffs_mat(1, num_mask_coeffs, CV_32F,
                                 const_cast<float *>(mask_coeffs.data()));
        cv::Mat mask_low = coeffs_mat * proto_mat;        // [1, mask_h_*mask_w_]
        mask_low = mask_low.reshape(1, mask_h_);          // [mask_h_, mask_w_]

        // Sigmoid in place: 1 / (1 + exp(-x))
        cv::exp(-mask_low, mask_low);
        cv::add(mask_low, 1.0, mask_low);
        cv::divide(1.0, mask_low, mask_low);

        // Upscale proto mask to letterboxed input size
        cv::Mat mask_input;
        cv::resize(mask_low, mask_input, cv::Size(input_w_, input_h_), 0, 0, cv::INTER_LINEAR);

        // Crop out the letterbox padding, leaving only the real-image region
        cv::Rect crop_roi(pad_x, pad_y, input_w_ - 2 * pad_x, input_h_ - 2 * pad_y);
        crop_roi &= cv::Rect(0, 0, mask_input.cols, mask_input.rows);
        if (crop_roi.width <= 0 || crop_roi.height <= 0) {
            return {};
        }
        cv::Mat mask_cropped = mask_input(crop_roi);

        // Resize to original image dimensions
        cv::Mat mask_full;
        cv::resize(mask_cropped, mask_full, cv::Size(orig_w, orig_h), 0, 0, cv::INTER_LINEAR);

        // Threshold to binary
        cv::Mat mask_bin;
        cv::threshold(mask_full, mask_bin, mask_threshold, 255, cv::THRESH_BINARY);
        mask_bin.convertTo(mask_bin, CV_8U);

        // Gate by the detection's own box (proto masks are class-agnostic;
        // the box tells us which instance this mask belongs to)
        cv::Rect box(static_cast<int>(det.x1), static_cast<int>(det.y1),
                    static_cast<int>(det.x2 - det.x1), static_cast<int>(det.y2 - det.y1));
        box &= cv::Rect(0, 0, orig_w, orig_h);

        cv::Mat box_mask = cv::Mat::zeros(mask_bin.size(), CV_8U);
        if (box.width > 0 && box.height > 0) {
            mask_bin(box).copyTo(box_mask(box));
        }

        // Extract the largest external contour as the polygon
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(box_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            return {};
        }

        auto largest = std::max_element(contours.begin(), contours.end(),
            [](const auto & a, const auto & b) { return a.size() < b.size(); });

        std::vector<cv::Point2f> polygon;
        polygon.reserve(largest->size());
        for (const auto & pt : *largest) {
            polygon.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
        }
        return polygon;
    }

    static float compute_iou(const Anchor & a, const Anchor & b)
    {
        float x1 = std::max(a.x - a.w / 2.0f, b.x - b.w / 2.0f);
        float y1 = std::max(a.y - a.h / 2.0f, b.y - b.h / 2.0f);
        float x2 = std::min(a.x + a.w / 2.0f, b.x + b.w / 2.0f);
        float y2 = std::min(a.y + a.h / 2.0f, b.y + b.h / 2.0f);

        float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
        float union_area = a.w * a.h + b.w * b.h - inter;
        return (union_area > 0.0f) ? (inter / union_area) : 0.0f;
    }

    float estimate_distance_m(
        const cv::Mat & depth_image,
        const std::string & depth_encoding,
        const Detection & det)
    {
        if (depth_image.empty()) {
            return -1.0f;
        }

        cv::Rect box(
            static_cast<int>(std::floor(det.x1)),
            static_cast<int>(std::floor(det.y1)),
            static_cast<int>(std::ceil(det.x2 - det.x1)),
            static_cast<int>(std::ceil(det.y2 - det.y1)));
        box &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);
        if (box.width <= 0 || box.height <= 0) {
            return -1.0f;
        }

        cv::Mat object_mask = cv::Mat::zeros(depth_image.size(), CV_8UC1);
        if (det.mask_polygon.size() >= 3) {
            std::vector<cv::Point> contour;
            contour.reserve(det.mask_polygon.size());
            for (const auto & pt : det.mask_polygon) {
                contour.emplace_back(
                    static_cast<int>(std::lround(pt.x)),
                    static_cast<int>(std::lround(pt.y)));
            }
            const cv::Point * pts = contour.data();
            int npts = static_cast<int>(contour.size());
            cv::fillPoly(object_mask, &pts, &npts, 1, cv::Scalar(255));
        } else {
            object_mask(box).setTo(255);
        }

        std::vector<float> samples;
        samples.reserve(static_cast<size_t>(box.width * box.height));

        const cv::Mat depth_roi = depth_image(box);
        const cv::Mat mask_roi = object_mask(box);

        if (depth_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            for (int y = 0; y < depth_roi.rows; ++y) {
                const auto * depth_row = depth_roi.ptr<uint16_t>(y);
                const auto * mask_row = mask_roi.ptr<uint8_t>(y);
                for (int x = 0; x < depth_roi.cols; ++x) {
                    const uint16_t depth_mm = depth_row[x];
                    if (mask_row[x] != 0 && depth_mm > 0) {
                        samples.push_back(static_cast<float>(depth_mm) * 0.001f);
                    }
                }
            }
        } else if (depth_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            for (int y = 0; y < depth_roi.rows; ++y) {
                const auto * depth_row = depth_roi.ptr<float>(y);
                const auto * mask_row = mask_roi.ptr<uint8_t>(y);
                for (int x = 0; x < depth_roi.cols; ++x) {
                    const float depth_m = depth_row[x];
                    if (mask_row[x] != 0 && depth_m > 0.0f && std::isfinite(depth_m)) {
                        samples.push_back(depth_m);
                    }
                }
            }
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Unsupported depth encoding for range estimate: %s", depth_encoding.c_str());
            return -1.0f;
        }

        if (samples.empty()) {
            return -1.0f;
        }

        auto middle = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2);
        std::nth_element(samples.begin(), middle, samples.end());
        return *middle;
    }

    // ---- Publishing --------------------------------------------------------

    void publish_detections(const std::vector<Detection> & detections,
                            const rclcpp::Time & timestamp,
                            float inference_ms)
    {
        auto msg = snappy_interfaces::msg::DetectionArray();
        msg.header.stamp = timestamp;
        msg.header.frame_id = "d455_color_optical_frame";
        msg.inference_time_ms = static_cast<uint32_t>(std::lround(inference_ms));
        msg.detections.reserve(detections.size());

        for (const auto & det : detections) {
            snappy_interfaces::msg::ObjectDetection obj;
            obj.confidence = det.confidence;
            obj.object_class = class_name_for(det.class_id);
            obj.distance_m = det.distance_m;
            obj.timestamp = timestamp;

            snappy_interfaces::msg::BoundingBox2D bbox;
            bbox.x = det.x1;
            bbox.y = det.y1;
            bbox.width = det.x2 - det.x1;
            bbox.height = det.y2 - det.y1;
            obj.bounding_box = bbox;

            snappy_interfaces::msg::Polygon2D poly;
            poly.points.reserve(det.mask_polygon.size());
            for (const auto & pt : det.mask_polygon) {
                geometry_msgs::msg::Point32 p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0.0f;
                poly.points.push_back(p);
            }
            obj.mask_polygons.push_back(std::move(poly));

            msg.detections.push_back(std::move(obj));
        }

        detection_pub_->publish(msg);
    }

    // ---- Class label lookup -------------------------------------------------

    std::string class_name_for(int class_id) const
    {
        if (class_id >= 0 && static_cast<size_t>(class_id) < class_names_.size()) {
            return class_names_[class_id];
        }
        return "unknown";
    }

    // ---- TensorRT initialization -------------------------------------------

    void init_tensorrt()
    {
        std::ifstream file(engine_path_, std::ios::binary);
        if (!file.good()) {
            throw std::runtime_error("Failed to open TensorRT engine: " + engine_path_);
        }

        file.seekg(0, std::ios::end);
        size_t size = static_cast<size_t>(file.tellg());
        file.seekg(0, std::ios::beg);

        std::vector<char> engine_data(size);
        file.read(engine_data.data(), static_cast<std::streamsize>(size));
        file.close();

        // Ultralytics `yolo export format=engine` prepends a metadata blob:
        //   [int32 little-endian length][metadata JSON]["...serialized TRT plan"]
        // Skip it so deserialize sees the plan's magic tag. Plain trtexec engines
        // have no header (length byte won't look like a JSON-prefixed blob).
        size_t offset = 0;
        if (size > 4) {
            int32_t meta_len;
            std::memcpy(&meta_len, engine_data.data(), sizeof(meta_len));
            if (meta_len > 0 && static_cast<size_t>(meta_len) < size - 4 &&
                engine_data[4] == '{') {
                offset = 4 + static_cast<size_t>(meta_len);
                RCLCPP_INFO(get_logger(),
                            "Stripping %d-byte Ultralytics engine metadata header",
                            meta_len);
            }
        }

        runtime_ = nvinfer1::createInferRuntime(trt_logger_);
        if (!runtime_) throw std::runtime_error("Failed to create TRT runtime");

        engine_ = runtime_->deserializeCudaEngine(engine_data.data() + offset, size - offset);
        if (!engine_) throw std::runtime_error("Failed to deserialize TRT engine");

        context_ = engine_->createExecutionContext();
        if (!context_) throw std::runtime_error("Failed to create TRT execution context");

        // YOLOv26 seg has 3 I/O tensors: input, output0 (boxes+coeffs), output1 (proto)
        input_name_ = engine_->getIOTensorName(0);
        output0_name_ = engine_->getIOTensorName(1);
        output1_name_ = engine_->getIOTensorName(2);

        auto input_dims = engine_->getTensorShape(input_name_.c_str());
        input_h_ = input_dims.d[2];
        input_w_ = input_dims.d[3];

        auto out0_dims = engine_->getTensorShape(output0_name_.c_str());
        output0_size_ = 1;
        for (int i = 0; i < out0_dims.nbDims; ++i) output0_size_ *= out0_dims.d[i];
        // YOLO26 NMS-free: out0 = [1, num_det, attrs] e.g. (1, 300, 38)
        num_detections_ = out0_dims.d[1];
        det_attrs_      = out0_dims.d[2];

        auto out1_dims = engine_->getTensorShape(output1_name_.c_str());
        output1_size_ = 1;
        for (int i = 0; i < out1_dims.nbDims; ++i) output1_size_ *= out1_dims.d[i];
        mask_h_ = out1_dims.d[2];
        mask_w_ = out1_dims.d[3];

        // Allocate device buffers
        cudaStreamCreate(&stream_);
        cudaMalloc(&device_input_,   3 * input_h_ * input_w_ * sizeof(float));
        cudaMalloc(&device_output0_, output0_size_ * sizeof(float));
        cudaMalloc(&device_output1_, output1_size_ * sizeof(float));

        // Pinned host buffers for async D2H
        cudaHostAlloc(&host_output0_, output0_size_ * sizeof(float), cudaHostAllocDefault);
        cudaHostAlloc(&host_output1_, output1_size_ * sizeof(float), cudaHostAllocDefault);

        // Bind tensor addresses once
        context_->setTensorAddress(input_name_.c_str(),   device_input_);
        context_->setTensorAddress(output0_name_.c_str(), device_output0_);
        context_->setTensorAddress(output1_name_.c_str(), device_output1_);

        RCLCPP_INFO(get_logger(), "Engine loaded: %dx%d input | out0=%zu | out1=%zu",
                     input_w_, input_h_, output0_size_, output1_size_);
    }

    // ---- TRT logger --------------------------------------------------------

    class TRTLogger : public nvinfer1::ILogger {
        void log(Severity severity, const char * msg) noexcept override {
            if (severity <= Severity::kWARNING)
                std::cerr << "[TRT] " << msg << std::endl;
        }
    } trt_logger_;

    // ---- Members -----------------------------------------------------------

    // ROS
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<snappy_interfaces::msg::DetectionArray>::SharedPtr detection_pub_;

    // TensorRT / CUDA
    nvinfer1::IRuntime *         runtime_      = nullptr;
    nvinfer1::ICudaEngine *      engine_       = nullptr;
    nvinfer1::IExecutionContext * context_     = nullptr;
    cudaStream_t                 stream_       = nullptr;
    float * device_input_   = nullptr;
    float * device_output0_ = nullptr;
    float * device_output1_ = nullptr;
    float * host_output0_   = nullptr;
    float * host_output1_   = nullptr;

    std::string input_name_, output0_name_, output1_name_;
    int    input_w_ = 640, input_h_ = 640;
    size_t output0_size_ = 0, output1_size_ = 0;
    int    num_detections_ = 0, det_attrs_ = 0;

    // Video capture
    bool video_capture_enabled_ = true;
    std::string video_capture_dir_ = "/home/snappy_data/front_camera";
    double video_capture_fps_ = 30.0;
    std::string video_path_;
    cv::VideoWriter video_writer_;
    std::mutex video_mutex_;

    // Threading
    std::deque<InferenceJob>   job_queue_;
    std::mutex                 queue_mutex_;
    std::condition_variable    queue_cv_;
    std::thread                worker_thread_;
    std::atomic<bool>          worker_running_{false};

    // Parameters
    double      inference_hz_    = 10.0;
    double      conf_threshold_  = 0.5;
    int         num_classes_     = 80;
    std::string engine_path_;
    std::optional<rclcpp::Time> last_inference_time_;
    double last_save_s_ = 0.0;

    int mask_h_ = 0, mask_w_ = 0;

    // FFC (front camera) classes, indexed by class_id (see detection_classes.hpp).
    // class_name_for() returns "unknown" for out-of-range ids.
    std::vector<std::string> class_names_ = snappy_computer_vision::kFrontDetectionClasses;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontCameraVision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
