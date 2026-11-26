/*
    Node.cpp constructs front camera inference node and image callback.
*/

#include "front_camera_vision.hpp"

FrontCameraVision::FrontCameraVision(const rclcpp::NodeOptions & options)
: Node("front_camera_vision", options)
{
    // Declare parameters
    inference_hz_ = this->declare_parameter<double>("inference_hz", 0.0);
    high_conf_threshold_ = this->declare_parameter<double>("high_conf_threshold", 0.5);
    engine_path_ = this->declare_parameter<std::string>(
        "engine_path", "/home/kraken/Desktop/ffc_rs_26.engine");
    num_classes_ = this->declare_parameter<int>("num_classes", 8);
    camera_name_ = this->declare_parameter<std::string>("camera_name", "front");
    capture_target_ = this->declare_parameter<std::string>("capture_target", "both");
    capture_mode_ = this->declare_parameter<std::string>("capture_mode", "video");
    video_capture_enabled_ = this->declare_parameter<bool>(
        "video_capture_enabled",
        capture_mode_ == "video");
    image_capture_dir_ = this->declare_parameter<std::string>(
        "image_capture_directory", "/home/snappy_data/front_camera/images");
    video_capture_dir_ = this->declare_parameter<std::string>(
        "video_capture_directory", "/home/snappy_data/front_camera/videos");
    video_capture_fps_ = this->declare_parameter<double>("video_capture_fps", 30.0);

    capture_enabled_ = (capture_target_ == "both" || capture_target_ == camera_name_);

    if (capture_mode_ != "image" && capture_mode_ != "video") {
        RCLCPP_WARN(get_logger(), "Unsupported capture_mode '%s'; defaulting to 'video'",
                    capture_mode_.c_str());
        capture_mode_ = "video";
    }
    if (capture_mode_ == "video") {
        video_capture_enabled_ = capture_enabled_;
    } else {
        video_capture_enabled_ = false;
    }

    RCLCPP_INFO(get_logger(), "Front Camera Vision (YOLOv26 Seg) initializing...");
    RCLCPP_INFO(get_logger(), "  inference_hz: %.1f", inference_hz_);
    RCLCPP_INFO(get_logger(), "  conf_threshold: %.2f", high_conf_threshold_);
    RCLCPP_INFO(get_logger(), "  engine_path: %s", engine_path_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_name: %s", camera_name_.c_str());
    RCLCPP_INFO(get_logger(), "  capture_target: %s", capture_target_.c_str());
    RCLCPP_INFO(get_logger(), "  capture_mode: %s", capture_mode_.c_str());
    RCLCPP_INFO(get_logger(), "  capture_enabled: %s", capture_enabled_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  video_capture_enabled: %s", video_capture_enabled_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  image_capture_directory: %s", image_capture_dir_.c_str());
    RCLCPP_INFO(get_logger(), "  video_capture_directory: %s", video_capture_dir_.c_str());

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

FrontCameraVision::~FrontCameraVision()
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

void FrontCameraVision::image_callback(
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

        if (capture_enabled_) {
            write_capture_frame(job.image, job.timestamp);
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

void FrontCameraVision::inference_worker()
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
        if (capture_mode_ == "image" && capture_enabled_) {
            const double frame_s = job.timestamp.seconds();
            if (frame_s - last_save_s_ >= 5.0) {
                last_save_s_ = frame_s;
                save_image(job.image, job.timestamp);
            }
        }
        publish_detections(detections, job.timestamp, inference_ms);
    }
}