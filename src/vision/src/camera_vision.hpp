#pragma once

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
#include <iostream>

class CameraVision : public rclcpp::Node
{
public:
    explicit FrontCameraVision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    ~FrontCameraVision() override;

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
    
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;


    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

    void inference_worker();
    
    void write_capture_frame(const cv::Mat &image, const rclcpp::Time &timestamp);
   
    void start_video_capture(const cv::Mat &frame);
    
    void save_image(const cv::Mat &image, const rclcpp::Time &timestamp);
    
    std::vector<Detection> run_inference(const cv::Mat & image, float & inference_ms);

    void letterbox(const cv::Mat & src, cv::Mat & dst, float & scale, int & pad_x, int & pad_y);
     std::vector<Detection> parse_segmentation_output(
        const float * output0, const float * output1,
        int orig_w, int orig_h, float scale, int pad_x, int pad_y);

    std::vector<cv::Point2f> decode_mask_polygon(
        const std::vector<float> & mask_coeffs,
        const float * proto,
        const Detection & det,
        int orig_w, int orig_h,
        int pad_x, int pad_y,
        float mask_threshold);
        static float compute_iou(const Anchor & a, const Anchor & b);

    float estimate_distance_m(
        const cv::Mat & depth_image,
        const std::string & depth_encoding,
        const Detection & det);

    void publish_detections(const std::vector<Detection> & detections,
                            const rclcpp::Time & timestamp,
                            float inference_ms);

    std::string class_name_for(int class_id) const;
    
    // ---- Members -----------------------------------------------------------

    std::string input_name_, output0_name_, output1_name_;
    int    input_w_ = 640, input_h_ = 640;
    size_t output0_size_ = 0, output1_size_ = 0;
    int    num_detections_ = 0, det_attrs_ = 0;

    // Video capture
    bool video_capture_enabled_ = true;
    std::string image_capture_dir_;
    std::string video_capture_dir_;
    double video_capture_fps_ = 30.0;
    std::string video_path_;
    cv::VideoWriter video_writer_;
    std::mutex video_mutex_;

    // Parameters
    double      inference_hz_;
    double      high_conf_threshold_;
    double      low_conf_threshold_;
    int         num_classes_;
    std::string camera_name_;
    std::string capture_target_;
    std::string capture_mode_ =;
    bool capture_enabled_ = true;
    std::string engine_path_;
    std::optional<rclcpp::Time> last_inference_time_;
    double last_save_s_ = 0.0;

    int mask_h_ = 0, mask_w_ = 0;
    
    std::vector<std::string> class_names_;
};