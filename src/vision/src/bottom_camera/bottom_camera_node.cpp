/*
    Node.cpp constructs front camera inference node and image callback.
*/

#include "bottom_camera_vision.hpp"

BottomCameraVision : BottomCameraVision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("bottom_camera_vision", options)
{
    // Declare parameters
    inference_hz_ = this->declare_parameter<double>("inference_hz", 0.0);
    high_conf_threshold_ = this->declare_parameter<double>("high_conf_threshold", 0.5);
    engine_path_ = this->declare_parameter<std::string>(
        "engine_path", "/home/kraken/Desktop/dfc_rs_26.engine");
    num_classes_ = this->declare_parameter<int>("num_classes", 8);
    camera_name_ = this->declare_parameter<std::string>("camera_name", "bottom");
    capture_target_ = this->declare_parameter<std::string>("capture_target", "both");
    capture_mode_ = this->declare_parameter<std::string>("capture_mode", "video");
    video_capture_enabled_ = this->declare_parameter<bool>(
        "video_capture_enabled",
        capture_mode_ == "video");
    image_capture_dir_ = this->declare_parameter<std::string>(
        "image_capture_directory", "/home/snappy_data/bottom_camera/images");
    video_capture_dir_ = this->declare_parameter<std::string>(
        "video_capture_directory", "/home/snappy_data/bottom_camera/videos");
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

    RCLCPP_INFO(get_logger(), "Bottom Camera Vision (YOLOv26 Seg) initializing...");
    RCLCPP_INFO(get_logger(), "  inference_hz: %.1f", inference_hz_);
    RCLCPP_INFO(get_logger(), "  conf_threshold: %.2f", high_conf_threshold_);
    RCLCPP_INFO(get_logger(), "  engine_path: %s", engine_path_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_name: %s", camera_name_.c_str());
    RCLCPP_INFO(get_logger(), "  capture_target: %s", capture_target_.c_str());
    RCLCPP_INFO(get_logger(), "  capture_mode: %s", capture_mode_.c_str());
    RCLCPP_INFO(get_logger(), "  capture_enabled: %s", capture_enabled_ ? "true" : "false");        RCLCPP_INFO(get_logger(), "  image_capture_directory: %s", image_capture_dir_.c_str());        RCLCPP_INFO(get_logger(), "  video_capture_directory: %s", video_capture_dir_.c_str());

    init_tensorrt();

    // Subscribe to D405 color and aligned depth feeds together so objects get range estimate from the same frame
    color_sub_.subscribe(
        this, "/d405/d405/color/image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    depth_sub_.subscribe(
        this, "/d405/d405/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10));
    sync_->connectInput(color_sub_, depth_sub_);
    sync_->registerCallback(
        std::bind(&BottomCameraVision::image_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

    // Publish detections
    detection_pub_ = this->create_publisher<snappy_interfaces::msg::DetectionArray>(
        "/d405/detections", 10);

    // Start inference worker
    worker_running_ = true;
    worker_thread_ = std::thread(&BottomCameraVision::inference_worker, this);

    RCLCPP_INFO(get_logger(), "Bottom Camera Vision ready");
}

~BottomCameraVision() override
{
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