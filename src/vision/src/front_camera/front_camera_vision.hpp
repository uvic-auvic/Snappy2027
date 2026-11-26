class FrontCameraVision : public CameraVision
{
public:
    explicit FrontCameraVision(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    ~FrontCameraVision() override;

private:
   
    void init_tensorrt();

    // --- TRT Logger --------------------------------------------------------

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

    // Video capture
    std::string image_capture_dir_ = "/home/snappy_data/front_camera/images";
    std::string video_capture_dir_ = "/home/snappy_data/front_camera/videos";

    // Threading
    std::deque<InferenceJob>   job_queue_;
    std::mutex                 queue_mutex_;
    std::condition_variable    queue_cv_;
    std::thread                worker_thread_;
    std::atomic<bool>          worker_running_{false};

    // Parameters
    double      inference_hz_         = 10.0;
    double      high_conf_threshold_  = 0.5;
    double      low_conf_threshold_;
    int         num_classes_     = 80;
    std::string camera_name_ = "front";
    std::string capture_target_ = "both";
    std::string capture_mode_ = "video";
    bool capture_enabled_ = true;
    std::string engine_path_;
    std::optional<rclcpp::Time> last_inference_time_;
    double last_save_s_ = 0.0;

    int mask_h_ = 0, mask_w_ = 0;

    // FFC (front camera) classes, indexed by class_id (see detection_classes.hpp).
    // class_name_for() returns "unknown" for out-of-range ids.
    std::vector<std::string> class_names_ = snappy_computer_vision::kFrontDetectionClasses;
};