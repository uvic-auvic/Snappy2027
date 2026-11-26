#include "front_camera_vision.hpp"

void CameraVision::write_capture_frame(const cv::Mat &image, const rclcpp::Time &timestamp)
    {
        if (capture_mode_ == "video") {
            std::lock_guard<std::mutex> video_lock(video_mutex_);
            if (!video_writer_.isOpened()) {
                start_video_capture(image);
            }
            if (video_writer_.isOpened()) {
                video_writer_ << image;
            }
            return;
        }

        const double frame_s = timestamp.seconds();
        if (frame_s - last_save_s_ >= 5.0) {
            last_save_s_ = frame_s;
            save_image(image, timestamp);
        }
    }

void CameraVision::start_video_capture(const cv::Mat &frame)
    {
        //std::filesystem::create_directories(video_capture_dir_, std::error_code{});
        std::error_code ec;
        std::filesystem::create_directories(this->video_capture_dir_, ec);

        const auto now = std::chrono::system_clock::now();
        const auto tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
        localtime_r(&tt, &tm_buf);

        std::ostringstream timestamp;
        timestamp << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        video_path_ = (std::filesystem::path(this->video_capture_dir_) / (this->camera_name_ + timestamp.str() + ".mp4")).string();

        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        video_writer_ = cv::VideoWriter(video_path_, fourcc, video_capture_fps_, cv::Size(frame.cols, frame.rows));
        if (!video_writer_.isOpened()) {
            RCLCPP_WARN(get_logger(), "Failed to open video writer at %s", video_path_.c_str());
            video_capture_enabled_ = false;
            return;
        }

        RCLCPP_INFO(get_logger(), "Video capture enabled: %s", video_path_.c_str());
    }


// take image and save to desktop
void CameraVision::save_image(const cv::Mat &image, const rclcpp::Time &timestamp)
    {
        std::error_code ec;
        std::filesystem::create_directories(this->image_capture_dir_, ec);

        const std::filesystem::path path =
            std::filesystem::path(this->image_capture_dir_) /
            (this->camera_name_ + std::to_string(timestamp.seconds()) + ".jpg");

        if (!cv::imwrite(path.string(), image)) {
            RCLCPP_WARN(
                get_logger(),
                "save_image: failed to write %s",
                path.string().c_str());
        }
    }