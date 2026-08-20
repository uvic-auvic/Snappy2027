// displayFeed.cpp
// Standalone manual-launch visualization node. Subscribes to BOTH the front
// (D455) and bottom (D405) camera image + detection topics, annotates each
// frame independently, stacks them vertically, and shows the combined result
// in a single OpenCV window.
//
// Image and detection streams are subscribed INDEPENDENTLY (no time-sync):
// the camera publishes color frames at ~30 fps while inference runs slower, so
// requiring a matched image+detection pair (the old ApproximateTime approach)
// dropped most frames and made the video look frozen / "no signal". Instead we
// cache the latest image and the latest detections per camera, and overlay the
// newest detections on the newest frame on a fixed render timer. Video stays
// smooth at camera rate; boxes appear best-effort and are tagged with their age.
//
// Run manually (NOT part of the main launch file):
//   ros2 run snappy_computer_vision displayFeed

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include "snappy_interfaces/msg/detection_array.hpp"

class DisplayFeed : public rclcpp::Node
{
public:
    DisplayFeed()
    : Node("display_feed")
    {
        display_window_name_ = "snappy_camera_feeds";
        display_width_ = this->declare_parameter<int>("display_width", 960);
        // Detections older than this (relative to now) are considered stale and
        // shown dimmed, so a frozen/dead inference node is visually obvious.
        detection_timeout_s_ = this->declare_parameter<double>("detection_timeout_s", 1.0);

        cv::namedWindow(display_window_name_, cv::WINDOW_AUTOSIZE);

        // Front camera (D455) -- image and detections subscribed independently.
        front_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d455/color/image_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
                std::lock_guard<std::mutex> lock(front_mutex_);
                front_image_ = msg;
            });
        front_detection_sub_ = this->create_subscription<snappy_interfaces::msg::DetectionArray>(
            "/d455/detections", rclcpp::QoS(10),
            [this](const snappy_interfaces::msg::DetectionArray::ConstSharedPtr & msg) {
                std::lock_guard<std::mutex> lock(front_mutex_);
                front_det_ = msg;
            });

        // Bottom camera (D405).
        bottom_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/d405/color/image_rect_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
                std::lock_guard<std::mutex> lock(bottom_mutex_);
                bottom_image_ = msg;
            });
        bottom_detection_sub_ = this->create_subscription<snappy_interfaces::msg::DetectionArray>(
            "/d405/detections", rclcpp::QoS(10),
            [this](const snappy_interfaces::msg::DetectionArray::ConstSharedPtr & msg) {
                std::lock_guard<std::mutex> lock(bottom_mutex_);
                bottom_det_ = msg;
            });

        // Render on a timer rather than directly in each callback, so a slow
        // or missing feed on one camera doesn't block updates from the other.
        render_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30Hz display refresh
            std::bind(&DisplayFeed::render_combined, this));

        RCLCPP_INFO(get_logger(), "DisplayFeed ready (front: /d455, bottom: /d405)");
    }

    ~DisplayFeed() override
    {
        cv::destroyWindow(display_window_name_);
    }

private:
    // ---- Per-camera annotation (shared logic) ------------------------------
    // Draws the latest detections over the latest frame. `det_msg` may be null
    // (no detections received yet) or stale (older than the timeout) -- in both
    // cases the live video is still shown; stale boxes are simply not drawn.

    cv::Mat annotate_frame(
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
        const snappy_interfaces::msg::DetectionArray::ConstSharedPtr & det_msg)
    {
        cv::Mat annotated;
        try {
            annotated = cv_bridge::toCvShare(image_msg, "bgr8")->image.clone();
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return cv::Mat();
        }

        // Is the cached detection set fresh enough to overlay?
        bool det_fresh = false;
        double det_age_s = -1.0;
        size_t det_count = 0;
        uint32_t inf_ms = 0;
        if (det_msg) {
            const rclcpp::Time det_stamp(det_msg->header.stamp);
            det_age_s = (this->get_clock()->now() - det_stamp).seconds();
            det_fresh = det_age_s >= 0.0 && det_age_s <= detection_timeout_s_;
            det_count = det_msg->detections.size();
            inf_ms = det_msg->inference_time_ms;
        }

        if (det_fresh) {
            for (const auto & d : det_msg->detections) {
                cv::Rect rect(
                    static_cast<int>(d.bounding_box.x),
                    static_cast<int>(d.bounding_box.y),
                    static_cast<int>(d.bounding_box.width),
                    static_cast<int>(d.bounding_box.height));
                cv::rectangle(annotated, rect, cv::Scalar(0, 255, 0), 2);

                for (const auto & poly : d.mask_polygons) {
                    if (poly.points.size() < 2) continue;
                    std::vector<cv::Point> pts;
                    pts.reserve(poly.points.size());
                    for (const auto & p : poly.points) {
                        pts.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));
                    }
                    const cv::Point * pts_data = pts.data();
                    int npts = static_cast<int>(pts.size());
                    cv::polylines(annotated, &pts_data, &npts, 1, true, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
                }

                std::string distance_str = (d.distance_m > 0.0f)
                    ? std::to_string(static_cast<int>(d.distance_m * 100.0f)) + "cm"
                    : "N/A";
                std::string label = d.object_class + " " +
                    std::to_string(static_cast<int>(d.confidence * 100.0f)) + "% [" + distance_str + "]";

                int baseLine = 0;
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseLine);
                int textX = static_cast<int>(d.bounding_box.x);
                int textY = static_cast<int>(d.bounding_box.y) - 5;

                cv::rectangle(annotated,
                    cv::Point(textX, textY - textSize.height - baseLine - 4),
                    cv::Point(textX + textSize.width + 4, textY + baseLine),
                    cv::Scalar(0, 255, 0), cv::FILLED);
                cv::putText(annotated, label, cv::Point(textX + 2, textY - baseLine - 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            }
        }

        // Status line. Yellow when detections are fresh; red when stale/missing
        // so a dead inference node is obvious even though video keeps flowing.
        std::string info;
        cv::Scalar info_color;
        if (det_msg && det_fresh) {
            info = "det: " + std::to_string(det_count) +
                   "  inference: " + std::to_string(inf_ms) + "ms";
            info_color = cv::Scalar(0, 255, 255);
        } else if (det_msg) {
            info = "det: STALE (" + std::to_string(static_cast<int>(det_age_s * 1000)) + "ms old)";
            info_color = cv::Scalar(0, 0, 255);
        } else {
            info = "det: waiting...";
            info_color = cv::Scalar(0, 0, 255);
        }
        cv::putText(annotated, info, {10, 30},
            cv::FONT_HERSHEY_SIMPLEX, 0.8, info_color, 2, cv::LINE_AA);

        return annotated;
    }

    // ---- Combine + display --------------------------------------------------

    void render_combined()
    {
        sensor_msgs::msg::Image::ConstSharedPtr front_img, bottom_img;
        snappy_interfaces::msg::DetectionArray::ConstSharedPtr front_det, bottom_det;
        {
            std::lock_guard<std::mutex> lock(front_mutex_);
            front_img = front_image_;
            front_det = front_det_;
        }
        {
            std::lock_guard<std::mutex> lock(bottom_mutex_);
            bottom_img = bottom_image_;
            bottom_det = bottom_det_;
        }

        cv::Mat front_frame = front_img ? annotate_frame(front_img, front_det) : cv::Mat();
        cv::Mat bottom_frame = bottom_img ? annotate_frame(bottom_img, bottom_det) : cv::Mat();

        // Note: deliberately NOT returning early when both are empty -- the
        // window should appear immediately on startup showing "no signal"
        // placeholders, rather than waiting for the first frame.
        cv::Mat front_resized = resize_to_width(front_frame, display_width_, "FRONT (D455)");
        cv::Mat bottom_resized = resize_to_width(bottom_frame, display_width_, "BOTTOM (D405)");

        cv::Mat combined;
        cv::vconcat(front_resized, bottom_resized, combined);

        cv::imshow(display_window_name_, combined);
        if (cv::waitKey(1) == 27) {
            RCLCPP_INFO(get_logger(), "ESC pressed, shutting down");
            rclcpp::shutdown();
        }
    }

    // Resize (or synthesize a placeholder if empty) to a fixed display width,
    // preserving aspect ratio, and label which camera it is.
    cv::Mat resize_to_width(const cv::Mat & frame, int target_width, const std::string & label) const
    {
        cv::Mat out;
        if (frame.empty()) {
            // No frame received yet for this camera -- show a placeholder
            // rather than skipping it, so the stack size stays consistent.
            out = cv::Mat(target_width * 9 / 16, target_width, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::putText(out, label + " - no signal", {20, out.rows / 2},
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            return out;
        }

        double scale = static_cast<double>(target_width) / frame.cols;
        cv::resize(frame, out, cv::Size(target_width, static_cast<int>(frame.rows * scale)));
        cv::putText(out, label, {10, 20},
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        return out;
    }

    std::string display_window_name_;
    int display_width_ = 960;
    double detection_timeout_s_ = 1.0;

    // Independent subscriptions; latest message of each cached under a per-camera mutex.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_image_sub_;
    rclcpp::Subscription<snappy_interfaces::msg::DetectionArray>::SharedPtr front_detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bottom_image_sub_;
    rclcpp::Subscription<snappy_interfaces::msg::DetectionArray>::SharedPtr bottom_detection_sub_;

    rclcpp::TimerBase::SharedPtr render_timer_;

    std::mutex front_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr front_image_;
    snappy_interfaces::msg::DetectionArray::ConstSharedPtr front_det_;

    std::mutex bottom_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr bottom_image_;
    snappy_interfaces::msg::DetectionArray::ConstSharedPtr bottom_det_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisplayFeed>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
