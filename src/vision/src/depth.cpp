#include "front_camera_vision.cpp"

CameraVision::float estimate_distance_m(
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