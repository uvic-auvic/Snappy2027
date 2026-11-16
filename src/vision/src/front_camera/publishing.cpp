#include "front_camera_vision.cpp"

void FrontCameraVision::publish_detections(const std::vector<Detection> & detections,
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

    std::string class_name_for(int class_id) const
    {
        if (class_id >= 0 && static_cast<size_t>(class_id) < class_names_.size()) {
            return class_names_[class_id];
        }
        return "unknown";
    }