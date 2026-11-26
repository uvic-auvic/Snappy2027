// bottom camera.cpp
// Headless YOLOv26 segmentation inference node for D405 camera
// Publishes DetectionArray for external visualization node

#include "bottom_camera_vision.hpp"

std::vector<std::string> class_names_ = snappy_computer_vision::kBottomDetectionClasses;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottomCameraVision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
