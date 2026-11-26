// front_camera_vision.cpp
// Headless YOLOv26 segmentation inference node for D455 camera
// Publishes DetectionArray for external visualization nodes

#include "front_camera_vision.hpp"

std::vector<std::string> class_names_ = snappy_computer_vision::kFrontDetectionClasses;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontCameraVision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
