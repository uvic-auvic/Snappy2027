// detection_classes.hpp
// Class-id -> name lookup tables for the front and bottom camera
// segmentation models. class_id (as produced by parse_segmentation_output())
// indexes directly into these arrays.
//
// Usage in front_camera_vision.cpp:
//   #include "snappy_computer_vision/detection_classes.hpp"
//   ...
//   std::vector<std::string> class_names_ = snappy_computer_vision::kFrontDetectionClasses;
//
// Usage in bottom_camera_vision.cpp:
//   std::vector<std::string> class_names_ = snappy_computer_vision::kBottomDetectionClasses;

#pragma once

#include <string>
#include <vector>

namespace snappy_computer_vision
{

// Front camera (FFC) classes.
// NOTE: fire_engine and ambulance were not annotated, despite appearing
// in some upstream COCO-derived class lists -- do not add them here.
inline const std::vector<std::string> kFrontDetectionClasses = {
    "blood",               // 0
    "buoy",                // 1
    "compass",             // 2
    "circle",              // 3
    "fire",                // 4
    "hammer_and_wrench",   // 5
    "slalom",              // 6
    "sos",                 // 7
};

// Bottom camera (DFC) classes.
inline const std::vector<std::string> kBottomDetectionClasses = {
    "bandage",      // 0
    "blood",        // 1
    "fire",         // 2
    "helmet",       // 3
    "nut_and_bolt", // 4
    "pill",         // 5
    "plug",         // 6
    "warning",      // 7
};

}  // namespace snappy_computer_vision
