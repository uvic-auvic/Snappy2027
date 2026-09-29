cmake_minimum_required(VERSION 3.8)
project(snappy_computer_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WITH_CUDA "Build with CUDA and TensorRT support" OFF)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snappy_interfaces REQUIRED)

add_executable(image_capture src/image_capture.cpp)
ament_target_dependencies(image_capture rclcpp sensor_msgs cv_bridge)
target_link_libraries(image_capture ${OpenCV_LIBS})

if(WITH_CUDA)
  find_package(CUDA REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(message_filters REQUIRED)

  if(POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
  endif()
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)
  endif()
  enable_language(CUDA)

  find_library(NVINFER nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
  find_library(NVONNXPARSER nvonnxparser
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
    REQUIRED
  )

  foreach(camera_node front_camera_vision bottom_camera_vision)
    add_executable(${camera_node}
      src/${camera_node}.cpp
      src/preprocess_cuda.cu
    )
    target_include_directories(${camera_node} PRIVATE
      include
      /usr/lib/aarch64-linux-gnu
      /usr/local/cuda/include
    )
    set_target_properties(${camera_node} PROPERTIES
      CUDA_STANDARD 17
      CUDA_STANDARD_REQUIRED ON
    )
    ament_target_dependencies(${camera_node}
      cv_bridge
      geometry_msgs
      message_filters
      rclcpp
      sensor_msgs
      snappy_interfaces
    )
    target_link_libraries(${camera_node}
      ${NVINFER}
      ${NVONNXPARSER}
      ${CUDA_LIBRARIES}
      ${OpenCV_LIBS}
    )
  endforeach()

  add_executable(displayFeed src/displayFeed.cpp)
  ament_target_dependencies(displayFeed
    cv_bridge
    message_filters
    rclcpp
    sensor_msgs
    snappy_interfaces
  )
  target_link_libraries(displayFeed ${OpenCV_LIBS})

  install(
    TARGETS front_camera_vision bottom_camera_vision displayFeed
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  TARGETS image_capture
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
