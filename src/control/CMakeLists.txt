cmake_minimum_required(VERSION 3.8)
project(snappy_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snappy_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(control_algorithms
  src/pid.cpp
  src/thruster_allocator.cpp
)
target_include_directories(control_algorithms
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(control_algorithms PUBLIC Eigen3::Eigen)

add_executable(controller src/controller.cpp)
target_include_directories(controller PRIVATE include)
ament_target_dependencies(controller
  geometry_msgs
  nav_msgs
  rclcpp
  snappy_interfaces
  std_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(controller control_algorithms Eigen3::Eigen)

install(
  TARGETS control_algorithms
  EXPORT export_control_algorithms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_control_algorithms HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
