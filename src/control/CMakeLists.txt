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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(snappy_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(control_algorithms
  src/pid.cpp
  src/thruster_allocator.cpp
  src/rov_input.cpp
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
add_library(rov_control src/rov_control.cpp)
target_include_directories(rov_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rov_control rclcpp snappy_interfaces std_msgs std_srvs)
target_link_libraries(rov_control control_algorithms)
target_link_libraries(controller rov_control control_algorithms Eigen3::Eigen)

# Separate operating configuration: there is no runtime mode switch.
add_executable(rov_depth_controller src/controller.cpp)
target_compile_definitions(rov_depth_controller PRIVATE SNAPPY_ROV_CONFIGURATION)
ament_target_dependencies(rov_depth_controller
  geometry_msgs nav_msgs rclcpp snappy_interfaces std_msgs tf2 tf2_geometry_msgs)
target_link_libraries(rov_depth_controller rov_control control_algorithms Eigen3::Eigen)

add_executable(rov_controller src/rov_controller.cpp)
target_include_directories(rov_controller PRIVATE include)
ament_target_dependencies(rov_controller rclcpp sensor_msgs snappy_interfaces std_msgs)
target_link_libraries(rov_controller control_algorithms)

install(
  TARGETS control_algorithms rov_control
  EXPORT export_control_algorithms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS controller rov_controller rov_depth_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_control_algorithms HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp snappy_interfaces std_msgs std_srvs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rov_input test/test_rov_input.cpp)
  target_link_libraries(test_rov_input control_algorithms)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_rov_system.py TIMEOUT 90)
endif()

ament_package()
