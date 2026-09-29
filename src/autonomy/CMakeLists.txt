cmake_minimum_required(VERSION 3.8)
project(snappy_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snappy_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(planner src/planner.cpp)
ament_target_dependencies(planner
  rclcpp
  snappy_interfaces
  std_msgs
  yaml_cpp_vendor
)
target_link_libraries(planner Eigen3::Eigen)

install(
  TARGETS planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
