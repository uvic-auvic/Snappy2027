cmake_minimum_required(VERSION 3.8)
project(snappy_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snappy_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(bt_planner src/planner.cpp)

ament_target_dependencies(
  bt_planner
  rclcpp
  snappy_interfaces
  std_msgs
)
target_link_libraries(bt_planner behaviortree_cpp::behaviortree_cpp)

install(
  TARGETS bt_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES src/bhvr_tree.xml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
