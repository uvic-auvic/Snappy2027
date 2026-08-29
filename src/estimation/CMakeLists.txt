cmake_minimum_required(VERSION 3.8)
project(snappy_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snappy_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(kalman_filter src/kalman.cpp)
target_include_directories(kalman_filter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

add_executable(state_estimator src/state_estimator.cpp)
ament_target_dependencies(state_estimator
  nav_msgs
  rclcpp
  sensor_msgs
  snappy_interfaces
  std_msgs
  tf2
)
target_link_libraries(state_estimator kalman_filter Eigen3::Eigen)

# Offline replay utility retained from the source package. It requires recorded
# estimator CSVs supplied at runtime; no large recordings are stored here.
add_executable(test_kalman test/test_kalman.cpp)
target_link_libraries(test_kalman kalman_filter Eigen3::Eigen)

install(
  TARGETS kalman_filter
  EXPORT export_kalman_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS state_estimator test_kalman
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_kalman_filter HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
