cmake_minimum_required(VERSION 3.8)
project(snappy_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pressureSensor src/pressure_sensor.cpp)
ament_target_dependencies(pressureSensor rclcpp std_msgs)

set(driver_targets pressureSensor)

# The solenoid node is Jetson-specific. Build it whenever the libgpiod C++
# development files are installed, while keeping non-hardware workspaces
# buildable without them.
find_path(GPIODCXX_INCLUDE_DIR gpiod.hpp)
find_library(GPIODCXX_LIBRARY NAMES gpiodcxx)

if(GPIODCXX_INCLUDE_DIR AND GPIODCXX_LIBRARY)
  add_executable(solenoid_channel src/solenoid_channel.cpp)
  target_include_directories(solenoid_channel PRIVATE ${GPIODCXX_INCLUDE_DIR})
  ament_target_dependencies(solenoid_channel rclcpp std_msgs)
  target_link_libraries(solenoid_channel ${GPIODCXX_LIBRARY})
  list(APPEND driver_targets solenoid_channel)
else()
  message(WARNING
    "libgpiod C++ development files were not found; solenoid_channel will not be built")
endif()

install(
  TARGETS ${driver_targets}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
