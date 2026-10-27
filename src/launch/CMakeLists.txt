cmake_minimum_required(VERSION 3.10)
project(snappy_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
