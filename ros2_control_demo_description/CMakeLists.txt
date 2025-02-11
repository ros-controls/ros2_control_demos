cmake_minimum_required(VERSION 3.5)
project(uwrt_ros2_control_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY diffbot/urdf
  DESTINATION share/${PROJECT_NAME}/diffbot
)

ament_package()
