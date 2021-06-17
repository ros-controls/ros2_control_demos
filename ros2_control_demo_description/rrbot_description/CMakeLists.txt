cmake_minimum_required(VERSION 3.5)
project(rrbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo ros2_control launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
