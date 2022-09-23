cmake_minimum_required(VERSION 3.5)
project(rrbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo meshes launch ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
