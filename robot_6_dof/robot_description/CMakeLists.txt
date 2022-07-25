cmake_minimum_required(VERSION 3.5)
project(robot_description)

# find dependencies
find_package(ament_cmake REQUIRED)


install(
    DIRECTORY launch meshes urdf srdf rviz
    DESTINATION share/robot_description
)


ament_package()

