cmake_minimum_required(VERSION 3.5)
project(robot_6_dof)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY config launch meshes rviz urdf srdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

