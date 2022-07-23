cmake_minimum_required(VERSION 3.5)
project(6_dof_robot)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY config launch meshes rviz urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

