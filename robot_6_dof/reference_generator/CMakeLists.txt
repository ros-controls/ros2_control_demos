cmake_minimum_required(VERSION 3.5)
project(reference_generator)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(send_trajectory src/send_trajectory.cpp)

ament_target_dependencies(
    send_trajectory
    ament_cmake
    ament_cmake
    control_msgs
    control_toolbox
    controller_interface
    kdl_parser
    geometry_msgs
    hardware_interface
    joint_trajectory_controller
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    trajectory_msgs
)


install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/reference_generator
)


install(
    DIRECTORY launch
    DESTINATION share
)


ament_package()

