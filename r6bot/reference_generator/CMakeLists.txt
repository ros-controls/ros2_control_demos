cmake_minimum_required(VERSION 3.5)
project(reference_generator)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(send_trajectory src/send_trajectory.cpp)

ament_target_dependencies(
    send_trajectory
    ament_cmake
    kdl_parser
    geometry_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
    trajectory_msgs
)


install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/reference_generator
)


install(
    DIRECTORY launch
    DESTINATION share/reference_generator
)


ament_package()
