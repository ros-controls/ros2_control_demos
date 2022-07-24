cmake_minimum_required(VERSION 3.5)
project(robot_6_dof)

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
    DIRECTORY config launch meshes rviz urdf srdf
    DESTINATION share/robot_6_dof
)

install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/robot_6_dof
)

#add_subdirectory(robot_6_dof_hardware)




find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(
    robot_6_dof_hardware
    SHARED
    src/robot_hardware.cpp
)

target_include_directories(
    robot_6_dof_hardware
    PUBLIC
    include
)

ament_target_dependencies(
    robot_6_dof_hardware
    pluginlib
    rcpputils
    hardware_interface
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(robot_6_dof_hardware PRIVATE "ROBOT_HARDWARE_DLL")

pluginlib_export_plugin_description_file(hardware_interface robot_6_dof_hardware_plugin_description.xml)

install(
    DIRECTORY include
    DESTINATION include
)


install(
    TARGETS robot_6_dof_hardware
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)





ament_package()

