cmake_minimum_required(VERSION 3.5)
project(robot_controller)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rcpputils REQUIRED)

add_library(
    robot_controller
    SHARED
    src/robot_controller.cpp
)

target_include_directories(
    robot_controller
    PRIVATE
    include
)

ament_target_dependencies(
    robot_controller
    pluginlib
    rcpputils
    controller_interface
    realtime_tools
    trajectory_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(robot_controller PRIVATE "ROBOT_CONTROLLER_DLL")

pluginlib_export_plugin_description_file(controller_interface robot_6_dof_controller_plugin_description.xml)

install(
    DIRECTORY include
    DESTINATION include
)


install(
    TARGETS robot_controller
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY config launch
    DESTINATION share/robot_controller
)

ament_package()

