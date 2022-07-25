cmake_minimum_required(VERSION 3.5)
project(robot_6_dof_hardware)

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

