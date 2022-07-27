cmake_minimum_required(VERSION 3.5)
project(r6bot_hardware)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(
    r6bot_hardware
    SHARED
    src/r6bot_hardware.cpp
)

target_include_directories(
    r6bot_hardware
    PUBLIC
    include
)

ament_target_dependencies(
    r6bot_hardware
    pluginlib
    rcpputils
    hardware_interface
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(r6bot_hardware PRIVATE "ROBOT_HARDWARE_DLL")

pluginlib_export_plugin_description_file(hardware_interface r6bot_hardware_plugin_description.xml)

install(
    DIRECTORY include
    DESTINATION include
)

install(
    TARGETS r6bot_hardware
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_package()
