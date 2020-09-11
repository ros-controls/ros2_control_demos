cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

## COMPILE
add_executable(
  ros2_control_manager_node
  src/ros2_control_manager_node.cpp
)
target_include_directories(
  ros2_control_manager_node
  PRIVATE
  include
)
ament_target_dependencies(
  ros2_control_manager_node
  hardware_interface
  pluginlib
  rclcpp
)

## INSTALL
install(
  TARGETS ros2_control_manager_node
  DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY description launch/
  DESTINATION share
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ros2_control_manager_node
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
