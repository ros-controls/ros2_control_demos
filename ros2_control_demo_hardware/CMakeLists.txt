cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control_core REQUIRED)

## COMPILE
add_library(${PROJECT_NAME} SHARED
  src/demo_robot_hardware_headless.cpp
  src/demo_robot_hardware_minimal.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  ros2_control_core
)

pluginlib_export_plugin_description_file(ros2_control_hardware ros2_control_demo_hardware_headless.xml)

# INSTALL
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dummy test/test_dummy.cpp)
  target_include_directories(test_dummy PRIVATE include)
#   ament_target_dependencies(test_dummy rcpputils)

endif()

## EXPORTS
ament_export_dependencies(
  pluginlib
  rclcpp
  ros2_control_core
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(
  include
)
ament_package()
