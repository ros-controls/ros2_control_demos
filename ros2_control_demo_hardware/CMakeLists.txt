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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

## COMPILE
add_library(${PROJECT_NAME} SHARED
  src/robot_minimal_hardware.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  hardware_interface
)

pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_hardware.xml)

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
  hardware_interface
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(
  include
)
ament_package()
