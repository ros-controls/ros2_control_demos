cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_robot)

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
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_components REQUIRED)

find_package(ros2_control_demo_hardware REQUIRED)

## COMPILE
add_executable(${PROJECT_NAME}_minimal_node
  src/${PROJECT_NAME}_minimal_node.cpp
)

target_include_directories(${PROJECT_NAME}_minimal_node
  PRIVATE include
)

ament_target_dependencies(${PROJECT_NAME}_minimal_node
  control_msgs
  hardware_interface
  rclcpp
  robot_control_components
  ros2_control_demo_hardware
)

## INSTALL
install(TARGETS ${PROJECT_NAME}_minimal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ description/ launch/
  DESTINATION share
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dummy test/test_dummy.cpp)
  target_include_directories(test_dummy PRIVATE include)
#   ament_target_dependencies(test_dummy rcpputils)

endif()

## EXPORTS
ament_package()
