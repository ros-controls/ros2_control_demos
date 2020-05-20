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
find_package(rclcpp REQUIRED)
find_package(ros2_control_core REQUIRED)


# Test node
add_executable(ros2_control_demo_robot_manager src/ros2_control_demo_robot_manager.cpp)

target_include_directories(ros2_control_demo_robot_manager
  PRIVATE include
)

ament_target_dependencies(ros2_control_demo_robot_manager
  control_msgs
  rclcpp
  ros2_control_core
)

## INSTALL
install(TARGETS ros2_control_demo_robot_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dummy test/test_dummy.cpp)
  target_include_directories(test_dummy PRIVATE include)
#   ament_target_dependencies(test_dummy rcpputils)

endif()

ament_package()
