cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_franka)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(iDynTree REQUIRED)

include_directories(
  include
)

# create library for the controller
add_library(franka_ik_controller src/fraka_controller.cpp)

# create the node executable
add_executable(ros2_control_demo_franka src/franka_controller_node.cpp)
target_link_libraries(ros2_control_demo_franka franka_ik_controller ${iDynTree_LIBRARIES})

## INSTALL
install(
  TARGETS franka_ik_controller ros2_control_demo_franka
  DIRECTORY config description launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

## EXPORTS
ament_package()
