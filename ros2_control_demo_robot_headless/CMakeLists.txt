cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_robot_headless)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)
find_package(std_msgs REQUIRED)

add_library(generic_robot_hardware SHARED
  src/generic_robot_hardware.cpp)

target_include_directories(generic_robot_hardware
   PRIVATE include
   PUBLIC
   $<INSTALL_INTERFACE:include>
   ${TinyXML_INCLUDE_DIRS}
   )
ament_target_dependencies(
   generic_robot_hardware
   hardware_interface
   rclcpp
   urdf
   std_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(generic_robot_hardware PRIVATE
  "GENERIC_ROBOT_HARDWARE_BUILDING_DLL")

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS generic_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_generic_robot_hardware
    test/test_generic_robot_hardware.cpp)
  if(TARGET test_generic_robot_hardware)
    target_include_directories(test_generic_robot_hardware PRIVATE include)
    target_link_libraries(
      test_generic_robot_hardware
      generic_robot_hardware
    )
  endif()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
