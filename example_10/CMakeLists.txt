cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_10 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  control_msgs
  std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  ros2_control_demo_example_10
  SHARED
  hardware/rrbot.cpp
  controllers/gpio_controller.cpp
)
target_compile_features(ros2_control_demo_example_10 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_10 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_10>
)
ament_target_dependencies(
  ros2_control_demo_example_10 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_10_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_10.xml)
# Export controllers
pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_10.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_10
)
install(
  DIRECTORY controllers/include/
  DESTINATION include/ros2_control_demo_example_10
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_10
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_10
)
install(TARGETS ros2_control_demo_example_10
  EXPORT export_ros2_control_demo_example_10
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_10_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_10_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_10 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
