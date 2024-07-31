cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_12 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_msgs
  generate_parameter_library
  controller_interface
  parameter_traits
  realtime_tools
  std_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_demo_example_12
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(ros2_control_demo_example_12 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_12 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_12>
)
ament_target_dependencies(
  ros2_control_demo_example_12 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_12.xml)

# Add library of the controller and export it
generate_parameter_library(passthrough_controller_parameters
  controllers/src/passthrough_controller_parameters.yaml
)

add_library(passthrough_controller SHARED
  controllers/src/passthrough_controller.cpp
)
target_include_directories(passthrough_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
$<INSTALL_INTERFACE:include/passthrough_controller>
)
target_link_libraries(passthrough_controller PUBLIC passthrough_controller_parameters)
ament_target_dependencies(passthrough_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface passthrough_controller.xml)


# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_12
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_12
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_12
)
install(TARGETS ros2_control_demo_example_12
  EXPORT export_ros2_control_demo_example_12
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY controllers/include/
  DESTINATION include/passthrough_controller
)

install(TARGETS
    passthrough_controller
    passthrough_controller_parameters
  EXPORT export_passthrough_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_12_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_12_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_12_launch test/test_rrbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_passthrough_controller HAS_LIBRARY_TARGET)
ament_export_targets(export_ros2_control_demo_example_12 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
