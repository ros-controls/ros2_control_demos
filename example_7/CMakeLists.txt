cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_7 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  controller_interface
  realtime_tools
  trajectory_msgs
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

target_link_libraries(send_trajectory PUBLIC
  ${trajectory_msgs_TARGETS}
  kdl_parser::kdl_parser
  rclcpp::rclcpp
)

add_library(
  ros2_control_demo_example_7
  SHARED
  hardware/r6bot_hardware.cpp
  controller/r6bot_controller.cpp
)

target_compile_features(ros2_control_demo_example_7 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_7 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_7>
)
target_link_libraries(ros2_control_demo_example_7 PUBLIC
  ${trajectory_msgs_TARGETS}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_7.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_7.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_7
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_7
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_7
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/ros2_control_demo_example_7
)

install(TARGETS ros2_control_demo_example_7
  EXPORT export_ros2_control_demo_example_7
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)

  # Integration (launch) tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_view_robot_launch.py)
  add_ros_isolated_launch_test(test/test_r6bot_controller_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_7 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
