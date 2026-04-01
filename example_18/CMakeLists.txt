cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_18 LANGUAGES C CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Dependencies (matching package.xml)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  control_msgs
  controller_interface
  geometry_msgs
  generate_parameter_library
  hardware_interface
  mujoco_ros2_control
  mujoco_ros2_control_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  transmission_interface
  control_toolbox
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
    "Are you using the correct branch of the ros2_control_demos repository?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VelocityCommandWithHead.msg"
  DEPENDENCIES geometry_msgs
)

# Example 18 requires ONNX Runtime; skip building if not found
find_package(onnxruntime_vendor QUIET)
if(NOT onnxruntime_vendor_FOUND)
  message(FATAL_ERROR
    "Example 18 cannot be built without ONNX Runtime. "
    "Install or build onnxruntime_vendor to build this example.")
endif()

## COMPILE
# Static libs linked into shared controller (require -fPIC)
add_library(observation_formatter STATIC controller/src/observation_formatter.cpp)
set_target_properties(observation_formatter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(observation_formatter PUBLIC cxx_std_17)
target_include_directories(observation_formatter PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/motion_controller>
)
target_link_libraries(observation_formatter
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
)

add_library(action_processor STATIC controller/src/action_processor.cpp)
set_target_properties(action_processor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(action_processor PUBLIC cxx_std_17)
target_include_directories(action_processor PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/motion_controller>
)

generate_parameter_library(motion_controller_parameters
  controller/src/motion_controller_parameters.yaml
)

# Main library (hardware + controller)
add_library(${PROJECT_NAME}_lib SHARED
  hardware/duck_mini_mujoco_system_interface.cpp
  controller/src/motion_controller.cpp
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_lib
  ament_index_cpp::ament_index_cpp
  motion_controller_parameters
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${cpp_typesupport_target}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  control_toolbox::control_toolbox
  transmission_interface::transmission_interface
  ${mujoco_ros2_control_msgs_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
  observation_formatter
  action_processor
  mujoco_ros2_control::mujoco_ros2_control
  hardware_interface::hardware_interface
  controller_interface::controller_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
)
target_link_libraries(${PROJECT_NAME}_lib onnxruntime::onnxruntime)

pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_18.xml)
pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_18.xml)

# INSTALL
install(DIRECTORY controller/include/ DESTINATION include/motion_controller)
install(DIRECTORY hardware/include/ DESTINATION include)
install(DIRECTORY bringup/ DESTINATION share/ros2_control_demo_example_18)
install(DIRECTORY description/launch DESTINATION share/ros2_control_demo_example_18/description)
install(DIRECTORY onnx_model/ DESTINATION share/ros2_control_demo_example_18/onnx_model)
install(TARGETS observation_formatter action_processor motion_controller_parameters ${PROJECT_NAME}_lib
  EXPORT export_ros2_control_demo_example_18
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(mujoco_vendor REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_example_18_mujoco_launch.py)
  add_ros_isolated_launch_test(test/test_view_robot_launch.py)

  ament_add_pytest_test(example_18_urdf_xacro test/test_urdf_xacro.py)

  ament_add_gtest(test_observation_formatter controller/test/test_observation_formatter.cpp)
  target_include_directories(test_observation_formatter PRIVATE ${PROJECT_SOURCE_DIR}/controller/include)
  target_link_libraries(test_observation_formatter
    observation_formatter
    rclcpp::rclcpp
    ${control_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
  )

  ament_add_gtest(test_action_processor controller/test/test_action_processor.cpp)
  target_include_directories(test_action_processor PRIVATE ${PROJECT_SOURCE_DIR}/controller/include)
  target_link_libraries(test_action_processor action_processor)

  ament_add_gtest(test_motion_controller_initialization controller/test/test_motion_controller_initialization.cpp)
  target_include_directories(test_motion_controller_initialization PRIVATE ${PROJECT_SOURCE_DIR}/controller/include)

  ament_add_gtest(test_onnx_shape_handling controller/test/test_onnx_shape_handling.cpp)

  ament_add_gtest(test_motion_controller_mujoco controller/test/test_motion_controller_mujoco.cpp)
  target_include_directories(test_motion_controller_mujoco PRIVATE ${PROJECT_SOURCE_DIR}/controller/include)
  target_link_libraries(test_motion_controller_mujoco mujoco::mujoco)

  ament_add_gtest(test_contact_detection hardware/test/test_contact_detection.cpp)
  target_include_directories(test_contact_detection PRIVATE ${PROJECT_SOURCE_DIR}/hardware/include)
  target_link_libraries(test_contact_detection
    ${PROJECT_NAME}_lib
    mujoco::mujoco
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    hardware_interface::hardware_interface
    mujoco_ros2_control::mujoco_ros2_control
  )

  ament_add_gtest(test_motion_controller_error_handling
    controller/test/test_motion_controller_error_handling.cpp)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_18 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} rosidl_default_runtime onnxruntime_vendor)
ament_package()
