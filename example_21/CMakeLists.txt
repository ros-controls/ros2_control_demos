# The ExecuTorch CMake package defines its imported targets only on CMake 3.28 or newer.
cmake_minimum_required(VERSION 3.28)
project(ros2_control_demo_example_21 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ros2_control_demo_example_18
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
    "Are you using the correct branch of the ros2_control_demos repository?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# executorch_vendor points find_package(executorch) at the ExecuTorch C++ libraries it provides.
find_package(executorch_vendor QUIET)
if(NOT executorch_vendor_FOUND)
  message(FATAL_ERROR
    "Example 21 cannot be built without executorch_vendor. "
    "Clone https://github.com/ros2/rosidl_buffer_backends.git into your workspace src directory "
    "(until its pull request 36 is merged, run "
    "git fetch origin pull/36/head && git checkout FETCH_HEAD in it) "
    "and build it (colcon build --packages-select executorch_vendor), "
    "then source the workspace before building example_21.")
endif()
find_package(executorch ${EXECUTORCH_VENDOR_VERSION_RANGE} CONFIG REQUIRED)

## COMPILE
add_library(executorch_policy STATIC controller/src/executorch_policy.cpp)
set_target_properties(executorch_policy PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(executorch_policy PUBLIC cxx_std_17)
target_include_directories(executorch_policy PUBLIC
  ${PROJECT_SOURCE_DIR}/controller/include
)
target_link_libraries(executorch_policy PUBLIC
  executorch::runtime
  executorch::threadpool
  executorch::backend_xnnpack
)

generate_parameter_library(executorch_policy_controller_parameters
  controller/src/executorch_policy_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED controller/src/executorch_policy_controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  executorch_policy
  executorch_policy_controller_parameters
  ament_index_cpp::ament_index_cpp
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${control_msgs_TARGETS}
  ros2_control_demo_example_18::observation_formatter
  ros2_control_demo_example_18::action_processor
  ros2_control_demo_example_18::ros2_control_demo_example_18__rosidl_typesupport_cpp
)

pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_21.xml)

# INSTALL
install(DIRECTORY bringup/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY executorch_model/ DESTINATION share/${PROJECT_NAME}/executorch_model
  FILES_MATCHING PATTERN "*.pte"
)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(onnxruntime_vendor REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_example_21_mujoco_launch.py)

  ament_add_gtest(test_executorch_policy controller/test/test_executorch_policy.cpp)
  target_link_libraries(test_executorch_policy executorch_policy onnxruntime::onnxruntime)
  target_compile_definitions(test_executorch_policy PRIVATE
    PTE_MODEL_PATH="${PROJECT_SOURCE_DIR}/executorch_model/open_duck_mini_policy.pte"
    ONNX_MODEL_PATH="${ros2_control_demo_example_18_DIR}/../onnx_model/open_duck_mini_policy.onnx")

  ament_add_gtest(test_executorch_policy_controller
    controller/test/test_executorch_policy_controller.cpp)
  target_link_libraries(test_executorch_policy_controller ${PROJECT_NAME})
  target_compile_definitions(test_executorch_policy_controller PRIVATE
    PTE_MODEL_PATH="${PROJECT_SOURCE_DIR}/executorch_model/open_duck_mini_policy.pte")

  ament_add_gtest(test_matches_onnx_policy_controller
    controller/test/test_matches_onnx_policy_controller.cpp)
  target_link_libraries(test_matches_onnx_policy_controller
    ${PROJECT_NAME}
    ros2_control_demo_example_18::ros2_control_demo_example_18_lib
  )
  target_compile_definitions(test_matches_onnx_policy_controller PRIVATE
    PTE_MODEL_PATH="${PROJECT_SOURCE_DIR}/executorch_model/open_duck_mini_policy.pte"
    ONNX_MODEL_PATH="${ros2_control_demo_example_18_DIR}/../onnx_model/open_duck_mini_policy.onnx")
endif()

ament_package()
