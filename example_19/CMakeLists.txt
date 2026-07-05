cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_19 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY bringup description doc
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/mock_vla_policy.py
    scripts/verify_smoothness.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_bridge_demo_launch.py)
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
