cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_19)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY bringup description
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
  # run the launch test in an isolated ROS domain (matches the other examples)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_bridge_demo_launch.py)
endif()

ament_package()
