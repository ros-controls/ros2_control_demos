cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_17)

# INSTALL
install(
  DIRECTORY description/gazebo description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_17
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_17
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_17_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_17_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_17_launch test/test_diffbot_launch.py)
endif()

ament_package()
