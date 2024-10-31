cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_13 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# INSTALL
install(
  DIRECTORY description/ros2_control description/urdf description/rviz
  DESTINATION share/ros2_control_demo_example_13
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_13
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_13_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_13_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_13_launch test/test_three_robots_launch.py)
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
