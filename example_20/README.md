# ros2_control_demo_example_20

This example demonstrates the ``cartesian_trajectory_controller``: a mock 6-DOF r6bot is driven by end-effector pose chunks that the controller interpolates (cubic-spline translation + SLERP orientation) and runs through differential inverse kinematics, so the tool center point traces a commanded Cartesian path (circle / square / line) on mock hardware.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_20/doc/userdoc.html).
