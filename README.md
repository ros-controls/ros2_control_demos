The repository has two goals:

1. It provides templates for faster start of implementing own hardware and controllers;
2. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager,  communication between robot hardware and controllers). 

# ROS2 Control Demos

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulation of a robot to demonstrate and prove `ros2_control` concepts.
The repository is inspired by [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman.
The simulation has three parts/packages:
1. The first package, `ros2_control_demo_robot_headless`, uses scripts to simulate communication to and movements of virtual hardware.
This package does not have any dependencies except on the `ros2` core packages and can, therefore, be used on SoC-hardware of headless systems.
2. The second package, `ros2_control_demo_robot_gazebo`, uses a gazebo simulator to simulate the *RRBot* and its physics.
This package is useful to test the connection of `ros2_control` to the gazebo simulator and to detect any missing plugins.
3. The third package holds template/example files for `ros2_control` enabled robots and controllers.
The intention of those files is to simplify start with `ros2_control` and to enable faster integration of new robots and controllers.

This repository demonstrates the following `ros2_control` concepts:

* Creating of `hardware_interface` for a Robot, Sensor, and Actor
* Creating a robot description in the form of YAML files
* Loading configuration and starting robot using launch files 
* Control of two joints of *RRBot*
* Using simulated robots and starting `ros_control` with gazebo simulator
* Implementing of controller switching strategy for a robot
* Using joint limits and transmission concepts in `ros2_control`
* TBD
