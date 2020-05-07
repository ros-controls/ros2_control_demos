# ROS2 Control Demos

[![Build Status](https://github.com/ros-controls/ros2_control_demos/workflows/CI/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions)
[![Linters Status](https://github.com/ros-controls/ros2_control_demos/workflows/Linters/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions)
[![Coverage Status](https://github.com/ros-controls/ros2_control_demos/workflows/Coverage/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulation of a robot to demonstrate and prove `ros2_control` concepts.

## Goals

The repository has two goals:

1. It provides templates for faster start of implementing own hardware and controllers;
2. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager,  communication between robot hardware and controllers).


## Description

The repository is inspired by [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman.
The simulation has three parts/packages:
1. The first package, `ros2_control_demo_communication_headless`, uses scripts to simulate communication to and movements of virtual hardware.
This package does not have any dependencies except on the `ros2` core packages and can, therefore, be used on SoC-hardware of headless systems.
2. The second package, `ros2_control_demo_communication_gazebo`, uses a gazebo simulator to simulate the *RRBot* and its physics.
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

# Getting Started with ROS2 Control

 To test `demo_robot` start launch file with:
```
ros2 launch ros2_control_demo_robot demo_robot_launch.py
```
for headless version of the `demo_robot` or
```
ros2 launch ros2_control_demo_robot demo_robot_modular_launch.py
```
for headless modular version of the `demo_robot` which uses separate interfaces for its sensors and actuators.

## Class Diagram of `demo_robot`

The figure hereunder depicts the class diagram of `ros2_control_demos` metapackage, i.e., `demo_robot`, together with used interfaces and classes from `ros2_control` package.
Diagram gives an overview of the components needed for a robot that is ROS2-Control compatible.
In the middle of the diagram (no color), entities are core components of `ros2_control`.
Green colored entities are "ROS2-Control"-standardized data structures, hardware representations, and communication interfaces.
Blue colored is the implementation of `demo_robot`.
A large amount of classes is for the exemplary purpose, therefore __don't worry__ you probably don't need so many different interfaces and classes.

![ROS2 Control Demo Robot - Class Diagram][ros2_control_demo_robot_class_diagram]


# Making a Robot for ROS2-Control Compatible

Before starting the implementation of your robot, please check the following diagram:

![ROS2 Control - Enabling a new Robot][ROS2 Control - Enabling a new Robot]



<!-- References -->
[ros2_control_demo_robot_class_diagram]: docs/ros2_control_demo_robot_class_diagram.svg "ROS2 Control Demo Robot - Class Diagram"
[ros2_control_new_robot]: docs/ros2_control_new_robot.svg "ROS2 Control - Enabling a new Robot"
