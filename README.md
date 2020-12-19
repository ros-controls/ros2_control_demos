# ROS2 Control Demos

[![Build Status](https://github.com/ros-controls/ros2_control_demos/workflows/CI/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACI)
[![Linters Status](https://github.com/ros-controls/ros2_control_demos/workflows/Linters/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ALinters)
[![Coverage Status](https://github.com/ros-controls/ros2_control_demos/workflows/Coverage/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACoverage)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulation of a robot to demonstrate and prove `ros2_control` concepts.

**ATTENTION: `ros2_control` is currently under heavy development and the any APIs implementation in this repositry, can be broken without any announcement!**

## Goals

The repository has three goals:
1. Implements the example configuration described in the `ros-controls/roadmap` repository file [components_architecture_and_urdf_examples](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).
2. It provides templates for faster start of implementing own hardware and controllers;
3. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).


## Description

The repository is inspired by [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman.
The simulation has three parts/packages:
1. The first package, `ros2_control_demo_hardware`, implements the hardware interfaces described in the roadmap.
This implemented examples simulate *RRbot* internally to provide sufficient test and demonstration data, but to reduce amount of package dependencies.
This package does not have any dependencies except on the `ros2` core packages and can, therefore, be used on SoC-hardware of headless systems.
2. The second package, `ros2_control_demo_hardware_gazebo`, uses a gazebo simulator to simulate the *RRBot* and its physics.
This package is useful to test the connection of `ros2_control` to the gazebo simulator and to detect any missing plugins.
3. The third package `ros2_control_demo_robot` holds examples for *RRbot* URDF-description, launch files and controllers.
The intention of those files is to simplify start with `ros2_control` and to enable faster integration of new robots and controllers.

This repository demonstrates the following `ros2_control` concepts:

* Creating of `*HardwareInterface` for a System, Sensor, and Actuator.
* Creating a robot description in the form of URDF files
* Loading configuration and starting robot using launch files 
* Control of two joints of *RRBot*
* Using simulated robots and starting `ros_control` with gazebo simulator
* Implementing of controller switching strategy for a robot
* Using joint limits and transmission concepts in `ros2_control`
* TBD...

# Test of the Scenario Before the First Release
* Checkout [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) to get the core.
* Checkout [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers) to get all the controllers.
* Checkout [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) to get example hardware and robot launch files.

* Install dependencies (maybe you need `sudo`):
  ```
  apt install ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-angles
  ```

* Build everything, e.g. with:
  ``` 
  colcon build --symlink-install
  ```
  
* Do not forget to source `setup.bash` from the `install` folder!
  
  
# Getting Started with ROS2 Control

Each of the described example cases from the roadmap has its own launch and URDF file.

## Example 1: "Industrial Robots with only one interface"

1. Start the roslaunch file:
  ```
  ros2 launch ros2_control_demo_robot rrbot_system_position_only.launch.py
  ```

2. Open another terminal and check if `RRBotSystemPositionOnlyHardware` is loaded properly:
  ```
  ros2 control list_hardware_interfaces
  ```
  You should get something like:
  ```
  command interfaces
        joint1/position [unclaimed]
        joint2/position [unclaimed]
  state interfaces
         joint1/position
         joint2/position
  ```

3. Open another terminal and load, configure and start controllers:
  ```
  ros2 control load_start_controller joint_state_controller
  ros2 control load_configure_controller forward_command_controller_position
  ```
  
  Check if controller is loaded properly:
  ```
  ros2 control list_controller
  ```
  You should get the response:
  ```
  joint_state_controller[joint_state_controller/JointStateController] active  
  forward_command_controller_position[forward_command_controller/ForwardCommandController] inactive
  ```

4. Starting controller:
  ```
  ros2 control switch_controllers --start-controllers forward_command_controller_position 
  ```
  
  Check if controllers are activated:
  ```
  ros2 control list_controller
  ```
  You should get `active` in the response:
  ```
  joint_state_controller[joint_state_controller/JointStateController] active    
  forward_command_controller_position[forward_command_controller/ForwardCommandController] active
  ```
  
5. Open another terminal and send a message to the controller:
  ```
  ros2 topic pub /forward_command_controller_position/commands std_msgs/msg/Float64MultiArray "data: 
  - 0.5                                                               
  - 0.5"
  ```
  
  You should see how the example output changes. Look for the following lines
  ```
  [RRBotSystemPositionOnlyHardware]: Got state 0.0 for joint 0!
  [RRBotSystemPositionOnlyHardware]: Got state 0.0 for joint 1!
  ```
  
  If you echo the `/joint_states` or `/dynamic_joint_states` topics you should also get similar values.
  ```
  ros2 topic echo /joint_states
  ros2 topic echo /dynamic_joint_states
  ```

The other launch-files have corresponding names to their coresponding example.
The URDF files can be found in the `description` folder.
