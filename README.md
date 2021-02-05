# ROS2 Control Demos

[![Build Status](https://github.com/ros-controls/ros2_control_demos/workflows/CI/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACI)
[![Linters Status](https://github.com/ros-controls/ros2_control_demos/workflows/Linters/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ALinters)
[![Coverage Status](https://github.com/ros-controls/ros2_control_demos/workflows/Coverage/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACoverage)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulation of a robot to demonstrate and prove `ros2_control` concepts.

[`ros2_control_demo_robot/description/rrbot_system_position_only.urdf.xacro`](ros2_control_demo_robot/description/rrbot_system_position_only.urdf.xacro)

**ATTENTION**: `ros2_control` is currently under heavy development and the any APIs implementation in this repositry, can be broken without any announcement!**

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

# Testing from the sources
* Checkout [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) to get the core.
* Checkout [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers) to get all the controllers.
* Checkout [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) to get example hardware and robot launch files.

**NOTE**: `ros2_control` and `ros2_controllers` packages are released for foxy and can be installed using package manager.
  Still, it is recommended to use source version since there might be some not-yet-released changes.

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

## Starting example robots

First you need to start robot's hardware and load controllers configuration.
This is done using one launch file.

To visualize the robot's state use `rviz2`.
The robot models can be visualized using `RobotModel` display using `/robot_description` topic.
Or you can simply open the configuration from `rviz` folder in `ros2_control_demo_robot` package manually or directly by executing:
```
rviz2 --display-config `ros2 pkg prefix ros2_control_demo_robot`/share/ros2_control_demo_robot/rviz/test_controllers.rviz
```

The *RRbbot*'s URDF files can be found in the `description` folder of `ros2_control_demo_robot` package.

### Example 1: "Industrial Robots with only one interface"

1. Open another terminal and start the roslaunch file:
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

## Controlles and moving hardware
To move the robot you should load and start contorllers.
To get feedback about robot's state `JointStateController` is used.
to send command to the robot `ForwardcommandController` (direct goals) or `JointTrajectoryController` (interpolates trajectory).
The sections below describe their usage.
Than jump to the Results section to read how to check if everything is fine.

### JointStateController

Open another terminal and load, configure and start `joint_state_controller`:
```
ros2 control load_start_controller joint_state_controller
```
Check if controller is loaded properly:
```
ros2 control list_controllers
```
You should get the response:
```
joint_state_controller[joint_state_controller/JointStateController] active
```

Now you should also see the *RRbot* represented correctly in the `rviz2`.


### Using ForwardCommandController

1. If you want to test hardware with `ForwardCommandController` first load and configure it:
   ```
   ros2 control load_configure_controller forward_position_controller
   ```
   Check if controller is loaded properly:
   ```
   ros2 control list_controllers
   ```
   You should get the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   forward_position_controller[forward_command_controller/ForwardCommandController] inactive
   ```

2. Now start the controller:
   ```
   ros2 control switch_controllers --start-controllers forward_position_controller
   ```
  
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   forward_position_controller[forward_command_controller/ForwardCommandController] active
   ```

**NOTE**: You can do this in only one step by using `load_start_controller` verb instead of `load_configure_controller`.

3. Send command to the controller, either:

   a. Manually using ros2 cli interface:
   ```
   ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
   - 0.5                                                               
   - 0.5"
   ```
   b. Or you can start demo node which sends two goals every 5 seconds in a loop:
   ```
   ros2 launch ros2_control_test_nodes rrbot_test_forward_position_controller.launch.py
   ```

### Using JointTrajectoryController

1. If a `ForwardCommandController` is started you should stop it first by using:
   ```
   ros2 control switch_controllers --stop-controllers forward_position_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   forward_position_controller[forward_command_controller/ForwardCommandController] inactive
   ```

2. If you want to test hardware with `JointTrajectoryController` first load, configure and start it:
   ```
   ros2 control load_start_controller position_trajectory_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
   ```

3. Send command to the controller using test node:
   ```
   ros2 launch ros2_control_demo_robot test_joint_trajectory_controller.launch.py
   ```
   
**NOTE**: You can swith contorllers (step 1 and 2) also with one command:
```
ros2 control switch_controllers --stop-controllers forward_<controller_type>_controller --start-controllers joint_trajectory_controller
```

## Result

1. Independently from the controller you should see how the example's output changes.
  Look for the following lines
   ```
   [RRBotSystemPositionOnlyHardware]: Got state 0.0 for joint 0!
   [RRBotSystemPositionOnlyHardware]: Got state 0.0 for joint 1!
   ```

2. If you echo the `/joint_states` or `/dynamic_joint_states` topics you should also get similar values.
   ```
   ros2 topic echo /joint_states
   ros2 topic echo /dynamic_joint_states
   ```

3. You should also see the *RRbot* moving in the `rviz2`.
