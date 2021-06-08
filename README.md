# ros2_control Demos

[![Build Status](https://github.com/ros-controls/ros2_control_demos/workflows/CI/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACI)
[![Linters Status](https://github.com/ros-controls/ros2_control_demos/workflows/Linters/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ALinters)
[![Coverage Status](https://github.com/ros-controls/ros2_control_demos/workflows/Coverage/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACoverage)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulations to demonstrate and prove `ros2_control` concepts.

## Goals

The repository has three goals:
1. Implements the example configuration described in the `ros-controls/roadmap` repository file [components_architecture_and_urdf_examples](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).
2. It provides templates for faster implementation of custom hardware and controllers;
3. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).


## Description

The repository is inspired by the [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman.
The simulation has three parts/packages:
1. The first package, `ros2_control_demo_bringup`, holds launch files and runtime configurations for demo robots.
2. The second package, `ros2_control_demo_description`, stored URDF-description files, rviz configurations and meshes for the demo robots.
3. The third package, `ros2_control_demo_hardware`, implements the hardware interfaces described in the roadmap.
The examples simulate a simple *RRbot* internally to provide sufficient test and demonstration data and reduce external dependencies.
This package does not have any dependencies except `ros2` core packages and can, therefore, be used on SoC-hardware of headless systems.

This repository demonstrates the following `ros2_control` concepts:

* Creating of `*HardwareInterface` for a System, Sensor, and Actuator.
* Creating a robot description in the form of URDF files
* Loading the configuration and starting a robot using launch files
* Control of two joints of *RRBot*
* Using simulated robots and starting `ros_control` with Gazebo simulator
* Implementing of controller switching strategy for a robot
* Using joint limits and transmission concepts in `ros2_control`

## Quick Hints

These are some quick hints, especially for those coming from a ROS1 control background:

* There are now three categories of hardware interface: *Sensor*, *Actuator*, and *System*. Sensor is for individual sensors; Actuator is for individual actuators; System is for any combination of multiple sensors/actuators. You could think of a Sensor as read-only.
* ros(1)_control only allowed three hardware interface types: position, velocity, and effort. ros2_control allows you to create any interface type by defining a custom string. For example, you might define a `position_in_degrees` or a `temperature` interface. The most common (position, velocity, acceleration, effort) are already defined as constants in hardware_interface/types/hardware_interface_type_values.hpp.
* In ros2_control, all parameters for the driver are specified in the URDF. The ros2_control framework uses the <ros2_control> tag in the URDF.
* <ros2_control> tags in the URDF must be compatible with the controller's configuration.
* PLUGINLIB_EXPORT_CLASS macro is required when implementing an interface.

# Build from source
```
git clone https://github.com/ros-controls/ros2_control
git clone https://github.com/ros-controls/ros2_controllers
git clone https://github.com/ros-controls/ros2_control_demos
```

**NOTE**: `ros2_control` and `ros2_controllers` packages are released for foxy and can be installed using package manager.
  For daily use it is recommended to use the released version but there may always be some not-yet-released changes that are required to build the demos.

* Install dependencies (maybe you need `sudo`):
  ```
  apt install ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-angles
  ```

* Build everything, e.g. with:
  ```
  colcon build --symlink-install
  ```

* Do not forget to source `setup.bash` from the `install` folder!


# Getting Started with ros2_control

Each of the described example cases from the roadmap has its own launch and URDF file.

## Starting example robots

Each example is started with a single launch file which starts up the robot hardware, loads controller configurations and it also opens `rviz2`.

The `rviz2` setup can be recreated following these steps:

- The robot models can be visualized using `RobotModel` display using `/robot_description` topic.
- Or you can simply open the configuration from `rviz` folder in `ros2_control_demo_description` package manually or directly by executing:
  ```
  rviz2 --display-config `ros2 pkg prefix ros2_control_demo_description`/share/ros2_control_demo_description/rviz/rrbot.rviz
  ```

*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will use to demonstrate various features. It essentially a double inverted pendulum and demonstrates some fun control concepts within a simulator and was originally introduced for Gazebo tutorials.
The *RRbot* URDF files can be found in the `urdf` folder of `ros2_control_demo_description` package.

### Example 1: "Industrial Robots with only one interface"

1. Open another terminal and start the roslaunch file:
   ```
   ros2 launch ros2_control_demo_bringup rrbot_system_position_only.launch.py
   ```

2. Open another terminal and check that `RRBotSystemPositionOnlyHardware` loaded properly:
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

### Example 2: "Robots with multiple interfaces"

1. Open another terminal and start the roslaunch file:
   ```
   ros2 launch ros2_control_demo_bringup rrbot_system_multi_interface.launch.py
   ```

2. Open another terminal and check that `RRBotSystemPositionOnlyHardware` loaded properly:
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
To move the robot you should load and start controllers.
The `JointStateController` is used to publish the joint states to ROS topics.
Direct joint commands are sent to this robot via the `ForwardCommandController`.
The sections below describe their usage.
Check the [Results](##result) section on how to ensure that things went well.


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

Now you should also see the *RRbot* represented correctly in `rviz2`.


### Using ForwardCommandController

1. If you want to test hardware with `ForwardCommandController` first load and configure it:
   ```
   ros2 control load_configure_controller forward_command_controller_position
   ```
   Check if the controller is loaded properly:
   ```
   ros2 control list_controllers
   ```
   You should get the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   forward_command_controller_position[forward_command_controller/ForwardCommandController] inactive
   ```

2. Now start the controller:
   ```
   ros2 control switch_controllers --start-controllers forward_command_controller_position
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   forward_command_controller_position[forward_command_controller/ForwardCommandController] active
   ```

**NOTE**: You can do this in only one step by using `load_start_controller` verb instead of `load_configure_controller`.

3. Send command to the controller, either:

   a. Manually using ros2 cli interface:
   ```
   ros2 topic pub /forward_command_controller_position/commands std_msgs/msg/Float64MultiArray "data:
   - 0.5
   - 0.5"
   ```
   B. Or you can start demo node which sends two goals every 5 seconds in a loop:
   ```
   ros2 launch ros2_control_test_nodes rrbot_test_forward_position_controller.launch.py
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

3. You should also see the *RRbot* moving in `rviz2`.
