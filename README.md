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
The examples have three parts/packages according to usual structure of ROS packages for robots:
1. The bringup package `ros2_control_demo_bringup`, holds launch files and runtime configurations for demo robots.
2. Description packages `rrbot_description` and `diffbot_description` (inside `ros2_control_demo_description`), store URDF-description files, rviz configurations and meshes for the demo robots.
3. Hardware interface package `ros2_control_demo_hardware`, implements the hardware interfaces described in the roadmap.

The examples of *RRBot* and *DiffBot* are trivial simulations to demonstrate and test `ros2_control` concepts.
This package does not have any dependencies except `ros2` core packages and can, therefore, be used on SoC-hardware or headless systems.

This repository demonstrates the following `ros2_control` concepts:

* Creating a `*HardwareInterface` for a System, Sensor, and Actuator.
* Creating a robot description in the form of URDF files.
* Loading the configuration and starting a robot using launch files.
* Control of two joints of *RRBot*.
* Using simulated robots and starting `ros2_control` with Gazebo simulator.
* Implementing a controller switching strategy for a robot.
* Using joint limits and transmission concepts in `ros2_control`.

## Quick Hints

These are some quick hints, especially for those coming from a ROS1 control background:

* There are now three categories of hardware components: *Sensor*, *Actuator*, and *System*.
  *Sensor* is for individual sensors; *Actuator* is for individual actuators; *System* is for any combination of multiple sensors/actuators.
  You could think of a Sensor as read-only.
  All components are used as plugins and therefore exported using `PLUGINLIB_EXPORT_CLASS` macro.
* *ros(1)_control* only allowed three hardware interface types: position, velocity, and effort.
  *ros2_control* allows you to create any interface type by defining a custom string. For example, you might define a `position_in_degrees` or a `temperature` interface.
  The most common (position, velocity, acceleration, effort) are already defined as constants in hardware_interface/types/hardware_interface_type_values.hpp.
* Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.
* In ros2_control, all parameters for the driver are specified in the URDF.
  The ros2_control framework uses the **<ros2_control>** tag in the URDF.
* Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.

# Build from source
```
git clone https://github.com/ros-controls/ros2_control
git clone https://github.com/ros-controls/ros2_controllers
git clone https://github.com/ros-controls/ros2_control_demos
```

**NOTE**: `ros2_control` and `ros2_controllers` packages are released for foxy and can be installed using a package manager.
We provide officially released and maintained debian packages, which can easily be installed via aptitude.
However, there might be cases in which not-yet released demos or features are only available through a source build in your own workspace.

* Install dependencies (maybe you need `sudo`):
  ```
  apt install ros-foxy-realtime-tools ros-foxy-xacro ros-foxy-angles
  ```

* Build everything, e.g. with:
  ```
  colcon build --symlink-install
  ```

* Do not forget to source `setup.bash` from the `install` folder!


# Getting Started with demos

This repository provides two simple example robots: a 2 degrees of freedom manipulator - *RRBot* - and a mobile differential drive base - *DiffBot*.

## *RRBot*

*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will use to demonstrate various features.
It is essentially a double inverted pendulum and demonstrates some fun control concepts within a simulator and was originally introduced for Gazebo tutorials.
The *RRBot* URDF files can be found in the `urdf` folder of `rrbot_description` package.



## *DiffBot*

*DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
The robot is basically a box moving according to differential drive kinematics.
The *DiffBot* URDF files can be found in `urdf` folder of `diffbot_description` package.

1. To start *DiffBot* example open open a terminal, source your ROS2-workspace and execute its launch file with:
   ```
   ros2 launch ros2_control_demo_bringup diffbot.launch.py
   ```
   The launch file loads and starts the robot hardware, controllers and opens `RViz`.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purpuses and should be avoided as much as possible in a hardware interface implementation.

1. If you can see an orange box in `RViz` everything has started properly. Still, to be sure, let's introspect the control system:
   Check if the hardware interface loaded properly, by opening another terminal and executing:
   ```
   ros2 control list_hardware_interfaces
   ```
   You should get:
   ```
   command interfaces
        left_wheel_joint/velocity [claimed]
        right_wheel_joint/velocity [claimed]
   state interfaces
         left_wheel_joint/position
         left_wheel_joint/velocity
         right_wheel_joint/position
         right_wheel_joint/velocity
   ```

1. Check is controllers are running:
   ```
   ros2 control list_controllers
   ```
   You should get:
   ```
   diffbot_base_controller[diff_drive_controller/DiffDriveController] active
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   ```

1. If you get output from about you can send some command to *Diff Drive Controller*:
   ```
   ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
    x: 0.7
    y: 0.0
    z: 0.0
   angular:
    x: 0.0
    y: 0.0
    z: 1.0"
    ```
   You should now see an orange box circling in `rviz`. Also, you should see


Files used for this demos:
  - Launch file: [diffbot.launch.py](ros2_control_demo_bringup/launch/diffbot.launch.py)
  - Controllers yaml: [diffbot_diff_drive_controller.yaml](ros2_control_demo_bringup/config/diffbot_diff_drive_controller.yaml)
  - URDF file: [diffbot.urdf.xacro](ros2_control_demo_description/diffbot_description/urdf/diffbot.urdf.xacro)
    - Description: [diffbot_description.urdf.xacro](ros2_control_demo_description/diffbot_description/urdf/diffbot_description.urdf.xacro)
    - `ros2_control` tag: [diffbot.ros2_control.xacro](ros2_control_demo_description/diffbot_description/ros2_control/diffbot.ros2_control.xacro)
  - RViz configuration: [diffbot.rviz](ros2_control_demo_description/diffbot_description/config/diffbot.rviz)

  - Hardware interface plugin: [diffbot_system.cpp](ros2_control_demo_hardware/src/diffbot_system.cpp)


Controllers from this demo:
  - `Joint State Broadcaster` ([`ros2_controllers` repository](https://github.com/ros-controls/ros2_controllers)): [doc](https://ros-controls.github.io/control.ros.org/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)
  - `Diff Drive Controller` ([`ros2_controllers` repository](https://github.com/ros-controls/ros2_controllers)): [doc](https://ros-controls.github.io/control.ros.org/ros2_controllers/diff_drive_controller/doc/userdoc.html)


# Examples of ros2_control concepts

Each of the described example cases from the [roadmap](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md) has its own launch and URDF file.

## Starting example robots

Each example is started with a single launch file which starts up the robot hardware, loads controller configurations and it also opens `rviz2`.

The `rviz2` setup can be recreated following these steps:

- The robot models can be visualized using `RobotModel` display using `/robot_description` topic.
- Or you can simply open the configuration from `rviz` folder in `rrbot_description` or `diffbot_description` package manually or directly by executing:
  ```
  rviz2 --display-config `ros2 pkg prefix rrbot_description`/share/rrbot_description/config/rrbot.rviz
  ```
  of
  ```
  rviz2 --display-config `ros2 pkg prefix diffbot_description`/share/rrbot_description/config/diffbot.rviz
  ```


### General notes about examples

1. To check that robot descriptions are working properly use following launch commands:

   *RRBot*
   ```
   ros2 launch rrbot_description view_robot.launch.py
   ```
   Optional arguments for specific example (the robot visualization will be the same for all examples):
   ```
   description_file:=rrbot_system_multi_interface.urdf.xacro
   ```

   *DiffBot*
   ```
   ros2 launch diffbot_description view_robot.launch.py
   ```

**NOTE**: Getting the following output in terminal is OK: `Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist`.

1. To start an example open a terminal, source your ROS2-workspace and execute a launch file with:
   ```
   ros2 launch ros2_control_demo_bringup <example_launch_file>
   ```

1. To stop RViz2 from auto-start use `start_rviz:=false` launch file argument.

1. To check if the hardware interface loaded properly, open another terminal and execute:
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

1. Check which controllers are running using:
   ```
   ros2 control list_controllers
   ```
   You should get something like:
   ```
   forward_position_controller[forward_command_controller/ForwardCommandController] unconfigured
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   ```


1. Check [Controllers and moving hardware](#Controlles-and-moving-hardware) section to move *RRBot*.


### Example 1: "Industrial Robots with only one interface"

Files:
  - Launch file: [rrbot_minimal.launch.py](ros2_control_demo_bringup/launch/rrbot_minimal.launch.py)
  - Controllers yaml: [rrbot_controllers.yaml](ros2_control_demo_bringup/config/rrbot_controllers.yaml)
  - `ros2_control` URDF tag: [rrbot_system_position_only.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot_system_position_only.ros2_control.xacro)

Interfaces:
  - Command interfaces:
    - joint1/position
    - joint2/position
  - State interfaces:
    - joint1/position
    - joint2/position

Available controllers:
  - `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`
  - `forward_position_controller[forward_command_controller/ForwardCommandController]` (position)

Available launch-file options:
  - `use_fake_hardware:=true` - start `FakeSystem` instead of hardware.
    This is a simple simulation that mimics joint command to their states.
    This is useful to test *ros2_control* integration and controllers without physical hardware.

**NOTE:** This example also shows how launch files are usually reused when working with `ros2_control`. The [`rrbot.launch.py`](ros2_control_demo_bringup/launch/rrbot.launch.py) is used as a configurable base-file of a robot and [`rrbot_system_position_only.launch.py`](ros2_control_demo_bringup/launch/rrbot_system_position_only.launch.py) shows how this file can be used for a specific scenario.

### Example 2: "Industrial Robots with only one interface" (Gazebo simulation)

- **TBA**


### Example 3: "Robots with multiple interfaces"

Files:
  - Launch file: [rrbot_system_multi_interface.launch.py](ros2_control_demo_bringup/launch/rrbot_system_multi_interface.launch.py)
  - Controllers yaml: [rrbot_multi_interface_forward_controllers.yaml](ros2_control_demo_bringup/config/rrbot_multi_interface_forward_controllers.yaml)
  - `ros2_control` URDF tag: [rrbot_system_multi_interface.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot_system_multi_interface.ros2_control.xacro)

Interfaces:
  - Command interfaces:
    - joint1/position
    - joint2/position
    - joint1/velocity
    - joint2/velocity
    - joint1/acceleration
    - joint2/acceleration
  - State interfaces:
    - joint1/position
    - joint2/position
    - joint1/velocity
    - joint2/velocity
    - joint1/acceleration
    - joint2/acceleration

Available controllers:
  - `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`
  - `forward_position_controller[position_controllers/JointGroupPositionController]`
  - `forward_velocity_controller[velocity_controllers/JointGroupVelocityController]`
  - `forward_acceleration_controller[forward_command_controller/ForwardCommandController]`
  - `forward_illegal1_controller[forward_command_controller/ForwardCommandController]`
  - `forward_illegal2_controller[forward_command_controller/ForwardCommandController]`

Notes:
  - The example shows how to implement multi-interface robot hardware taking care about interfaces used.
    The two illegal controllers demonstrate how hardware interface declines faulty claims to access joint command interfaces.


### Example 4: "Differential drive mobile robot"

Files:
  - Launch file: [diffbot_system.launch.py](ros2_control_demo_bringup/launch/diffbot_system.launch.py)
  - Controllers yaml: [diffbot_diff_drive_controller.yaml](ros2_control_demo_bringup/config/diffbot_diff_drive_controller.yaml)
  - `ros2_control` URDF tag: [diffbot_system.ros2_control.xacro](ros2_control_demo_description/diffbot_description/ros2_control/diffbot_system.ros2_control.xacro)

Interfaces:
  - Command interfaces:
    - left_wheel_joint/velocity
    - right_wheel_joint/velocity
  - State interfaces:
    - left_wheel_joint/position
    - left_wheel_joint/velocity
    - right_wheel_joint/position
    - right_wheel_joint/velocity

Available controllers:
  - `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`
  - `diffbot_base_controller[diff_drive_controller/DiffDriveController]`

Sending commands to diff drive controller:

```
ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.7
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"
```

You should now see an orange box circling in `rviz2`.


## Controllers and moving hardware
To move the robot you should load and start controllers.
The `JointStateController` is used to publish the joint states to ROS topics.
Direct joint commands are sent to this robot via the `ForwardCommandController`.
The sections below describe their usage.
Check the [Results](##result) section on how to ensure that things went well.


### JointStateController

Open another terminal and load, configure and start `joint_state_controller`:
```
ros2 control set_controller_state joint_state_controller start
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

1. If you want to test hardware with `ForwardCommandController` first load a controller (not always needed):
   ```
   ros2 control load_controller forward_position_controller
   ```
   Check if the controller is loaded properly:
   ```
   ros2 control list_controllers
   ```

2. Then configure it:
   ```
   ros2 control set_controller_state forward_position_controller configure
   ```
   Check if the controller is loaded properly:
   ```
   ros2 control list_controllers
   ```
   You should get the response:
   ```
   forward_position_controller[forward_command_controller/ForwardCommandController] inactive
   ```

2. Now start the controller:
   ```
   ros2 control switch_controllers --start forward_position_controller
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

3. Send a command to the controller, either:

   a. Manually using ros2 cli interface:
   ```
   ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
   - 0.5
   - 0.5"
   ```
   B. Or you can start a demo node which sends two goals every 5 seconds in a loop:
   ```
   ros2 launch ros2_control_demo_bringup test_forward_position_controller.launch.py
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
