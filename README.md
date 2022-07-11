# ros2_control Demos

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulations to demonstrate and prove `ros2_control` concepts.

### Goals

The repository has three goals:
1. Implements the example configuration described in the `ros-controls/roadmap` repository file [components_architecture_and_urdf_examples](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).
2. It provides templates for faster implementation of custom hardware and controllers;
3. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).


## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`rolling`](https://github.com/ros-controls/ros2_control_demos/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br /> [![Rolling Source Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-source-build.yml?branch=master) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/) | [ros2_control_demos](https://index.ros.org/p/ros2_control_demos/#rolling)
**Rolling - last Focal** | [`rolling`](https://github.com/ros-controls/ros2_control_demos/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build-last-focal.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build-last-focal.yml?branch=master) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/) | [ros2_control_demos](https://index.ros.org/p/ros2_control_demos/#rolling)
**Galactic** | [`galactic`](https://github.com/ros-controls/ros2_control_demos/tree/galactic) | [![Galactic Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-binary-build.yml?branch=galactic) <br /> [![Galactic Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-semi-binary-build.yml?branch=galactic) <br /> [![Galactic Source Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-source-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-source-build.yml?branch=galactic) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/) | [ros2_control_demos](https://index.ros.org/p/ros2_control_demos/#galactic)
**Foxy** | [`foxy`](https://github.com/ros-controls/ros2_control_demos/tree/foxy) | [![Foxy Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-binary-build.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-semi-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-semi-binary-build.yml?branch=foxy) <br /> [![Foxy Source Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-source-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-source-build.yml?branch=foxy) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/) | [ros2_control_demos](https://index.ros.org/p/ros2_control_demos/#foxy)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


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
* Control of a differential mobile base *DiffBot*.
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

* Install dependencies:
  ```
  rosdep install --from-paths src --ignore-src -r -y
  ```

* Build everything, e.g. with:
  ```
  colcon build --symlink-install
  ```

* Do not forget to source `setup.bash` from the `install` folder!


# Getting Started with demos

This repository provides the following simple example robots: a 2 degrees of freedom manipulator - *RRBot* - and a mobile differential drive base - *DiffBot*.

The first two examples demonstrate the minimal setup for those two robots to run.
Later examples show more details about `ros2_control`-concepts and some more advanced use-cases.

## *RRBot*

*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will use to demonstrate various features.
It is essentially a double inverted pendulum and demonstrates some fun control concepts within a simulator and was originally introduced for Gazebo tutorials.
The *RRBot* URDF files can be found in the `urdf` folder of `rrbot_description` package.

1. To check that *RRBot* descriptions are working properly use following launch commands:

   *RRBot*
   ```
   ros2 launch rrbot_description view_robot.launch.py
   ```
   **NOTE**: Getting the following output in terminal is OK: `Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist`.
             This happens because `joint_state_publisher_gui` node need some time to start.

The `joint_state_publisher_gui` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in `Rviz`.

1. To start *RRBot* example open open a terminal, source your ROS2-workspace and execute its launch file with:
   ```
   ros2 launch ros2_control_demo_bringup rrbot.launch.py
   ```
   The launch file loads and starts the robot hardware, controllers and opens `RViz`.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purpuses and should be avoided as much as possible in a hardware interface implementation.

   If you can see two orange and one yellow rectangle in in `RViz` everything has started properly.
   Still, to be sure, let's introspect the control system before moving *RRBot*.

1. Check if the hardware interface loaded properly, by opening another terminal and executing:
   ```
   ros2 control list_hardware_interfaces
   ```
   You should get:
   ```
   command interfaces
        joint1/position [claimed]
        joint2/position [claimed]
   state interfaces
         joint1/position
         joint2/position

   ```
   Marker `[claimed]` by command interfaces means that a controller has access to command *RRBot*.

1. Check is controllers are running:
   ```
   ros2 control list_controllers
   ```
   You should get:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   forward_position_controller[forward_command_controller/ForwardCommandController] active
   ```

1. If you get output from above you can send commands to *Forward Command Controller*, either:

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
   You should now see orange and yellow blocks moving in `RViz`.
   Also, you should see changing states in the termnal where launch file is started.


Files used for this demos:
  - Launch file: [rrbot.launch.py](ros2_control_demo_bringup/launch/rrbot.launch.py)
  - Controllers yaml: [rrbot_controllers.yaml](ros2_control_demo_bringup/config/rrbot_controllers.yaml)
  - URDF file: [rrbot.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot.urdf.xacro)
    - Description: [rrbot_description.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot_description.urdf.xacro)
    - `ros2_control` tag: [rrbot.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot.ros2_control.xacro)
  - RViz configuration: [rrbot.rviz](ros2_control_demo_description/rrbot_description/config/rrbot.rviz)

  - Hardware interface plugin: [rrbot_system_position_only.cpp](ros2_control_demo_hardware/src/rrbot_system_position_only.cpp)


Controllers from this demo:
  - `Joint State Broadcaster` ([`ros2_controllers` repository](https://github.com/ros-controls/ros2_controllers)): [doc](https://ros-controls.github.io/control.ros.org/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)
  - `Forward Command Controller` ([`ros2_controllers` repository](https://github.com/ros-controls/ros2_controllers)): [doc](https://ros-controls.github.io/control.ros.org/ros2_controllers/forward_command_controller/doc/userdoc.html)


## *DiffBot*

*DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
The robot is basically a box moving according to differential drive kinematics.
The *DiffBot* URDF files can be found in `urdf` folder of `diffbot_description` package.

..TBD... (in the next PR!)


# Examples of ros2_control concepts

Each of the described example cases from the [roadmap](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md) has its own launch and URDF file.


### General notes about examples

1. Each example is started with a single launch file which starts up the robot hardware, loads controller configurations and it also opens `RViz`.

   The `RViz` setup can be recreated following these steps:

   - The robot models can be visualized using `RobotModel` display using `/robot_description` topic.
   - Or you can simply open the configuration from `rviz` folder in `rrbot_description` or `diffbot_description` package manually or directly by executing:
   ```
   rviz2 --display-config `ros2 pkg prefix rrbot_description`/share/rrbot_description/config/rrbot.rviz
   ```

1. To check that robot descriptions are working properly use following launch commands:
   ```
   ros2 launch rrbot_description view_robot.launch.py
   ```
   Optional arguments for specific example (the robot visualization will be the same for all examples):
   ```
   description_file:=rrbot_system_multi_interface.urdf.xacro
   ```

**NOTE**: Getting the following output in terminal is OK: `Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist`.
          This happens because `joint_state_publisher_gui` node need some time to start.

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

1. Check [Controllers and moving hardware](#controllers-and-moving-hardware) section to move *RRBot*.


### Example 1: "Industrial Robots with only one interface"

- Launch file: rrbot_system_position_only.launch.py
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


### Example 2: "Industrial Robots with only one interface" (Gazebo simulation)

- **TBA**


### Example 3: "Robots with multiple interfaces"

- Launch file: rrbot_system_multi_interface.launch.py
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

- Launch file: diffbot_system.launch.py
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
  - `diffbot_base_controller[diff_drive_controller/DiffDriveController] active`

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

You should now see an orange box circling in `RViz`.


### Example 5: "Industrial robot with integrated sensor"

- Launch file: [rrbot_system_with_sensor.launch.py](ros2_control_demo_bringup/launch/rrbot_system_with_sensor.launch.py)
- URDF: [rrbot_system_with_sensor.urdf.xacro](ros2_control_demo_bringup/config/rrbot_with_sensor_controllers.yaml)
- ros2_control URDF: [rrbot_system_with_sensor.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot_system_with_sensor.ros2_control.xacro)

- Command interfaces:
  - joint1/position
  - joint2/position
- State interfaces:
  - joint1/position
  - joint2/position
  - tcp_fts_sensor/force.x
  - tcp_fts_sensor/torque.z

Available controllers:
- `forward_position_controller[forward_command_controller/ForwardCommandController]`
- `fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster]`
- `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`

Notes:
  - Wrench messages are not displayed properly in Rviz as NaN values are not handled in Rviz and FTS Broadcaster may send NaN values.

Commanding the robot: see the commands below.

Accessing Wrench data from 2D FTS:
```
ros2 topic echo /fts_broadcaster/wrench
```


## Controllers and moving hardware

To move the robot you should load and start controllers.
The `JointStateController` is used to publish the joint states to ROS topics.
Direct joint commands are sent to this robot via the `ForwardCommandController` and `JointTrajectoryController`.
The sections below describe their usage.
Check the [Results](##result) section on how to ensure that things went well.

**NOTE**: Before doing any action with controllers check their state using command:
```
ros2 control list_controllers
```


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

Now you should also see the *RRbot* represented correctly in `RViz`.


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

3. Now start the controller:
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

4. Send a command to the controller, either:

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
   You can adjust the goals in [rrbot_forward_position_publisher.yaml](ros2_control_demo_bringup/config/rrbot_forward_position_publisher.yaml).

### Using JointTrajectoryController

1. If you want to test hardware with `JointTrajectoryController` first load and configure a controller (not always needed):
   ```
   ros2 control load_controller joint_trajectory_position_controller --set-state configure
   ```
   Check if the controller is loaded and configured properly:
   ```
   ros2 control list_controllers
   ```
   You should get the response:
   ```
   joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive
   ```

2. Now start the controller (and stop other running contorller):
   ```
   ros2 control switch_controllers --stop forward_position_controller --start joint_trajectory_position_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_controller[joint_state_controller/JointStateController] active
   joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] active
   ```

3. Send a command to the controller using demo node which sends two goals every 6 seconds in a loop:
   ```
   ros2 launch ros2_control_demo_bringup test_joint_trajectory_position_controller.launch.py
   ```
   You can adjust the goals in [rrbot_joint_trajectory_publisher.yaml](ros2_control_demo_bringup/config/rrbot_joint_trajectory_publisher.yaml).

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

3. You should also see the *RRbot* moving in `RViz`.
