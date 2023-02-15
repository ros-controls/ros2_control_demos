# ros2_control Demos

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides examples for functionalities and capabilities of `ros2_control` framework.
It consists of simple implementations that demonstrate different concepts.

If you want to have rather step by step manual how to do things with `ros2_control` checkout [ros-control/roscon2022_workshop](https://github.com/ros-controls/roscon2022_workshop) repository.

### Goals

The repository has two other goals goals:

1. Implements the example configuration described in the `ros-controls/roadmap` repository file [components_architecture_and_urdf_examples](https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md).
2. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).


## Getting started

The repository is structured into `example_XY` folders that fully contained packages with names `ros2_control_demos_example_XY`.

The packages have following structure of subfolders:

- `bringup` - stores launch files and runtime configurations for demo robots.
- `description` - stores URDF (and XACRO) description files, rviz configurations and meshes for the example robots.
- `hardware` - stores implementations of example hardware components (interfaces).
- `controllers` (optional) - stores implementation of example controllers.

The important files to check in each example are:

- `bringup/launch/<example_name>.launch.py` - launch file for the example
- `bringup/config/<example_name>_controllers.yaml` - parameters with controllers' setup for the example.
- `description/<example_name>.ros2_control.xacro` - XACRO file with `ros2_control`-URDF-tag with hardware setup and parameters.
- `description/<example_name>.urdf.xacro` - the main description for for the example used to generate URDF on the fly that is published on the `/robot_description` topic.
- `hardware/<example_name>.hpp` - header file of the example hardware component implementation.
- `hardware/<example_name>.cpp` - source file with the example hardware component implementation.
- `controllers/<example_name>.hpp` - header file of the example controller implementation.
- `controllers/<example_name>.cpp` - source file with the example controller implementation.

**NOTE** - The structure of packages, folders and files given in this repository is not recommended to be used for your robot. Usually you should have all of the above folders defined as separate packages with naming convention `<robot_name_or_type>/[bringup|description|hardware|controllers]`.
  More standard structure can be found in [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman or documentation on [ros_team_workspace](https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html) from Stogl Robotics.

The concepts in this package are demonstrated on the examples of *RRBot* and *DiffBot*.
Those two world-known imaginary robots are trivial simulations to demonstrate and test `ros2_control` concepts.


## What you can Find in This Repository and Example Description

This repository demonstrates the following `ros2_control` concepts:

* Creating a `*HardwareInterface` for a System, Sensor, and Actuator.
* Creating a robot description in the form of URDF files.
* Loading the configuration and starting a robot using launch files.
* Control of a differential mobile base *DiffBot*.
* Control of two joints of *RRBot*.
* Using simulated robots and starting `ros2_control` with Gazebo simulator.
* Implementing a controller switching strategy for a robot.
* Using joint limits and transmission concepts in `ros2_control`.


### Example Overview

Check README file inside each example folder for detailed description.

##### Example 1

*RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface.


##### Example 2

....

##### Example 5: "Industrial Robots with externally connected sensor"
*RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot
with an externally connected sensor.

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


## Build status

ROS2 Distro | Branch | Build status | Documentation
:---------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br /> [![Rolling Source Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-source-build.yml?branch=master) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/)
**Rolling - last Focal** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build-last-focal.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build-last-focal.yml?branch=master) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/)
**Galactic** | [`galactic`](https://github.com/ros-controls/ros2_control_demos/tree/galactic) | [![Galactic Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-binary-build.yml?branch=galactic) <br /> [![Galactic Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-semi-binary-build.yml?branch=galactic) <br /> [![Galactic Source Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-source-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-source-build.yml?branch=galactic) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/)
**Foxy** | [`foxy`](https://github.com/ros-controls/ros2_control_demos/tree/foxy) | [![Foxy Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-binary-build.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-semi-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-semi-binary-build.yml?branch=foxy) <br /> [![Foxy Source Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-source-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-source-build.yml?branch=foxy) | [Documentation](https://control.ros.org) <br /> [API Reference](https://control.ros.org/rolling/api/)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


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


## *DiffBot*

*DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
The robot is basically a box moving according to differential drive kinematics.
The *DiffBot* URDF files can be found in `urdf` folder of `diffbot_description` package.

1. To check that *DiffBot* description is working properly use following launch commands:
   ```
   ros2 launch diffbot_description view_robot.launch.py
   ```
   **NOTE**: Getting the following output in terminal is OK: `Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist`.
             This happens because `joint_state_publisher_gui` node need some time to start.

1. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with:
   ```
   ros2 launch ros2_control_demo_bringup diffbot.launch.py
   ```
   The launch file loads and starts the robot hardware, controllers and opens `RViz`.
   In the starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This excessive printing is only added for demonstration. In general, printing to the terminal should be avoided as much as possible in a hardware interface implementation.

   If you can see an orange box in `RViz` everything has started properly.
   Still, to be sure, let's introspect the control system before moving *DiffBot*.

1. Check if the hardware interface loaded properly, by opening another terminal and executing:
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
   The `[claimed]` marker on command interfaces means that a controller has access to command *DiffBot*.

1. Check if controllers are running:
   ```
   ros2 control list_controllers
   ```
   You should get:
   ```
   diffbot_base_controller[diff_drive_controller/DiffDriveController] active
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   ```

1. If everything is fine, now you can send a command to *Diff Drive Controller* using ros2 cli interface:
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
   Also, you should see changing states in the terminal where launch file is started.


Files used for this demos:
  - Launch file: [diffbot.launch.py](ros2_control_demo_bringup/launch/diffbot.launch.py)
  - Controllers yaml: [diffbot_controllers.yaml](ros2_control_demo_bringup/config/diffbot_controllers.yaml)
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


*NOTE:* The examples reuse the same, configurable base-launch file [`rrbot_base.launch.py`](ros2_control_demo_bringup/launch/rrbot_base.launch.py).
This also demonstrates how launch files are usually reused for different scenarios when working with `ros2_control`.


### Example 1: "Industrial Robots with only one interface"

Files:
  - Launch file: [rrbot_system_position_only.launch.py](ros2_control_demo_bringup/launch/rrbot_system_position_only.launch.py)
  - Controllers yaml: [rrbot_controllers.yaml](ros2_control_demo_bringup/config/rrbot_controllers.yaml)
  - URDF:  [rrbot_system_position_only.urdf.xacro](ros2_control_demos/ros2_control_demo_description/rrbot_description/urdf/rrbot_system_position_only.urdf.xacro)
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

Moving the robot:
  - see below description of `forward_position_controller`

Available launch file options:
  - `use_fake_hardware:=true` - start `FakeSystem` instead of hardware.
    This is a simple simulation that mimics joint command to their states.
    This is useful to test *ros2_control* integration and controllers without physical hardware.


### Example 1-Sim: "Industrial Robots with only one interface" (Gazebo simulation)

- **TBA**


### Example 2: "Robots with multiple interfaces"

Files:
  - Launch file: [rrbot_system_multi_interface.launch.py](ros2_control_demo_bringup/launch/rrbot_system_multi_interface.launch.py)
  - Controllers yaml: [rrbot_multi_interface_forward_controllers.yaml](ros2_control_demo_bringup/config/rrbot_multi_interface_forward_controllers.yaml)
  - URDF: [rrbot_system_multi_interface.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot_system_multi_interface.urdf.xacro)
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

Moving the robot:
  - when using velocity controller:
    ```
    ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 5
    - 5"
    ```

  - when using acceleration controller
    ```
    ros2 topic pub /forward_acceleration_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 10
    - 10"
    ```

Useful launch-file options:
  - `robot_controller:=forward_position_controller` - starts demo and spawns position controller.
    Robot can be then controlled using `forward_position_controller` as described below.
  - `robot_controller:=forward_acceleration_controller` - starts demo and spawns acceleration controller.
    Robot can be then controlled using `forward_acceleration_controller` as described below.


### Example 3: "Industrial robot with integrated sensor"

- Launch file: [rrbot_system_with_sensor.launch.py](ros2_control_demo_bringup/launch/rrbot_system_with_sensor.launch.py)
- Controllers: [rrbot_with_sensor_controllers.yaml](ros2_control_demo_bringup/config/rrbot_with_sensor_controllers.yaml)
- URDF: [rrbot_system_with_sensor.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot_system_with_sensor.urdf.xacro)
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
  - Wrench messages are may not be displayed properly in Rviz as NaN values are not handled in Rviz and FTS Broadcaster may send NaN values.

Commanding the robot: see the commands below.

Accessing Wrench data from 2D FTS:
```
ros2 topic echo /fts_broadcaster/wrench
```


### Example 5: "Modular Robots with separate communication to each actuator"

- Launch file: [rrbot_modular_actuators.launch.py](ros2_control_demo_bringup/launch/rrbot_modular_actuators.launch.py)
- Controllers: [rrbot_modular_actuators.yaml](ros2_control_demo_bringup/config/rrbot_modular_actuators.yaml)
- URDF: [rrbot_modular_actuators.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot_modular_actuators.urdf.xacro)
- ros2_control URDF: [rrbot_modular_actuators.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot_modular_actuators.ros2_control.xacro)

- Command interfaces:
  - joint1/position
  - joint2/position
- State interfaces:
  - joint1/position
  - joint2/position

Available controllers:
- `forward_position_controller[forward_command_controller/ForwardCommandController]`
- `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`

Commanding the robot: see the commands below.


## Controllers and moving hardware

To move the robot you should load and start controllers.
The `JointStateBroadcaster` is used to publish the joint states to ROS topics.
Direct joint commands are sent to this robot via the `ForwardCommandController` and `JointTrajectoryController`.
The sections below describe their usage.
Check the [Results](##result) section on how to ensure that things went well.

**NOTE**: Before doing any action with controllers check their state using command:
```
ros2 control list_controllers
```


### JointStateBroadcaster

Open another terminal and load, configure and start `joint_state_broadcaster`:
```
ros2 control set_controller_state joint_state_broadcaster start
```
Check if controller is loaded properly:
```
ros2 control list_controllers
```
You should get the response:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
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
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
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
   ros2 control load_controller position_trajectory_controller --set-state configure
   ```
   Check if the controller is loaded and configured properly:
   ```
   ros2 control list_controllers
   ```
   You should get the response:
   ```
   position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
   ```

2. Now start the controller (and stop other running contorller):
   ```
   ros2 control switch_controllers --stop forward_position_controller --start position_trajectory_controller
   ```
   Check if controllers are activated:
   ```
   ros2 control list_controllers
   ```
   You should get `active` in the response:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
   ```

3. Send a command to the controller using demo node which sends four goals every 6 seconds in a loop:
   ```
   ros2 launch ros2_control_demo_bringup test_joint_trajectory_controller.launch.py
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
