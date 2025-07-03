# ros2_control_demos

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides examples for functionalities and capabilities of `ros2_control` framework.
It consists of simple implementations that demonstrate different concepts. Choose the right branch of this repository matching you ROS 2 distribution as well as the full documentation on [control.ros.org](https://control.ros.org), see [this table](#build-status).

If you want to have rather step by step manual how to do things with `ros2_control` checkout the [ros-control/roscon2022_workshop](https://github.com/ros-controls/roscon2022_workshop) repository.

## Contributing

As an open-source project, we welcome each contributor, regardless of their background and experience. Pick a [PR](https://github.com/ros-controls/ros2_control_demos/pulls) and review it, or [create your own](https://github.com/ros-controls/ros2_control_demos/contribute)!
If you are new to the project, please read the [contributing guide](https://control.ros.org/rolling/doc/contributing/contributing.html) for more information on how to get started. We are happy to help you with your first contribution.

## Getting Started

Follow the steps provided in the [documentation](https://control.ros.org/master/doc/ros2_control_demos/doc/index.html#installation) to install ros2_control_demos.

## Content

The following examples are part of this demo repository:

* Example 1: [*RRBot*](example_1)

   *RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.

* Example 2: [*DiffBot*](example_2)

   *DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
   The robot is basically a box moving according to differential drive kinematics.

* Example 3: ["RRBot with multiple interfaces"](example_3)

   *RRBot* with multiple interfaces.

* Example 4: ["Industrial robot with integrated sensor"](example_4)

   *RRBot* with an integrated sensor.

* Example 5: ["Industrial robots with externally connected sensor"](example_5)

   *RRBot* with an externally connected sensor.

* Example 6: ["Modular robots with separate communication to each actuator"](example_6)

   The example shows how to implement robot hardware with separate communication to each actuator.

* Example 7: ["6-DOF robot"](example_7)

   A full tutorial for a 6 DOF robot for intermediate ROS 2 users.

* Example 8: ["Using transmissions"](example_8)

   *RRBot* with an exposed transmission interface.

* Example 9: ["Gazebo simulation"](example_9)

   Demonstrates how to switch between simulation and hardware.

* Example 10: ["Industrial robot with GPIO interfaces"](example_10)

   *RRBot* with GPIO interfaces.

* Example 11: ["Car-like robot using steering controller library"](example_11)

* Example 12: ["Controller chaining"](example_12)

   The example shows a simple chainable controller and its integration to form a controller chain to control the joints of *RRBot*.

* Example 13: ["Multi-robot system with hardware lifecycle management"](example_13)

   This example shows how to handle multiple robots in a single controller manager instance.

* Example 14: ["Modular robots with actuators not providing states and with additional sensors"](example_14)

   The example shows how to implement robot hardware with actuators not providing states and with additional sensors.

* Example 15: ["Using multiple controller managers"](example_15)

   This example shows how to integrate multiple robots under different controller manager instances.

* Example 16: ["DiffBot with Chained Controllers"](example_16)

   This example shows how to create chained controllers using diff_drive_controller and two pid_controllers to control a differential drive robot.

* Example 17: ["RRBot with Hardware Component that publishes diagnostics"](example_17)

   This example shows how to publish diagnostics from a hardware component using the Executor passed from Controller Manager.

## Structure

The repository is structured into `example_XY` folders that fully contained packages with names `ros2_control_demos_example_XY`.

The packages have following structure of subfolders:

* `bringup` - stores launch files and runtime configurations for demo robots.
* `description` - stores URDF (and XACRO) description files, rviz configurations and meshes for the example robots.
* `hardware` - stores implementations of example hardware components (interfaces).
* `controllers` (optional) - stores implementation of example controllers.

The important files to check in each example are:

* `bringup/launch/<example_name>.launch.py` - launch file for the example
* `bringup/config/<example_name>_controllers.yaml` - parameters with controllers' setup for the example.
* `description/<example_name>.ros2_control.xacro` - XACRO file with `ros2_control`-URDF-tag with hardware setup and parameters.
* `description/<example_name>.urdf.xacro` - the main description for for the example used to generate URDF on the fly that is published on the `/robot_description` topic.
* `hardware/<example_name>.hpp` - header file of the example hardware component implementation.
* `hardware/<example_name>.cpp` - source file with the example hardware component implementation.
* `controllers/<example_name>.hpp` - header file of the example controller implementation.
* `controllers/<example_name>.cpp` - source file with the example controller implementation.

**NOTE** - The structure of packages, folders and files given in this repository is not recommended to be used for your robot. Usually you should have all of the above folders defined as separate packages with naming convention `<robot_name_or_type>/[bringup|description|hardware|controllers]`.
  More standard structure can be found in [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman or documentation on [ros_team_workspace](https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html) from Stogl Robotics.

The concepts in this package are demonstrated on the examples of *RRBot* and *DiffBot*.
Those two world-known imaginary robots are trivial simulations to demonstrate and test `ros2_control` concepts.

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml?branch=master) <br> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/rolling/doc/ros2_control_demos/doc/index.html)
**Kilted** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/master) | see above | [Documentation](https://control.ros.org/kilted/doc/ros2_control_demos/doc/index.html)
**Jazzy** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/master) | see above | [Documentation](https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/ros2_control_demos/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-binary-build.yml?branch=master) <br> [![Humble Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/humble/doc/ros2_control_demos/doc/index.html)

## Acknowledgements

The project has received major contributions from companies and institutions [listed on control.ros.org](https://control.ros.org/rolling/doc/acknowledgements/acknowledgements.html)
