# ros2_control Demos

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides examples for functionalities and capabilities of `ros2_control` framework.
It consists of simple implementations that demonstrate different concepts.

If you want to have rather step by step manual how to do things with `ros2_control` checkout the [ros-control/roscon2022_workshop](https://github.com/ros-controls/roscon2022_workshop) repository.

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


* Example 5: ["Industrial Robots with externally connected sensor"](example_5)

   *RRBot* with an externally connected sensor.

* Example 6: ["Modular Robots with separate communication to each actuator"](example_6)

   The example shows how to implement robot hardware with separate communication to each actuator.

* Example 7: "Multi-robot example (tba.)"

* Example 8: ["Using transmissions"](example_8)

   *RRBot* with an exposed transmission interface.

* Example 9: "Gazebo Classic and Gazebo Sim (tba.)"

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

## Running
### Modifying or building your own

```bash
cd Dockerfile
docker build -t ros2_control_demos .
```

### To run the demo

#### Using Docker

Docker allows us to run the demo without the GUI if configured properly. The following command runs the demo without the GUI:

Then on your local machine, you can run the Gazebo client:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.sh
rviz2 -d src/ros2_control_demos/example_2/description/rviz/diffbot.rviz
```


## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br /> | [Documentation](https://control.ros.org/master/index.html) <br /> [API Reference](https://control.ros.org/master/doc/api/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/ros2_control_demos/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-binary-build.yml?branch=humble) <br /> [![Humble Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=humble)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-semi-binary-build.yml?branch=humble) <br /> | [Documentation](https://control.ros.org/humble/index.html) <br /> [API Reference](https://control.ros.org/humble/doc/api/index.html)
**Galactic** | [`galactic`](https://github.com/ros-controls/ros2_control_demos/tree/galactic) | [![Galactic Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-binary-build.yml?branch=galactic) <br /> [![Galactic Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=galactic)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/galactic-semi-binary-build.yml?branch=galactic) <br /> | [Documentation](https://control.ros.org/galactic/index.html) <br /> [API Reference](https://control.ros.org/galactic/doc/api/index.html)
**Foxy** | [`foxy`](https://github.com/ros-controls/ros2_control_demos/tree/foxy) | [![Foxy Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-binary-build.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-semi-binary-build.yml/badge.svg?branch=foxy)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/foxy-semi-binary-build.yml?branch=foxy) <br /> | [Documentation](https://control.ros.org/foxy/index.html) <br /> [API Reference](https://control.ros.org/foxy/doc/api/index.html)

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

**NOTE**: `ros2_control` and `ros2_controllers` packages are released and can be installed using a package manager.
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
