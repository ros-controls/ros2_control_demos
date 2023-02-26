# ros2_control Demos

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides examples for functionalities and capabilities of `ros2_control` framework.
It consists of simple implementations that demonstrate different concepts.

If you want to have rather step by step manual how to do things with `ros2_control` checkout [ros-control/roscon2022_workshop](https://github.com/ros-controls/roscon2022_workshop) repository.


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


## Build status

ROS 2 Distro | Branch | Build status | Documentation
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
