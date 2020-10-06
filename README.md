# ROS2 Control Demos

[![Build Status](https://github.com/ros-controls/ros2_control_demos/workflows/CI/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACI)
[![Linters Status](https://github.com/ros-controls/ros2_control_demos/workflows/Linters/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ALinters)
[![Coverage Status](https://github.com/ros-controls/ros2_control_demos/workflows/Coverage/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions?query=workflow%3ACoverage)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides templates for the development of `ros2_control`-enabled robots and a simple simulation of a robot to demonstrate and prove `ros2_control` concepts.

**ATTENTION: `ros2_control` is currently under heavy development and the any APIs implementation in this repositry, can be broken without any announcement!**

## Goals

The repository has two goals:

1. It provides templates for faster start of implementing own hardware and controllers;
2. The repository is a validation environment for `ros2_control` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager,  communication between robot hardware and controllers).
