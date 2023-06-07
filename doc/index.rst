:github_url: https://github.com/ros-controls/ros2_control_demos/blob/|github_branch|/doc/index.rst

.. _ros2_control_demos:

#################
Demos
#################

This `GitHub Repository <https://github.com/ros-controls/ros2_control_demos>`_
provides templates for the development of ros2_control-enabled robots and a simple simulations to demonstrate and prove ros2_control concepts.

If you want to have a rather step by step manual how to do things with ``ros2_control`` checkout `ros-control/roscon2022_workshop <https://github.com/ros-controls/roscon2022_workshop>`_ repository.

==========================================
What you can find in this repository
==========================================

This repository demonstrates the following ``ros2_control`` concepts:

  * Creating a ``HardwareInterface`` for a System, Sensor, and Actuator.
  * Creating a robot description in the form of URDF files.
  * Loading the configuration and starting a robot using launch files.
  * Control of a differential mobile base *DiffBot*.
  * Control of two joints of *RRBot*.
  * Implementing a controller switching strategy for a robot.
  * Using joint limits and transmission concepts in ``ros2_control``.

=====================
Goals
=====================

The repository has two other goals:

1. Implements the example configuration described in the ``ros-controls/roadmap`` repository file `components_architecture_and_urdf_examples <https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md>`_.
2. The repository is a validation environment for ``ros2_control`` concepts, which can only be tested during run-time (e.g., execution of controllers by the controller manager, communication between robot hardware and controllers).

=====================
Example Overview
=====================

Example 1: RRBot
   *RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.


Example 2: DiffBot
   *DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
   The robot is basically a box moving according to differential drive kinematics.


Example 3: "RRBot with multiple interfaces"
   *RRBot* with multiple interfaces.


Example 4: "Industrial robot with integrated sensor"
   *RRBot* with an integrated sensor.


Example 5: "Industrial Robots with externally connected sensor"
   *RRBot* with an externally connected sensor.


Example 6: "Modular Robots with separate communication to each actuator"
   The example shows how to implement robot hardware with separate communication to each actuator.


Example 8: "Using transmissions"
   *RRBot* with an exposed transmission interface.


Example 9: "Gazebo Classic and Gazebo Sim"
   Demonstrates how to switch between simulation and hardware.

=====================
Quick Hints
=====================

These are some quick hints, especially for those coming from a ROS1 control background:

  * There are now three categories of hardware components: *Sensor*, *Actuator*, and *System*.
    *Sensor* is for individual sensors; *Actuator* is for individual actuators; *System* is for any combination of multiple sensors/actuators.
    You could think of a Sensor as read-only.
    All components are used as plugins and therefore exported using ``PLUGINLIB_EXPORT_CLASS`` macro.
  * *ros(1)_control* only allowed three hardware interface types: position, velocity, and effort.
    *ros2_control* allows you to create any interface type by defining a custom string. For example, you might define a ``position_in_degrees`` or a ``temperature`` interface.
    The most common (position, velocity, acceleration, effort) are already defined as constants in hardware_interface/types/hardware_interface_type_values.hpp.
  * Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.
  * In ros2_control, all parameters for the driver are specified in the URDF.
    The ros2_control framework uses the **<ros2_control>** tag in the URDF.
  * Joint names in <ros2_control> tags in the URDF must be compatible with the controller's configuration.

=====================
Examples
=====================
.. toctree::
   :titlesonly:

   Example 1: RRBot <../example_1/doc/userdoc.rst>
   Example 2: DiffBot <../example_2/doc/userdoc.rst>
   Example 3: RRBot with multiple interfaces <../example_3/doc/userdoc.rst>
   Example 4: Industrial robot with integrated sensor <../example_4/doc/userdoc.rst>
   Example 5: Industrial Robots with externally connected sensor <../example_5/doc/userdoc.rst>
   Example 6: Modular Robots with separate communication to each actuator <../example_6/doc/userdoc.rst>
   Example 8: Using transmissions <../example_8/doc/userdoc.rst>
   Example 9: Gazebo Classic and Gazebo <../example_9/doc/userdoc.rst>
