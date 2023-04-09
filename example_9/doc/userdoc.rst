.. _ros2_control_demos_example_9_userdoc:

Example 9: Simulation with RRBot
=================================

*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will
use to demonstrate various features.

It is essentially a double inverted pendulum and demonstrates some fun control concepts within a
simulator and was originally introduced for Gazebo tutorials.

With *example_9*, we demonstrate the interaction of simulators with ros2_control. More specifically,
Gazebo Classic and Gazebo is used for this purpose.


.. note::

   For this example to work, you have to install *gazebo_ros2_control* or build from source by
   adding the following repositories to your workspace:

 - gazebo_ros2_control: ``git clone git@github.com:ros-simulation/gazebo_ros2_control.git --branch master``

.. **NOTE**: For this example to work, you will probably have to use Ignition Gazebo version ``edifice`` by setting up the environment variable with:
..           ```
..           export IGNITION_VERSION=edifice
..           ```
..           and calling ``rosdep install --from-paths . -i -y``
..           Also take care to delete all other version of Gazebo on your computer using:
..           ```
..           sudo apt purge libignition-gazebo3 && sudo apt autoremove
..           ```
..           There are some issues with the current gazebo version "fortress" v6.3.0.

.. **ATTENTION**: The simulation brakes after first movement. This has to be debugged, feel free to help or simply use setup for your robot.

..  - ign_ros2_control: ``git clone https://github.com/ignitionrobotics/ign_ros2_control.git --branch main``
..  - ros_ign: ``git clone https://github.com/ignitionrobotics/ros_ign.git --branch ros2``

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 view_robot.launch.py

   The ``joint_state_publisher_gui`` provides a GUI to change the configuration for *RRbot*. It is immediately displayed in *RViz*.


2. To start *RRBot* with the hardware interface, open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 rrbot.launch.py

   It uses an identical hardware interface as already discussed with *example_9*, see its docs on details on the hardware interface.

3. To start *RRBot* in Gazebo Classic simulation open a terminal, source your ROS2-workspace and Gazebo classic installation first

   .. code-block:: shell

    source /usr/share/gazebo/setup.sh

   Then, execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 rrbot_gazebo_classic.launch.py

   The launch file loads the robot description, starts Gazebo Classic, *Joint State Broadcaster* and *Forward Command Controller*.

   If you can see two orange and one black "box" in Gazebo Classic everything has started properly.

4. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   .. code-block:: shell

    command interfaces
          joint1/position [available] [claimed]
          joint2/position [available] [claimed]
    state interfaces
          joint1/position
          joint2/position

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

5. Check if controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

6. If you get output from above you can send commands to *Forward Command Controller*, either:

   a. Manually using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 0.5
    - 0.5"

   B. Or you can start a demo node which sends two goals every 5 seconds in a loop

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 test_forward_position_controller.launch.py

   You should now see the robot moving in Gazebo Classic.

   If you echo the ``/joint_states`` or ``/dynamic_joint_states`` topics you should now get similar values, namely the simulated states of the robot

   .. code-block:: shell

    ros2 topic echo /joint_states
    ros2 topic echo /dynamic_joint_states


Files used for this demos
-------------------------

- Launch file: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/launch/rrbot.launch.py>`__
- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/rviz/rrbot.rviz>`__
- Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/config/rrbot_forward_position_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/forward_command_controller>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
