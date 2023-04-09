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

   For this example to work, you have to install add the following repositories to your workspace:

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

**ATTENTION**: The simulation brakes after first movement. This has to be debugged, feel free to help or simply use setup for your robot.

 - ign_ros2_control: ``git clone https://github.com/ignitionrobotics/ign_ros2_control.git --branch main``
 - ros_ign: ``git clone https://github.com/ignitionrobotics/ros_ign.git --branch ros2``

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 view_robot.launch.py

   The ``joint_state_publisher_gui`` provides a GUI to change the configuration for *RRbot*. It is immediately displayed in *RViz*.


2. To start *RRBot* with the hardware interface, open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 rrbot.launch.py

   It uses an identical hardware interface as already discussed with *example_1*, see its docs on details on the hardware interface.


1. To start *RRBot* gazebo simulation open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 rrbot_gazebo.launch.py

   The launch file loads robot description, starts gazebo, ``Joint State Broadcaster`` and ``Joint Trajectory Controller``.

   If you can see two orange and one black "box" in ``Gazebo`` everything has started properly.

2. Check if the hardware interface loaded properly, by opening another terminal and executing

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

3. Check if controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

4. If you get output from above you can send commands to *Forward Command Controller*, either:

   a. Manually using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 0.5
    - 0.5"

   B. Or you can start a demo node which sends two goals every 5 seconds in a loop

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_1 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 0!
    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 1!

   If you echo the ``/joint_states`` or ``/dynamic_joint_states`` topics you should now get similar values, namely the simulated states of the robot

   .. code-block:: shell

    ros2 topic echo /joint_states
    ros2 topic echo /dynamic_joint_states

5. Let's switch to a different controller, the ``Joint Trajectory Controller``.
   Load the controller manually by

   .. code-block:: shell

    ros2 control load_controller joint_trajectory_position_controller

   what should return ``Successfully loaded controller joint_trajectory_position_controller``. Check the status

   .. code-block:: shell

    ros2 control list_controllers

   what shows you that the controller is loaded but unconfigured.

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active
    joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] unconfigured

   Configure the controller by setting it ``inactive`` by

   .. code-block:: shell

    ros2 control set_controller_state joint_trajectory_position_controller inactive

   what should give ``Successfully configured joint_trajectory_position_controller``.

   .. note::

     The parameters are already set in `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/config/rrbot_controllers.yaml>`__
     but the controller was not loaded from the `launch file rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/launch/rrbot.launch.py>`__ before.

   As an alternative, you can load the controller directly in ``inactive``-state by means of the option for ``load_controller``

   .. code-block:: shell

    ros2 control load_controller joint_trajectory_position_controller --set-state inactive

   You should get the result ``Successfully loaded controller joint_trajectory_position_controller into state inactive``.

   See if it loaded properly with

   .. code-block:: shell

    ros2 control list_controllers

   what should now return

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active
    joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] inactive

   Note that the controller is loaded but still ``inactive``. Now you can switch the controller by

   .. code-block:: shell

    ros2 control set_controller_state forward_position_controller inactive
    ros2 control set_controller_state joint_trajectory_position_controller active

   or simply via this one-line command

   .. code-block:: shell

    ros2 control switch_controllers --activate joint_trajectory_position_controller --deactivate forward_position_controller

   Again, check via

   .. code-block:: shell

    ros2 control list_controllers

   what should now return

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] inactive
    joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] active

   Send a command to the controller using demo node, which sends four goals every 6 seconds in a loop:

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_1 test_joint_trajectory_controller.launch.py

   You can adjust the goals in `rrbot_joint_trajectory_publisher <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/config/rrbot_joint_trajectory_publisher.yaml>`__.

Files used for this demos
-------------------------

- Launch file: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/launch/rrbot.launch.py>`__
- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/description/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/description/rviz/rrbot.rviz>`__
- Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/config/rrbot_forward_position_publisher.yaml>`__
  + `rrbot_joint_trajectory_publisher <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/bringup/config/rrbot_joint_trajectory_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/master/example_1/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/forward_command_controller>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
- ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__
