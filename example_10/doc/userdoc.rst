.. _ros2_control_demos_example_10_userdoc:

Example 10: Industrial robot with GPIO interfaces
===============================================================

This demo shows how to interact with GPIO interfaces.

The *RRBot* URDF files can be found in the ``description/urdf`` folder.

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_10 view_robot.launch.py

   .. image:: rrbot.png
    :width: 400
    :alt: Revolute-Revolute Manipulator Robot


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_10 rrbot.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purposes and should be avoided as much as possible in a hardware interface implementation.

   If you can see two orange and one yellow rectangle in in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *RRBot*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: console

    ros2 control list_hardware_interfaces

   .. code-block::

    command interfaces
        flange_analog_IOs/analog_output1 [available] [claimed]
        flange_vacuum/vacuum [available] [claimed]
        joint1/position [available] [claimed]
        joint2/position [available] [claimed]
    state interfaces
        flange_analog_IOs/analog_input1
        flange_analog_IOs/analog_input2
        flange_analog_IOs/analog_output1
        flange_vacuum/vacuum
        joint1/position
        joint2/position

   In contrast to the *RRBot* of example_1, you see in addition to the joints now also GPIO interfaces.

4. Check is controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

5. If you get output from above you can send commands to *Forward Command Controller*, either:

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


Files used for this demos
-------------------------

- Launch file: `rrbot.launch.py <bringup/launch/rrbot.launch.py>`__
- Controllers yaml: `rrbot_controllers.yaml <bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <description/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <description/rviz/rrbot.rviz>`__
- Test nodes goals configuration:

  + `rrbot_forward_position_publisher <bringup/config/rrbot_forward_position_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
