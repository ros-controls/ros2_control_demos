:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_6/doc/userdoc.rst

.. _ros2_control_demos_example_6_userdoc:

***********************************************************************
Example 6: Modular Robots with separate communication to each actuator
***********************************************************************

The example shows how to implement robot hardware with separate communication to each actuator:

* The communication is done on actuator level using proprietary or standardized API (e.g., canopen_402, Modbus, RS232, RS485).
* Data for all actuators is exchanged separately from each other.
* Examples: Mara, Arduino-based-robots

This is implemented with a hardware interface of type ``hardware_interface::ActuatorInterface``.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_6 view_robot.launch.py

   .. note::

    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_6 rrbot_modular_actuators.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purposes and should be avoided as much as possible in a hardware interface implementation.

   If you can see two orange and one yellow rectangle in in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *RRBot*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

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

   Now, let's introspect the hardware components with

   .. code-block:: shell

    ros2 control list_hardware_components

   There are two hardware components, one for each actuator and one for each sensor:

   .. code-block:: shell

    Hardware Component 1
            name: RRBotModularJoint2
            type: actuator
            plugin name: ros2_control_demo_example_6/RRBotModularJoint
            state: id=3 label=active
            command interfaces
                    joint2/position [available] [claimed]
    Hardware Component 2
            name: RRBotModularJoint1
            type: actuator
            plugin name: ros2_control_demo_example_6/RRBotModularJoint
            state: id=3 label=active
            command interfaces
                    joint1/position [available] [claimed]

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    forward_position_controller[forward_command_controller/ForwardCommandController] active
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   #. Manually using ROS 2 CLI interface.

      .. code-block:: shell

        ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
        - 0.5
        - 0.5"

   #. Or you can start a demo node which sends two goals every 5 seconds in a loop

      .. code-block:: shell

        ros2 launch ros2_control_demo_example_6 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotModularJoint]: Writing...please wait...
    [RRBotModularJoint]: Got command 0.50000 for joint 'joint1'!
    [RRBotModularJoint]: Joints successfully written!
    [RRBotModularJoint]: Writing...please wait...
    [RRBotModularJoint]: Got command 0.50000 for joint 'joint2'!
    [RRBotModularJoint]: Joints successfully written!


Files used for this demos
--------------------------

* Launch file: `rrbot_modular_actuators.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_6/bringup/launch/rrbot_modular_actuators.launch.py>`__
* Controllers yaml: `rrbot_modular_actuators.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_6/bringup/config/rrbot_modular_actuators.yaml>`__
* URDF: `rrbot_modular_actuators.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_6/description/urdf/rrbot_modular_actuators.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` URDF tag: `rrbot_modular_actuators.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_6/description/ros2_control/rrbot_modular_actuators.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__

* Hardware interface plugin: `rrbot_actuator.cpp <https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_6/hardware/rrbot_actuator.cpp>`__

Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
