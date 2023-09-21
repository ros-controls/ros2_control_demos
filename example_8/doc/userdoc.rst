:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_8/doc/userdoc.rst

.. _ros2_control_demos_example_8_userdoc:

********************************************************************************
Example 8: Industrial Robots with an exposed transmission interface
********************************************************************************

*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will use to demonstrate various features.

In this example, both joints use an exposed transmission interface:

  * The communication is done using proprietary API to communicate with the robot control box.
  * Data for all joints is exchanged at once.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_8 view_robot.launch.py

   .. note::

    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to change the configuration for rrbot. It is immediately displayed in *RViz*.

2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_8 rrbot_transmissions_system_position_only.launch.py

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

4. Check if controllers are running by

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

    ros2 launch ros2_control_demo_example_8 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotTransmissionsSystemPositionOnlyHardware]: Command data:
      joint1: 0.5 --> transmission1(R=2) --> actuator1: 1
       joint2: 0.5 --> transmission2(R=4) --> actuator2: 2
    [RRBotTransmissionsSystemPositionOnlyHardware]: State data:
       joint1: 0.383253 <-- transmission1(R=2) <-- actuator1: 0.766505
       joint2: 0.383253 <-- transmission2(R=4) <-- actuator2: 1.53301


Files used for this demos
--------------------------

* Launch file: `rrbot_transmissions_system_position_only.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_8/bringup/launch/rrbot_transmissions_system_position_only.launch.py>`__
* Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_8/bringup/config/rrbot_controllers.yaml>`__
* URDF file: `rrbot_transmissions_system_position_only.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_8/description/urdf/rrbot_transmissions_system_position_only.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `rrbot_transmissions_system_position_only.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_8/description/ros2_control/rrbot_transmissions_system_position_only.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__

* Hardware interface plugin: `rrbot_transmissions_system_position_only.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_8/hardware/rrbot_transmissions_system_position_only.cpp>`__


Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
