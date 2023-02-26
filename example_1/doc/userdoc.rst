.. _ros2_control_demos_example_1_userdoc:

Example 1: RRBot
=====================


*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will use to demonstrate various features.
It is essentially a double inverted pendulum and demonstrates some fun control concepts within a simulator and was originally introduced for Gazebo tutorials.
The *RRBot* URDF files can be found in the ``description/urdf`` folder.

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_1 view_robot.launch.py

   .. note::

     Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
     This happens because ``joint_state_publisher_gui`` node need some time to start.

   The ``joint_state_publisher_gui`` provides a GUI to change the configuration for *RRbot*. It is immediately displayed in *RViz*.

   .. image:: rrbot.png
    :width: 400
    :alt: Revolute-Revolute Manipulator Robot

   The *RViz* setup can be recreated following these steps:

   - The robot models can be visualized using ``RobotModel`` display using ``/robot_description`` topic.
   - Or you can simply open the configuration from ``description/rviz`` folder manually or directly by executing

   .. code-block:: shell

    rviz2 --display-config `ros2 pkg prefix ros2_control_demo_example_1`/share/ros2_control_demo_example_1/rviz/rrbot.rviz


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_1 rrbot.launch.py

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

6. Let's switch to a different controller, the ``Joint Trajectory Controller``.
   Load the controller manually by

   .. code-block:: shell

    ros2 control load_controller position_trajectory_controller

   what should return ``Successfully loaded controller position_trajectory_controller``. Check the status

   .. code-block:: shell

    ros2 control list_controllers

   what shows you that the controller is loaded but unconfigured.

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active
    position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] unconfigured

   Configure the controller by setting it ``inactive`` by

   .. code-block:: shell

    ros2 control set_controller_state position_trajectory_controller inactive

   what should give ``Successfully configured position_trajectory_controller``.

   .. note::

     The parameters are already set in `rrbot_controllers.yaml <bringup/config/rrbot_controllers.yaml>`__
     but the controller was not loaded from the `launch file rrbot.launch.py <bringup/launch/rrbot.launch.py>`__ before.

   As an alternative, you can load the controller directly in ``inactive``-state by means of the option for ``load_controller``

   .. code-block:: shell

    ros2 control load_controller position_trajectory_controller --set-state inactive

   You should get the result ``Successfully loaded controller position_trajectory_controller into state inactive``.

   See if it loaded properly with

   .. code-block:: shell

    ros2 control list_controllers

   what should now return

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active
    position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive

   Note that the controller is loaded but still ``inactive``. Now you can switch the controller by

   .. code-block:: shell

    ros2 control set_controller_state forward_position_controller inactive
    ros2 control set_controller_state position_trajectory_controller active

   or simply via this one-line command

   .. code-block:: shell

    ros2 control switch_controllers --activate position_trajectory_controller --deactivate forward_position_controller

   Again, check via

   .. code-block:: shell

    ros2 control list_controllers

   what should now return

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] inactive
    position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active

   Send a command to the controller using demo node, which sends four goals every 6 seconds in a loop:

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_1 test_joint_trajectory_controller.launch.py

   You can adjust the goals in `rrbot_joint_trajectory_publisher <bringup/config/rrbot_joint_trajectory_publisher.yaml>`__.

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
  + `rrbot_joint_trajectory_publisher <bringup/config/rrbot_joint_trajectory_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
- ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__