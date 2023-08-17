:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_3/doc/userdoc.rst

.. _ros2_control_demos_example_3_userdoc:

************************************************
Example 3: Robots with multiple interfaces
************************************************

The example shows how to implement multi-interface robot hardware taking care about interfaces used.

For *example_3*, the hardware interface plugin is implemented having multiple interfaces.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.
* Examples: KUKA FRI, ABB Yumi, Schunk LWA4p, etc.

Two illegal controllers demonstrate how hardware interface declines faulty claims to access joint command interfaces.

.. include:: ../../doc/run_from_docker.rst


Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_3 view_robot.launch.py

   .. note::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_3 rrbot_system_multi_interface.launch.py

   Useful launch-file options:

   ``robot_controller:=forward_position_controller``
    starts demo and spawns position controller. Robot can be then controlled using ``forward_position_controller`` as described below.

   ``robot_controller:=forward_acceleration_controller``
    starts demo and spawns acceleration controller. Robot can be then controlled using ``forward_acceleration_controller`` as described below.

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
        joint1/acceleration [available] [unclaimed]
        joint1/position [available] [unclaimed]
        joint1/velocity [available] [claimed]
        joint2/acceleration [available] [unclaimed]
        joint2/position [available] [unclaimed]
        joint2/velocity [available] [claimed]
    state interfaces
        joint1/acceleration
        joint1/position
        joint1/velocity
        joint2/acceleration
        joint2/position
        joint2/velocity

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

4. Check which controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   gives

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_velocity_controller[velocity_controllers/JointGroupVelocityController] active

   Check how this output changes if you use the different launch file arguments described above.

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   #. Manually using ROS 2 CLI interface.

      * when using ``forward_position_controller`` controller

        .. code-block:: shell

          ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
          - 0.5
          - 0.5"

      * when using ``forward_velocity_controller`` controller (default)

        .. code-block:: shell

          ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
          - 5
          - 5"

      * when using ``forward_acceleration_controller`` controller

        .. code-block:: shell

          ros2 topic pub /forward_acceleration_controller/commands std_msgs/msg/Float64MultiArray "data:
          - 10
          - 10"


   #. Or you can start a demo node which sends two goals every 5 seconds in a loop when using ``forward_position_controller`` controller

      .. code-block:: shell

         ros2 launch ros2_control_demo_example_3 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

      [RRBotSystemMultiInterfaceHardware]: Got the commands pos: 0.78500, vel: 0.00000, acc: 0.00000 for joint 0, control_lvl:1
      [RRBotSystemMultiInterfaceHardware]: Got the commands pos: 0.78500, vel: 0.00000, acc: 0.00000 for joint 1, control_lvl:1
      [RRBotSystemMultiInterfaceHardware]: Got pos: 0.78500, vel: 0.00000, acc: 0.00000 for joint 0!
      [RRBotSystemMultiInterfaceHardware]: Got pos: 0.78500, vel: 0.00000, acc: 0.00000 for joint 1!

6. To demonstrate illegal controller configuration, use one of the following launch file arguments:

   * ``robot_controller:=forward_illegal1_controller`` or
   * ``robot_controller:=forward_illegal2_controller``

   You will see the following error messages, because the hardware interface enforces all joints having the same command interface

   .. code-block:: shell

    [ros2_control_node-1] [ERROR] [1676209982.531163501] [resource_manager]: Component 'RRBotSystemMultiInterface' did not accept new command resource combination:
    [ros2_control_node-1]  Start interfaces:
    [ros2_control_node-1] [
    [ros2_control_node-1]   joint1/position
    [ros2_control_node-1] ]
    [ros2_control_node-1] Stop interfaces:
    [ros2_control_node-1] [
    [ros2_control_node-1] ]
    [ros2_control_node-1]
    [ros2_control_node-1] [ERROR] [1676209982.531223835] [controller_manager]: Could not switch controllers since prepare command mode switch was rejected.
    [spawner-4] [ERROR] [1676209982.531717376] [spawner_forward_illegal1_controller]: Failed to activate controller

   Running ``ros2 control list_hardware_interfaces`` shows that no interface is claimed

   .. code-block:: shell

    command interfaces
          joint1/acceleration [available] [unclaimed]
          joint1/position [available] [unclaimed]
          joint1/velocity [available] [unclaimed]
          joint2/acceleration [available] [unclaimed]
          joint2/position [available] [unclaimed]
          joint2/velocity [available] [unclaimed]
    state interfaces
          joint1/acceleration
          joint1/position
          joint1/velocity
          joint2/acceleration
          joint2/position
          joint2/velocity

   and ``ros2 control list_controllers`` indicates that the illegal controller was not loaded

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_illegal1_controller[forward_command_controller/ForwardCommandController] inactive

Files used for this demos
--------------------------

* Launch file: `rrbot_system_multi_interface.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_3/bringup/launch/rrbot_system_multi_interface.launch.py>`__
* Controllers yaml: `rrbot_multi_interface_forward_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_3/bringup/config/rrbot_multi_interface_forward_controllers.yaml>`__
* URDF: `rrbot_system_multi_interface.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_3/description/urdf/rrbot_system_multi_interface.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` URDF tag: `rrbot_system_multi_interface.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_3/description/ros2_control/rrbot_system_multi_interface.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
* Hardware interface plugin: `rrbot_system_multi_interface.cpp <https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_3/hardware/rrbot_system_multi_interface.cpp>`__

Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
