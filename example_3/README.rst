************************************************
Example 3: Robots with multiple interface
************************************************


The example shows how to implement multi-interface robot hardware taking care about interfaces used.
The two illegal controllers demonstrate how hardware interface declines faulty claims to access joint command interfaces.

1. To check that *RRBot* descriptions are working properly use following launch commands::

    ros2 launch ros2_control_demo_example_3 view_robot.launch.py

   **NOTE**: Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
   This happens because ``joint_state_publisher_gui`` node need some time to start.
   The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with::

    ros2 launch ros2_control_demo_example_3 rrbot.launch.py

Useful launch-file options:
  - ``robot_controller:=forward_position_controller`` - starts demo and spawns position controller.
    Robot can be then controlled using ``forward_position_controller`` as described below.
  - ``robot_controller:=forward_acceleration_controller`` - starts demo and spawns acceleration controller.
    Robot can be then controlled using ``forward_acceleration_controller`` as described below.

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purposes and should be avoided as much as possible in a hardware interface implementation.

   If you can see two orange and one yellow rectangle in in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *RRBot*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing::

    ros2 control list_hardware_interfaces

   .. code-block:: shell

    command interfaces
          joint1/position [available] [claimed]
          joint2/position [available] [claimed]
    state interfaces
          joint1/position
          joint2/position

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

4. Check is controllers are running::

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   a. Manually using ros2 cli interface.

    - when using velocity controller:

   .. code-block:: shell

    ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 5
    - 5"

     - when using acceleration controller

   .. code-block:: shell

    ros2 topic pub /forward_acceleration_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 10
    - 10"


   b. Or you can start a demo node which sends two goals every 5 seconds in a loop::

        ros2 launch ros2_control_demo_example_3 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 0!
    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 1!


Files used for this demos
#########################

- Launch file: `rrbot_system_multi_interface.launch.py <https://github.com/ros-controls/ros2_control_demos/bringup/launch/rrbot_system_multi_interface.launch.py>`__
- Controllers yaml: `rrbot_multi_interface_forward_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/bringup/config/rrbot_multi_interface_forward_controllers.yaml>`__
- URDF: `rrbot_system_multi_interface.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/description/rrbot_description/urdf/rrbot_system_multi_interface.urdf.xacro>`__

  + ``ros2_control`` URDF tag: `rrbot_system_multi_interface.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/description/rrbot_description/ros2_control/rrbot_system_multi_interface.ros2_control.xacro>`__

- RViz configuration: ?

- Hardware interface plugin: ?

Controllers from this demo
##########################
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
- ``position_controllers``
-  ``velocity_controllers``
