.. _ros2_control_demos_example_2_userdoc:

*********
DiffBot
*********

*DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
The robot is basically a box moving according to differential drive kinematics.

For *example_2*, the hardware interface plugin is implemented having only one interface.

- The communication is done using proprietary API to communicate with the robot control box.
- Data for all joints is exchanged at once.

The *DiffBot* URDF files can be found in ``description/urdf`` folder.

Tutorial steps
--------------------------

1. To check that *DiffBot* description is working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_2 view_robot.launch.py

   .. warning::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.

   .. image:: diffbot.png
    :width: 400
    :alt: Differential Mobile Robot

2. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_2 diffbot.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In the starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This excessive printing is only added for demonstration. In general, printing to the terminal should be avoided as much as possible in a hardware interface implementation.

   If you can see an orange box in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *DiffBot*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   You should get

   .. code-block:: shell

    command interfaces
          left_wheel_joint/velocity [available] [claimed]
          right_wheel_joint/velocity [available] [claimed]
    state interfaces
          left_wheel_joint/position
          left_wheel_joint/velocity
          right_wheel_joint/position
          right_wheel_joint/velocity

   The ``[claimed]`` marker on command interfaces means that a controller has access to command *DiffBot*.

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   You should get

   .. code-block:: shell

    diffbot_base_controller[diff_drive_controller/DiffDriveController] active
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

5. If everything is fine, now you can send a command to *Diff Drive Controller* using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
      x: 0.7
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 1.0"

   You should now see an orange box circling in *RViz*.
   Also, you should see changing states in the terminal where launch file is started.

   .. code-block:: shell

    [DiffBotSystemHardware]: Got command 43.33333 for 'left_wheel_joint'!
    [DiffBotSystemHardware]: Got command 50.00000 for 'right_wheel_joint'!

Files used for this demos
#########################

  - Launch file: `diffbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/bringup/launch/diffbot.launch.py>`__
  - Controllers yaml: `diffbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/bringup/config/diffbot_controllers.yaml>`__
  - URDF file: `diffbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/description/urdf/diffbot.urdf.xacro>`__

    + Description: `diffbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/description/urdf/diffbot_description.urdf.xacro>`__
    + ``ros2_control`` tag: `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/description/ros2_control/diffbot.ros2_control.xacro>`__

  - RViz configuration: `diffbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/description/rviz/diffbot.rviz>`__

  - Hardware interface plugin: `diffbot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/master/example_2/hardware/diffbot_system.cpp>`__


Controllers from this demo
##########################

- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Diff Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>`__
