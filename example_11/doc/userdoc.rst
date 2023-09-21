:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_11/doc/userdoc.rst

.. _ros2_control_demos_example_11_userdoc:

************
CarlikeBot
************

*CarlikeBot* is a simple mobile base with ackermann drive.

This example shows how to use the ackermann steering controller, which is a sub-design of the steering controller library.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.

The *CarlikeBot* URDF files can be found in ``description/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *CarlikeBot* description is working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_11 view_robot.launch.py

   .. warning::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node needs some time to start.

   .. image:: carlikebot.png
    :width: 400
    :alt: Ackermann Mobile Robot

2. To start *CarlikeBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_11 carlikebot.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In the starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This excessive printing is only added for demonstration. In general, printing to the terminal should be avoided as much as possible in a hardware interface implementation.

   If you can see an orange box in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *CarlikeBot*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   You should get

   .. code-block:: shell

    command interfaces
        ackermann_steering_controller/angular/position [unavailable] [unclaimed]
        ackermann_steering_controller/linear/velocity [unavailable] [unclaimed]
        left_steering_joint/position [available] [claimed]
        rear_left_wheel_joint/velocity [available] [claimed]
        rear_right_wheel_joint/velocity [available] [claimed]
        right_steering_joint/position [available] [claimed]
    state interfaces
        left_steering_joint/position
        rear_left_wheel_joint/position
        rear_left_wheel_joint/velocity
        rear_right_wheel_joint/position
        rear_right_wheel_joint/velocity
        right_steering_joint/position

   The ``[claimed]`` marker on command interfaces means that a controller has access to command *CarlikeBot*.

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   You should get

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    ackermann_steering_controller[ackermann_steering_controller/AckermannSteeringController] active

5. If everything is fine, now you can send a command to *Diff Drive Controller* using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub --rate 30 /ackermann_steering_controller/reference_unstamped geometry_msgs/msg/Twist "linear:
      x: 0.7
      y: 0.0
      z: 0.0
      angular:
      x: 0.0
      y: 0.0
      z: 0.01"

   You should now see an orange box circling in *RViz*.
   Also, you should see changing states in the terminal where launch file is started.

   .. code-block:: shell

    [CarlikeBotSystemHardware]: Got command 0.00464 for 'left_steering_joint'
    [CarlikeBotSystemHardware]: Got command 0.00465 for 'right_steering_joint'
    [CarlikeBotSystemHardware]: Got command 13.97230 for 'rear_left_wheel_joint'
    [CarlikeBotSystemHardware]: Got command 14.02830 for 'rear_right_wheel_joint'

Files used for this demos
--------------------------

* Launch file: `carlikebot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/bringup/launch/carlikebot.launch.py>`__
* Controllers yaml: `carlikebot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/bringup/config/carlikebot_controllers.yaml>`__
* URDF file: `carlikebot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/description/urdf/carlikebot.urdf.xacro>`__

  * Description: `carlikebot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/description/urdf/carlikebot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `carlikebot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/description/ros2_control/carlikebot.ros2_control.xacro>`__

* RViz configuration: `carlikebot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/description/rviz/carlikebot.rviz>`__

* Hardware interface plugin: `carlikebot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_11/hardware/carlikebot_system.cpp>`__


Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Ackermann Steering Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/ackermann_steering_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html>`__
