:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_5/doc/userdoc.rst

.. _ros2_control_demos_example_5_userdoc:

*************************************************************
Example 5: Industrial robot with externally connected sensor
*************************************************************

This example shows how an externally connected sensor can be accessed:

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.
* Sensor data are exchanged independently of joint data.
* Examples: KUKA RSI and FTS connected to independent PC with ROS 2.

A 3D Force-Torque Sensor (FTS) is simulated by generating random sensor readings via a hardware interface of
type ``hardware_interface::SensorInterface``.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_5 view_robot.launch.py

   .. note::

    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_5 rrbot_system_with_external_sensor.launch.py

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
        tcp_fts_sensor/force.x
        tcp_fts_sensor/force.y
        tcp_fts_sensor/force.z
        tcp_fts_sensor/torque.x
        tcp_fts_sensor/torque.y

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

   Now, let's introspect the hardware components with

   .. code-block:: shell

    ros2 control list_hardware_components

   There are two hardware components, one for the robot and one for the sensor:

   .. code-block:: shell

    Hardware Component 1
            name: ExternalRRBotFTSensor
            type: sensor
            plugin name: ros2_control_demo_example_5/ExternalRRBotForceTorqueSensorHardware
            state: id=3 label=active
            command interfaces
    Hardware Component 2
            name: RRBotSystemPositionOnly
            type: system
            plugin name: ros2_control_demo_example_5/RRBotSystemPositionOnlyHardware
            state: id=3 label=active
            command interfaces
                    joint1/position [available] [claimed]
                    joint2/position [available] [claimed]

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    forward_position_controller[forward_command_controller/ForwardCommandController] active
    fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   #. Manually using ROS 2 CLI interface.

      .. code-block:: shell

        ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
        - 0.5
        - 0.5"

   #. Or you can start a demo node which sends two goals every 5 seconds in a loop

      .. code-block:: shell

         ros2 launch ros2_control_demo_example_5 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 0!
    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 1!

6. Access wrench data from 2D FTS via

   .. code-block:: shell

    ros2 topic echo /fts_broadcaster/wrench

   shows the random generated sensor values, republished by *Force Torque Sensor Broadcaster* as
   ``geometry_msgs/msg/WrenchStamped`` message

   .. code-block:: shell

    header:
      stamp:
        sec: 1676444704
        nanosec: 332221422
      frame_id: tool_link
    wrench:
      force:
        x: 1.2126582860946655
        y: 2.3202226161956787
        z: 3.4302282333374023
      torque:
        x: 4.540233612060547
        y: 0.647800624370575
        z: 1.7602499723434448

   Wrench data are also visualized in *RViz*:

   .. image:: rrbot_wrench.png
    :width: 400
    :alt: Revolute-Revolute Manipulator Robot with wrench visualization

Files used for this demos
--------------------------

* Launch file: `rrbot_system_with_external_sensor.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/bringup/launch/rrbot_system_with_external_sensor.launch.py>`__
* Controllers yaml: `rrbot_with_external_sensor_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/bringup/config/rrbot_with_external_sensor_controllers.yaml>`__
* URDF: `rrbot_with_external_sensor_controllers.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_5/description/urdf/rrbot_system_with_external_sensor.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` robot: `rrbot_system_position_only.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/description/ros2_control/rrbot_system_position_only.ros2_control.xacro>`__
  * ``ros2_control`` sensor: `external_rrbot_force_torque_sensor.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/description/ros2_control/external_rrbot_force_torque_sensor.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/description/rviz/rrbot.rviz>`__
* Hardware interface plugin:

  * robot `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/hardware/rrbot.cpp>`__
  * sensor `external_rrbot_force_torque_sensor.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/hardware/external_rrbot_force_torque_sensor.cpp>`__

Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
* ``Force Torque Sensor Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html>`__
