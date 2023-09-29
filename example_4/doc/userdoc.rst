:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_4/doc/userdoc.rst

.. _ros2_control_demos_example_4_userdoc:

***************************************************
Example 4: Industrial robot with integrated sensor
***************************************************

This example shows how a sensor can be integrated in a hardware interface:

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.
* Sensor data are exchanged together with joint data
* Examples: KUKA RSI with sensor connected to KRC (KUKA control box) or a prototype robot (ODRI interface).

A 2D Force-Torque Sensor (FTS) is simulated by generating random sensor readings via a hardware interface of
type ``hardware_interface::SystemInterface``.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_4 view_robot.launch.py

   .. note::

    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_4 rrbot_system_with_sensor.launch.py

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
            tcp_fts_sensor/torque.z

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

   Now, let's introspect the hardware components with

   .. code-block:: shell

    ros2 control list_hardware_components -v

   There is a single hardware component for the robot providing the command and state interfaces:

   .. code-block:: shell

    Hardware Component 1
        name: RRBotSystemWithSensor
        type: system
        plugin name: ros2_control_demo_example_4/RRBotSystemWithSensorHardware
        state: id=3 label=active
        command interfaces
                joint1/position [available] [claimed]
                joint2/position [available] [claimed]
        state interfaces
                joint1/position [available]
                joint2/position [available]
                tcp_fts_sensor/force.x [available]
                tcp_fts_sensor/torque.z [available]

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    fts_broadcaster     [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   #. Manually using ROS 2 CLI interface.

      .. code-block:: shell

        ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
        - 0.5
        - 0.5"

   #. Or you can start a demo node which sends two goals every 5 seconds in a loop

      .. code-block:: shell

         ros2 launch ros2_control_demo_example_4 test_forward_position_controller.launch.py

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotSystemWithSensorHardware]: Got command 0.50000 for joint 0!
    [RRBotSystemWithSensorHardware]: Got command 0.50000 for joint 1!

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
        x: 2.946532964706421
        y: .nan
        z: .nan
      torque:
        x: .nan
        y: .nan
        z: 4.0540995597839355

   .. warning::
    Wrench messages are not displayed properly in *RViz* as NaN values are not handled in *RViz* and FTS Broadcaster may send NaN values.


Files used for this demo
--------------------------

* Launch file: `rrbot_system_with_sensor.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/bringup/launch/rrbot_system_with_sensor.launch.py>`__
* Controllers yaml: `rrbot_with_sensor_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/bringup/config/rrbot_with_sensor_controllers.yaml>`__
* URDF: `rrbot_system_with_sensor.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/description/urdf/rrbot_system_with_sensor.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` URDF tag: `rrbot_system_with_sensor.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/description/ros2_control/rrbot_system_with_sensor.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/description/rviz/rrbot.rviz>`__
* Hardware interface plugin: `rrbot_system_with_sensor.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/hardware/rrbot_system_with_sensor.cpp>`__


Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
* ``Force Torque Sensor Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/force_torque_sensor_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html>`__
