:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_14/doc/userdoc.rst

.. _ros2_control_demos_example_14_userdoc:

**************************************************************
Example 14: Modular robot with actuators not providing states
**************************************************************

The example shows how to implement robot hardware with separate communication to each actuator as well as separate sensors for position feedback:

* The communication is done on actuator level using proprietary or standardized API (e.g., canopen_402, Modbus, RS232, RS485).
* Data for all actuators and sensors is exchanged separately from each other
* Examples: Arduino-based-robots, custom robots

This is implemented with hardware interfaces of type ``hardware_interface::ActuatorInterface`` and ``hardware_interface::SensorInterface``.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_14 view_robot.launch.py

   .. note::

    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.
    The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in *RViz*.


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_14 rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.launch.py

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

   There are four hardware components, one for each actuator and one for each sensor:

   .. code-block:: shell

      Hardware Component 1
              name: RRBotModularJoint2
              type: actuator
              plugin name: ros2_control_demo_example_14/RRBotActuatorWithoutFeedback
              state: id=3 label=active
              command interfaces
                      joint2/velocity [available] [claimed]
      Hardware Component 2
              name: RRBotModularJoint1
              type: actuator
              plugin name: ros2_control_demo_example_14/RRBotActuatorWithoutFeedback
              state: id=3 label=active
              command interfaces
                      joint1/velocity [available] [claimed]
      Hardware Component 3
              name: RRBotModularPositionSensorJoint2
              type: sensor
              plugin name: ros2_control_demo_example_14/RRBotSensorPositionFeedback
              state: id=3 label=active
              command interfaces
      Hardware Component 4
              name: RRBotModularPositionSensorJoint1
              type: sensor
              plugin name: ros2_control_demo_example_14/RRBotSensorPositionFeedback
              state: id=3 label=active
              command interfaces


4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_velocity_controller[forward_command_controller/ForwardCommandController] active

5. If you get output from above you can send commands to *Forward Command Controller*:

   .. code-block:: shell

    ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 5
    - 5"

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotActuatorWithoutFeedback]: Writing command: 5.000000
    [RRBotActuatorWithoutFeedback]: Sending data command: 5
    [RRBotActuatorWithoutFeedback]: Joints successfully written!
    [RRBotActuatorWithoutFeedback]: Writing command: 5.000000
    [RRBotActuatorWithoutFeedback]: Sending data command: 5
    [RRBotActuatorWithoutFeedback]: Joints successfully written!
    [RRBotSensorPositionFeedback]: Reading...
    [RRBotSensorPositionFeedback]: Got measured velocity 5.00000
    [RRBotSensorPositionFeedback]: Got state 0.25300 for joint 'joint1'!
    [RRBotSensorPositionFeedback]: Joints successfully read!
    [RRBotSensorPositionFeedback]: Reading...
    [RRBotSensorPositionFeedback]: Got measured velocity 5.00000
    [RRBotSensorPositionFeedback]: Got state 0.25300 for joint 'joint2'!
    [RRBotSensorPositionFeedback]: Joints successfully read!


Files used for this demos
--------------------------

* Launch file: `rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_14/bringup/launch/rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.launch.py>`__
* Controllers yaml: `rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_14/bringup/config/rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.yaml>`__
* URDF: `rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_14/description/urdf/rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` URDF tag: `rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_14/description/ros2_control/rrbot_modular_actuators_without_feedback_sensors_for_position_feedback.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__

* Hardware interface plugins:

  * `rrbot_actuator_without_feedback.cpp <https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_14/hardware/rrbot_actuator_without_feedback.cpp>`__
  * `rrbot_sensor_for_position_feedback.cpp <https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_14/hardware/rrbot_sensor_for_position_feedback.cpp>`__

Controllers from this demo
--------------------------
* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
* ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
