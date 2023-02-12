***************************************************
Example 4: Industrial robot with integrated sensor
***************************************************


The example shows how to implement multi-interface robot hardware taking care about interfaces used.
The two illegal controllers demonstrate how hardware interface declines faulty claims to access joint command interfaces.

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_4 view_robot.launch.py

   **NOTE**: Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
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

4. Check is controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active    
    fts_broadcaster     [force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active    
    forward_position_controller[forward_command_controller/ForwardCommandController] active   

5. If you get output from above you can send commands to *Forward Command Controller*, either:

   #. Manually using ros2 cli interface.

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

    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 0!
    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 1!

6. Accessing Wrench data from 2D FTS:

  .. code-block:: shell

    ros2 topic echo /fts_broadcaster/wrench


  .. warning::
    Wrench messages are may not be displayed properly in Rviz as NaN values are not handled in Rviz and FTS Broadcaster may send NaN values.


Files used for this demos
#########################

- Launch file: `rrbot_system_with_sensor.launch.py <https://github.com/ros-controls/ros2_control_demos/example_4/bringup/launch/rrbot_system_with_sensor.launch.py>`__
- Controllers yaml: `rrbot_with_sensor_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/example_4/bringup/config/rrbot_with_sensor_controllers.yaml>`__
- URDF: `rrbot_system_with_sensor.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/example_4/description/urdf/rrbot_system_with_sensor.urdf.xacro>`__

  + ``ros2_control`` URDF tag: `rrbot_system_with_sensor.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/example_4/description/ros2_control/rrbot_system_with_sensor.ros2_control.xacro>`__

- RViz configuration: ?

- Hardware interface plugin: ?

Controllers from this demo
##########################
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
