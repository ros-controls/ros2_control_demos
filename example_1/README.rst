

TODO(destogl): This is not adjusted yet!!


*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will use to demonstrate various features.
It is essentially a double inverted pendulum and demonstrates some fun control concepts within a simulator and was originally introduced for Gazebo tutorials.
The *RRBot* URDF files can be found in the ``urdf`` folder of ``rrbot_description`` package.

1. To check that *RRBot* descriptions are working properly use following launch commands:

   *RRBot*
   ```
   ros2 launch rrbot_description view_robot.launch.py
   ```
   **NOTE**: Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
   This happens because ``joint_state_publisher_gui`` node need some time to start.
   The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in ``Rviz``.


1. To check that *RRBot* descriptions are working properly use following launch commands:

   *RRBot*
   ```
   ros2 launch rrbot_description view_robot.launch.py
   ```
   **NOTE**: Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
   This happens because ``joint_state_publisher_gui`` node need some time to start.
   The ``joint_state_publisher_gui`` provides a GUI to generate  a random configuration for rrbot. It is immediately displayed in ``Rviz``.


1. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with:
   ```
   ros2 launch ros2_control_demo_bringup rrbot.launch.py
   ```
   The launch file loads and starts the robot hardware, controllers and opens ``RViz``.
   In starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This is only of exemplary purposes and should be avoided as much as possible in a hardware interface implementation.

   If you can see two orange and one yellow rectangle in in ``RViz`` everything has started properly.
   Still, to be sure, let's introspect the control system before moving *RRBot*.

1. Check if the hardware interface loaded properly, by opening another terminal and executing:
   ```
   ros2 control list_hardware_interfaces
   ```

   You should get::

    command interfaces
        joint1/position [claimed]
        joint2/position [claimed]
    state interfaces
        joint1/position
        joint2/position

   Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

1. Check is controllers are running:
   ```
   ros2 control list_controllers
   ```
   You should get:
   ```
   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
   forward_position_controller[forward_command_controller/ForwardCommandController] active
   ```

1. If you get output from above you can send commands to *Forward Command Controller*, either:

   a. Manually using ros2 cli interface:
   ```
   ros2 topic pub /position_commands std_msgs/msg/Float64MultiArray "data:
   - 0.5
   - 0.5"
   ```
   B. Or you can start a demo node which sends two goals every 5 seconds in a loop:
   ```
   ros2 launch ros2_control_demo_bringup test_forward_position_controller.launch.py
   ```
   You should now see orange and yellow blocks moving in ``RViz``.
   Also, you should see changing states in the terminal where launch file is started.


Files used for this demos:
  - Launch file: [rrbot.launch.py](ros2_control_demo_bringup/launch/rrbot.launch.py)
  - Controllers yaml: [rrbot_controllers.yaml](ros2_control_demo_bringup/config/rrbot_controllers.yaml)
  - URDF file: [rrbot.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot.urdf.xacro)
    - Description: [rrbot_description.urdf.xacro](ros2_control_demo_description/rrbot_description/urdf/rrbot_description.urdf.xacro)
    - `ros2_control` tag: [rrbot.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot.ros2_control.xacro)
  - RViz configuration: [rrbot.rviz](ros2_control_demo_description/rrbot_description/config/rrbot.rviz)

  - Hardware interface plugin: [rrbot_system_position_only.cpp](ros2_control_demo_hardware/src/rrbot_system_position_only.cpp)


Controllers from this demo:
  - ``Joint State Broadcaster`` ([``ros2_controllers`` repository](https://github.com/ros-controls/ros2_controllers)): [doc](https://ros-controls.github.io/control.ros.org/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)
  - ``Forward Command Controller`` ([``ros2_controllers`` repository](https://github.com/ros-controls/ros2_controllers)): [doc](https://ros-controls.github.io/control.ros.org/ros2_controllers/forward_command_controller/doc/userdoc.html)
