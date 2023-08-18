:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_10/doc/userdoc.rst

.. _ros2_control_demos_example_10_userdoc:

Example 10: Industrial robot with GPIO interfaces
===============================================================

This demo shows how to interact with GPIO interfaces.

The *RRBot* URDF files can be found in the ``description/urdf`` folder.

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_10 view_robot.launch.py


2. To start *RRBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_10 rrbot.launch.py

   The launch file loads and starts the robot hardware and controllers.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: console

    ros2 control list_hardware_interfaces

   .. code-block::

    command interfaces
        flange_analog_IOs/analog_output1 [available] [claimed]
        flange_vacuum/vacuum [available] [claimed]
        joint1/position [available] [claimed]
        joint2/position [available] [claimed]
    state interfaces
        flange_analog_IOs/analog_input1
        flange_analog_IOs/analog_input2
        flange_analog_IOs/analog_output1
        flange_vacuum/vacuum
        joint1/position
        joint2/position

   In contrast to the *RRBot* of example_1, you see in addition to the joints now also GPIO interfaces.

4. Check if controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    gpio_controller     [ros2_control_demo_example_10/GPIOController] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

5. If you get output from above you can subscribe to the ``/gpio_controller/inputs`` topic published by the *GPIO Controller* using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic echo /gpio_controller/inputs

   .. code-block:: shell

    interface_names:
    - flange_analog_IOs/analog_output1
    - flange_analog_IOs/analog_input1
    - flange_analog_IOs/analog_input2
    - flange_vacuum/vacuum
    values:
    - 0.0
    - 1199574016.0
    - 1676318848.0
    - 0.0

6. Now you can send commands to the *GPIO Controller* using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub /gpio_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5,0.7]}"

   You should see a change in the ``/gpio_controller/inputs`` topic and a different output in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotSystemWithGPIOHardware]: Got command 0.5 for GP output 0!
    [RRBotSystemWithGPIOHardware]: Got command 0.7 for GP output 1!


Files used for this demos
-------------------------

- Launch file: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/bringup/launch/rrbot.launch.py>`__
- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__

- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/hardware/rrbot.cpp>`__
- GPIO controller: `gpio_controller.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/controllers/gpio_controller.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
