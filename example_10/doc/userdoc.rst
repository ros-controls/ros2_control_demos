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

    [ros2_control_node-1] [INFO] [1721765648.271058850] [controller_manager.resource_manager.hardware_component.system.RRBot]: Writing commands:
    [ros2_control_node-1]   0.50 for GPIO output '0'
    [ros2_control_node-1]   0.70 for GPIO output '1'

7. Let's introspect the ros2_control hardware component. Calling

  .. code-block:: shell

    ros2 control list_hardware_components

  should give you

  .. code-block:: shell

    Hardware Component 1
        name: RRBot
        type: system
        plugin name: ros2_control_demo_example_10/RRBotSystemWithGPIOHardware
        state: id=3 label=active
        command interfaces
                joint1/position [available] [claimed]
                joint2/position [available] [claimed]
                flange_analog_IOs/analog_output1 [available] [claimed]
                flange_vacuum/vacuum [available] [claimed]

  This shows that the custom hardware interface plugin is loaded and running. If you work on a real
  robot and don't have a simulator running, it is often faster to use the ``mock_components/GenericSystem``
  hardware component instead of writing a custom one. Stop the launch file and start it again with
  an additional parameter

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_10 rrbot.launch.py use_mock_hardware:=True

  Calling ``list_hardware_components`` with the ``-v`` option

  .. code-block:: shell

    ros2 control list_hardware_components -v

  now should give you

  .. code-block:: shell

    Hardware Component 1
        name: RRBot
        type: system
        plugin name: mock_components/GenericSystem
        state: id=3 label=active
        command interfaces
                joint1/position [available] [claimed]
                joint2/position [available] [claimed]
                flange_analog_IOs/analog_output1 [available] [claimed]
                flange_vacuum/vacuum [available] [claimed]
        state interfaces
                joint1/position [available]
                joint2/position [available]
                flange_analog_IOs/analog_output1 [available]
                flange_analog_IOs/analog_input1 [available]
                flange_analog_IOs/analog_input2 [available]
                flange_vacuum/vacuum [available]

  One can see that the plugin ``mock_components/GenericSystem`` was now loaded instead: It will mirror the command interfaces to state interfaces with identical name. Call

  .. code-block:: shell

    ros2 topic echo /gpio_controller/inputs

  again and you should see that - unless commands are received - the values of the state interfaces are now ``nan`` except for the vacuum interface.

  .. code-block:: shell

    interface_names:
    - flange_analog_IOs/analog_output1
    - flange_analog_IOs/analog_input1
    - flange_analog_IOs/analog_input2
    - flange_vacuum/vacuum
    values:
    - .nan
    - .nan
    - .nan
    - 1.0

  This is, because for the vacuum interface an initial value of ``1.0`` is set in the URDF file.

  .. code-block:: xml

      <gpio name="flange_vacuum">
        <command_interface name="vacuum"/>
        <state_interface name="vacuum">
          <param name="initial_value">1.0</param>
        </state_interface>
      </gpio>

  Call again

  .. code-block:: shell

    ros2 topic pub /gpio_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5,0.7]}"

  and you will see that the GPIO command interfaces will be mirrored to their respective state interfaces.

Files used for this demos
-------------------------

- Launch file: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/bringup/launch/rrbot.launch.py>`__
- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__

- Hardware interface plugin:

  + `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/hardware/rrbot.cpp>`__
  + `generic_system.cpp <https://github.com/ros-controls/ros2_control/tree/{REPOS_FILE_BRANCH}/hardware_interface/src/mock_components/generic_system.cpp>`__

- GPIO controller: `gpio_controller.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_10/controllers/gpio_controller.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): :ref:`doc <forward_command_controller_userdoc>`
