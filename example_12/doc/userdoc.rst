:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_12/doc/userdoc.rst

.. _ros2_control_demos_example_12_userdoc:

Example 12: Controller chaining with RRBot
===========================================

The example shows how to write a simple chainable controller, and how to integrate it properly to have a functional controller chaining.

For *example_12*, we will use RRBot, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm to demonstrate the controller chaining functionality in ROS2 control.

For *example_12*, a simple chainable ros2_controller has been implemented that takes a vector of interfaces as an input and simple forwards them without any changes. Such a controller is simple known as a ``passthrough_controller``.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_12 view_robot.launch.py

   .. image:: rrbot.png
    :width: 400
    :alt: Revolute-Revolute Manipulator Robot

   The ``joint_state_publisher_gui`` provides a GUI to change the configuration for *RRbot*. It is immediately displayed in *RViz*.


2. To start *RRBot* with the hardware interface, open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_12 rrbot.launch.py

   The launch file loads and starts the robot hardware, controllers and opens RViz. In starting terminal you will see a lot of output from the hardware implementation showing its internal states. It uses an identical hardware interface as already discussed with *example_1*, see its docs on details on the hardware interface.

   If you can see two orange and one yellow rectangle in in RViz everything has started properly. Still, to be sure, let’s introspect the control system before moving RRBot.

3. Check if controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    joint2_position_controller[passthrough_controller/PassthroughController] active
    joint1_position_controller[passthrough_controller/PassthroughController] active

4. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   The output should be something like this:

   .. code-block:: shell

    command interfaces
          joint1/position [available] [claimed]
          joint1_position_controller/joint1/position [unavailable] [unclaimed]
          joint2/position [available] [claimed]
          joint2_position_controller/joint2/position [unavailable] [unclaimed]
    state interfaces
          joint1/position
          joint2/position

   At this stage the reference interfaces of controllers are listed under ``command_interfaces`` when ``ros2 control list_hardware_interfaces`` command is executed.

   * Marker ``[available]`` by command interfaces means that the hardware interfaces are available and are ready to command.

   * Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

   * Marker ``[unavailable]`` by command interfaces means that the hardware interfaces are unavailable and cannot be commanded. For instance, when there is an error in reading or writing an actuator module, it's interfaces are automatically become unavailable.

   * Marker ``[unclaimed]`` by command interfaces means that the reference interfaces of ``joint1_position_controller`` and ``joint2_position_controller`` are not yet in chained mode. However, their reference interfaces are available to be chained, as the controllers are active.

   .. note::

    In case of chained controllers, the command interfaces appear to be ``unavailable`` and ``unclaimed``, even though the controllers whose exposed reference interfaces are active, because these command interfaces become ``available`` only in chained mode i.e., when an another controller makes use of these command interface. In non-chained mode, it is expected for the chained controller to use references from subscribers, hence they are marked as ``unavailable``.

5. To start the complete controller chain, open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_12 launch_chained_controllers.launch.py

   This launch file starts the ``position_controller`` that uses the reference interfaces of both ``joint1_position_controller`` and ``joint2_position_controller`` and streamlines into one, and then the ``forward_position_controller`` uses the reference interfaces of the ``position_controller`` to command the *RRBot* joints.

   .. note::

    The second level ``position_controller`` is only added for demonstration purposes, however, a new chainable controller can be configured to directly command the reference interfaces of both ``joint1_position_controller`` and ``joint2_position_controller``.

6. Check if the new controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    joint2_position_controller[passthrough_controller/PassthroughController] active
    joint1_position_controller[passthrough_controller/PassthroughController] active
    position_controller [passthrough_controller/PassthroughController] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

7. Now check if the interfaces are loaded  properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   The output should be something like this:

   .. code-block:: shell

    command interfaces
          joint1/position [available] [claimed]
          joint1_position_controller/joint1/position [available] [claimed]
          joint2/position [available] [claimed]
          joint2_position_controller/joint2/position [available] [claimed]
          position_controller/joint1_position_controller/joint1/position [available] [claimed]
          position_controller/joint2_position_controller/joint2/position [available] [claimed]
    state interfaces
          joint1/position
          joint2/position

   At this stage the reference interfaces of all the controllers are listed under ``command_interfaces`` should be ``available`` and ``claimed`` when ``ros2 control list_hardware_interfaces`` command is executed.  Marker ``[claimed]`` by command interfaces means that a controller has access to command *RRBot*.

8. If you get output from above you can send commands to *Forward Command Controller*:

   .. code-block:: shell

    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 0.5
    - 0.5"

   You should now see orange and yellow blocks moving in *RViz*.
   Also, you should see changing states in the terminal where launch file is started, e.g.

   .. code-block:: shell

    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 0!
    [RRBotSystemPositionOnlyHardware]: Got command 0.50000 for joint 1!

   If you echo the ``/joint_states`` or ``/dynamic_joint_states`` topics you should now get similar values, namely the simulated states of the robot

   .. code-block:: shell

    ros2 topic echo /joint_states
    ros2 topic echo /dynamic_joint_states

   This clearly shows that the controller chaining is functional, as the commands sent to the ``forward_position_controller`` are passed through properly and then it is reflected in the hardware interfaces of the *RRBot*.


Files used for this demos
-------------------------

- Launch files:

  + Hardware: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/bringup/launch/rrbot.launch.py>`__
  + Controllers: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/bringup/launch/launch_chained_controllers.launch.py>`__
- ROS2 Controller: `passthrough_controller.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/controllers/src/passthrough_controller.cpp>`__
- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/bringup/config/rrbot_chained_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/description/ros2_control/rrbot.ros2_control.xacro>`__
- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_12/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
