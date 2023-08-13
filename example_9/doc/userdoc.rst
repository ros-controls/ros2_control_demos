:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_9/doc/userdoc.rst

.. _ros2_control_demos_example_9_userdoc:

Example 9: Simulation with RRBot
=================================

With *example_9*, we demonstrate the interaction of simulators with ros2_control. More specifically,
Gazebo Classic is used for this purpose.

.. note::

  Follow the installation instructions on :ref:`ros2_control_demos_install` how to install all dependencies,
  Gazebo Classic should be automatically installed.

  * If you have installed and compiled this repository locally, you can directly use the commands below.
  * If you have installed it via the provided docker image: To run the first two steps of this example (without Gazebo Classic), use the commands as described with :ref:`ros2_control_demos_install`. To run the later steps using Gazebo Classic, execute

    .. code::

      docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_9 rrbot_gazebo_classic.launch.py gui:=false

    first. Then on your local machine you can run the Gazebo Classic client with

    .. code-block:: shell

      gzclient

    and/or ``rviz2`` with

    .. code-block:: shell

      rviz2 -d src/ros2_control_demos/example_9/description/rviz/rrbot.rviz


  For details on the ``gazebo_ros2_control`` plugin, see :ref:`gazebo_ros2_control`.

Tutorial steps
--------------------------

1. To check that *RRBot* descriptions are working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 view_robot.launch.py

   The ``joint_state_publisher_gui`` provides a GUI to change the configuration for *RRbot*. It is immediately displayed in *RViz*.


2. To start *RRBot* with the hardware interface instead of the simulators, open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 rrbot.launch.py

   It uses an identical hardware interface as already discussed with *example_1*, see its docs on details on the hardware interface.

3. To start *RRBot* in the simulators, open a terminal, source your ROS2-workspace and Gazebo Classic installation first, i.e., by

  .. code-block:: shell

    source /usr/share/gazebo/setup.sh

  Then, execute the launch file with

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 rrbot_gazebo_classic.launch.py gui:=true

  The launch file loads the robot description, starts Gazebo Classic, *Joint State Broadcaster* and *Forward Command Controller*.
  If you can see two orange and one yellow "box" in Gazebo Classic everything has started properly.

  .. image:: rrbot_gazebo_classic.png
    :width: 400
    :alt: Revolute-Revolute Manipulator Robot in Gazebo Classic

4. Check if the hardware interface loaded properly, by opening another terminal and executing

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

5. Check if controllers are running by

   .. code-block:: shell

    ros2 control list_controllers

   .. code-block:: shell

    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    forward_position_controller[forward_command_controller/ForwardCommandController] active

6. If you get output from above you can send commands to *Forward Command Controller*, either:

   a. Manually using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    - 0.5
    - 0.5"

   B. Or you can start a demo node which sends two goals every 5 seconds in a loop

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_9 test_forward_position_controller.launch.py

   You should now see the robot moving in Gazebo Classic.

   If you echo the ``/joint_states`` or ``/dynamic_joint_states`` topics you should see the changing values,
   namely the simulated states of the robot

   .. code-block:: shell

    ros2 topic echo /joint_states
    ros2 topic echo /dynamic_joint_states


Files used for this demos
-------------------------

- Launch files:

  + Hardware: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/launch/rrbot.launch.py>`__
  + Gazebo Classic: `rrbot_gazebo_classic.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/launch/rrbot_gazebo_classic.launch.py>`__

- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
- Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/bringup/config/rrbot_forward_position_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_9/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
