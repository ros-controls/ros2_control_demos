.. _ros2_control_demos_example_9_userdoc:

Example 9: Simulation with RRBot
=================================

*RRBot*, or ''Revolute-Revolute Manipulator Robot'', is a simple 3-linkage, 2-joint arm that we will
use to demonstrate various features.

It is essentially a double inverted pendulum and demonstrates some fun control concepts within a
simulator and was originally introduced for Gazebo tutorials.

With *example_9*, we demonstrate the interaction of simulators with ros2_control. More specifically,
Gazebo Classic and Gazebo is used for this purpose.


.. note::

   For this example to work, you have to install Gazebo Classic and Gazebo itself. At the time of writing this tutorial,
   it is not possible to install Gazebo Classic and Gazebo Garden at the same time, see the notes in the `official docs <https://gazebosim.org/docs/garden/install_ubuntu>`_. Therefore, we recommend installing

   .. code-block:: shell

    sudo apt-get install ros-rolling-gazebo-ros ignition-fortress ros-rolling-ros-gz

   If like at the time of writing ``ros-rolling-ros-gz`` is not released, one has to `build it from source <https://github.com/gazebosim/ros_gz#from-source>`__ (use the humble branch with fortress, because ros2 only supports garden onwards).

   Then install the ros2_control integrations *gazebo_ros2_control* and *gz_ros2_control* via

   .. code-block:: shell

    sudo apt-get install ros-rolling-gazebo-ros2-control ros-rolling-ign-ros2-control

   or build from source by adding the following repositories to your workspace

   .. code-block:: shell

    git clone git@github.com:ros-simulation/gazebo_ros2_control.git --branch master
    git clone git@github.com:ros-simulation/gz_ros2_control.git --branch master

   and following the build descriptions in the respective repositories.


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

3. To start *RRBot* in the simulators, choose between Gazebo Classic and Gazebo:

   a.  For Gazebo Classic simulation open a terminal, source your ROS2-workspace and Gazebo Classic installation first, i.e., by

    .. code-block:: shell

      source /usr/share/gazebo/setup.sh

    Then, execute the launch file with

    .. code-block:: shell

      ros2 launch ros2_control_demo_example_9 rrbot_gazebo_classic.launch.py

    The launch file loads the robot description, starts Gazebo Classic, *Joint State Broadcaster* and *Forward Command Controller*.
    If you can see two orange and one yellow "box" in Gazebo Classic everything has started properly.

    .. image:: rrbot_gazebo_classic.png
      :width: 400
      :alt: Revolute-Revolute Manipulator Robot in Gazebo Classic

   b.  For Gazebo simulation open a terminal, source your ROS2-workspace and execute the launch file with

    .. code-block:: shell

      ros2 launch ros2_control_demo_example_9 rrbot_gazebo.launch.py

    The launch file loads the robot description, starts Gazebo, *Joint State Broadcaster* and *Forward Command Controller*.
    If you can see two orange and one yellow "box" in Gazebo everything has started properly.

    .. image:: rrbot_gazebo.png
      :width: 400
      :alt: Revolute-Revolute Manipulator Robot in Gazebo

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

   You should now see the robot moving in Gazebo Classic / Gazebo.

   .. note::
    The two simulators show different behavior due to a different implementation of the position interface. For further information see `this comment (Gazebo Classic) <https://github.com/ros-controls/gazebo_ros2_control/pull/172#issuecomment-1441805536>`__ vs. `this discussion (Gazebo) <https://github.com/ros-controls/gz_ros2_control/issues/87>`__.

   If you echo the ``/joint_states`` or ``/dynamic_joint_states`` topics you should see the changing values,
   namely the simulated states of the robot

   .. code-block:: shell

    ros2 topic echo /joint_states
    ros2 topic echo /dynamic_joint_states


Files used for this demos
-------------------------

- Launch files:

  + Hardware: `rrbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/launch/rrbot.launch.py>`__
  + Gazebo Classic: `rrbot_gazebo_classic.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/launch/rrbot_gazebo_classic.launch.py>`__
  + Gazebo `rrbot_gazebo.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/launch/rrbot_gazebo.launch.py>`__

- Controllers yaml: `rrbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/config/rrbot_controllers.yaml>`__
- URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/urdf/rrbot.urdf.xacro>`__

  + Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/urdf/rrbot_description.urdf.xacro>`__
  + ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/ros2_control/rrbot.ros2_control.xacro>`__

- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/description/rviz/rrbot.rviz>`__
- Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/bringup/config/rrbot_forward_position_publisher.yaml>`__

- Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/master/example_9/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/master/forward_command_controller>`__): `doc <https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
