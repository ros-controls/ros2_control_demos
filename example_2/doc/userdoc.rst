:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_2/doc/userdoc.rst

.. _ros2_control_demos_example_2_userdoc:

*********
DiffBot
*********

*DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
The robot is basically a box moving according to differential drive kinematics.

For *example_2*, the hardware interface plugin is implemented having only one interface.

* The communication is done using proprietary API to communicate with the robot control box.
* Data for all joints is exchanged at once.

The *DiffBot* URDF files can be found in ``description/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Tutorial steps
--------------------------

1. To check that *DiffBot* description is working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_2 view_robot.launch.py

   .. warning::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.

   .. image:: diffbot.png
    :width: 400
    :alt: Differential Mobile Robot

2. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_2 diffbot.launch.py

   The launch file loads and starts the robot hardware, controllers and opens *RViz*.
   In the starting terminal you will see a lot of output from the hardware implementation showing its internal states.
   This excessive printing is only added for demonstration. In general, printing to the terminal should be avoided as much as possible in a hardware interface implementation.

   If you can see an orange box in *RViz* everything has started properly.
   Still, to be sure, let's introspect the control system before moving *DiffBot*.

3. Check if the hardware interface loaded properly, by opening another terminal and executing

   .. code-block:: shell

    ros2 control list_hardware_interfaces

   You should get

   .. code-block:: shell

    command interfaces
          left_wheel_joint/velocity [available] [claimed]
          right_wheel_joint/velocity [available] [claimed]
    state interfaces
          left_wheel_joint/position
          left_wheel_joint/velocity
          right_wheel_joint/position
          right_wheel_joint/velocity

   The ``[claimed]`` marker on command interfaces means that a controller has access to command *DiffBot*.

   Furthermore, we can see that the command interface is of type ``velocity``, which is typical for a differential drive robot.

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   You should get

   .. code-block:: shell

    diffbot_base_controller[diff_drive_controller/DiffDriveController] active
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

5. If everything is fine, now you can send a command to *Diff Drive Controller* using ROS 2 CLI interface:

   .. code-block:: shell

    ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
    twist:
      linear:
        x: 0.7
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 1.0"

   You should now see an orange box circling in *RViz*.
   Also, you should see changing states in the terminal where launch file is started.

   .. code-block:: shell

    [DiffBotSystemHardware]: Got command 43.33333 for 'left_wheel_joint'!
    [DiffBotSystemHardware]: Got command 50.00000 for 'right_wheel_joint'!

6. Let's introspect the ros2_control hardware component. Calling

  .. code-block:: shell

    ros2 control list_hardware_components

  should give you

  .. code-block:: shell

    Hardware Component 1
            name: DiffBot
            type: system
            plugin name: ros2_control_demo_example_2/DiffBotSystemHardware
            state: id=3 label=active
            command interfaces
                    left_wheel_joint/velocity [available] [claimed]
                    right_wheel_joint/velocity [available] [claimed]

  This shows that the custom hardware interface plugin is loaded and running. If you work on a real
  robot and don't have a simulator running, it is often faster to use the ``mock_components/GenericSystem``
  hardware component instead of writing a custom one. Stop the launch file and start it again with
  an additional parameter

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_2 diffbot.launch.py use_mock_hardware:=True

  Calling

  .. code-block:: shell

    ros2 control list_hardware_components

  now should give you

  .. code-block:: shell

    Hardware Component 1
        name: DiffBot
        type: system
        plugin name: mock_components/GenericSystem
        state: id=3 label=active
        command interfaces
                left_wheel_joint/velocity [available] [claimed]
                right_wheel_joint/velocity [available] [claimed]

  You see that a different plugin was loaded. Having a look into the `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/ros2_control/diffbot.ros2_control.xacro>`__, one can find the
  instructions to load this plugin together with the parameter ``calculate_dynamics``.

  .. code-block:: xml

    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="calculate_dynamics">true</param>
    </hardware>

  This enables the integration of the velocity commands to the position state interface, which can be
  checked by means of ``ros2 topic echo /joint_states``: The position values are increasing over time if the robot is moving.
  You now can test the setup with the commands from above, it should work identically as the custom hardware component plugin.

  More information on mock_components can be found in the :ref:`ros2_control documentation <mock_components_userdoc>`.

Files used for this demos
--------------------------

* Launch file: `diffbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/launch/diffbot.launch.py>`__
* Controllers yaml: `diffbot_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/config/diffbot_controllers.yaml>`__
* URDF file: `diffbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/urdf/diffbot.urdf.xacro>`__

  * Description: `diffbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/urdf/diffbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/ros2_control/diffbot.ros2_control.xacro>`__

* RViz configuration: `diffbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/rviz/diffbot.rviz>`__

* Hardware interface plugin: `diffbot_system.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/hardware/diffbot_system.cpp>`__


Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
* ``Diff Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/diff_drive_controller>`__): :ref:`doc <diff_drive_controller_userdoc>`
