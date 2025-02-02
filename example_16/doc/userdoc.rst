:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_16/doc/userdoc.rst

.. _ros2_control_demos_example_16_userdoc:

********************************
DiffBot with Chained Controllers
********************************

*DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive. The robot is basically a box moving according to differential drive kinematics.

*example_16* extends *example_2* by demonstrating controller chaining. It shows how to chain a diff_drive_controller with two pid_controllers (one for each wheel) to achieve coordinated robot motion. The pid_controllers directly control wheel velocities, while the diff_drive_controller converts desired robot twist into wheel velocity commands.

The *DiffBot* URDF files can be found in ``description/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst

Inspired by the scenario outlined in `ROS2 controller manager chaining documentation <https://github.com/ros-controls/ros2_control/blob/master/controller_manager/doc/controller_chaining.rst?plain=1>`__. 
We'll implement a segament of the scenario and cover the chain of 'diff_drive_controller' with two PID controllers. Along with the process, we call out the pattern of contructing virtual interfaces in terms of controllers with details on what is happening behind the scenes. 

Two flows are demonstrated: activation/execution flow and deactivation flow.

In the activation and execution flow, we follow these steps:

  1. First, we activate only the PID controllers to verify proper motor velocity control. The PID controllers accept commands through topics and provide virtual interfaces for chaining.

  2. Next, we activate the diff_drive_controller, which connects to the PID controllers' virtual interfaces. When chained, the PID controllers switch to chained mode and disable their external command topics. This allows us to verify the differential drive kinematics.

  3. Upon activation, the diff_drive_controller provides odometry state interfaces.

  4. Finally, we send velocity commands to test robot movement, with dynamics enabled to demonstrate the PID controllers' behavior.

For the deactivation flow, controllers must be deactivated in the reverse order of their chain. When a controller is deactivated, all controllers that depend on it must also be deactivated. We demonstrate this process step by step and examine the controller states at each stage.

Tutorial steps
--------------------------

1. The first step is to check that *DiffBot* description is working properly use following launch commands

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_16 view_robot.launch.py

   .. warning::
    Getting the following output in terminal is OK: ``Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist``.
    This happens because ``joint_state_publisher_gui`` node need some time to start.

   .. image:: diffbot.png
    :width: 400
    :alt: Differential Mobile Robot

2. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with

   .. code-block:: shell

    ros2 launch ros2_control_demo_example_16 diffbot.launch.py inactive_mode:=true fixed_frame_id:=base_link

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
        diffbot_base_controller/angular/velocity [unavailable] [unclaimed]
        diffbot_base_controller/linear/velocity [unavailable] [unclaimed]
        left_wheel_joint/velocity [available] [claimed]
        pid_controller_left_wheel_joint/left_wheel_joint/velocity [available] [unclaimed]
        pid_controller_right_wheel_joint/right_wheel_joint/velocity [available] [unclaimed]
        right_wheel_joint/velocity [available] [claimed]
    state interfaces
        left_wheel_joint/position
        left_wheel_joint/velocity
        pid_controller_left_wheel_joint/left_wheel_joint/velocity
        pid_controller_right_wheel_joint/right_wheel_joint/velocity
        right_wheel_joint/position
        right_wheel_joint/velocity

   The ``[claimed]`` marker on command interfaces means that a controller has access to command *DiffBot*.

   In this example, diff_drive_controller/DiffDriveController is chinable after controller which another controllre can reference following command interfaces. Both intrfaces are in ``unclaimed`` state, which means that no controller is using them.

   .. code-block:: shell
    command interfaces
        diffbot_base_controller/angular/velocity [available] [unclaimed]
        diffbot_base_controller/linear/velocity [available] [unclaimed]


   
   more, we can see that the command interface is of type ``velocity``, which is typical for a differential drive robot.

4. Check if controllers are running

   .. code-block:: shell

    ros2 control list_controllers

   You should get

   .. code-block:: shell

    joint_state_broadcaster          joint_state_broadcaster/JointStateBroadcaster  active  
    diffbot_base_controller          diff_drive_controller/DiffDriveController      inactive
    pid_controller_right_wheel_joint pid_controller/PidController                   active  
    pid_controller_left_wheel_joint  pid_controller/PidController                   active  


5. Activate the chained diff_drive_controller 

   .. code-block:: shell

    ros2 control switch_controllers --activate diffbot_base_controller

   You should see the following output:

   .. code-block:: shell

    Successfully switched controllers

6. Check the hardware interfaces as well as the controllers

   .. code-block:: shell

    ros2 control list_hardware_interfaces
    ros2 control list_controllers

   You should see the following output:

   .. code-block:: shell

    command interfaces
      diffbot_base_controller/angular/velocity [available] [unclaimed]
      diffbot_base_controller/linear/velocity [available] [unclaimed]
      left_wheel_joint/velocity [available] [claimed]
      pid_controller_left_wheel_joint/left_wheel_joint/velocity [available] [claimed]
      pid_controller_right_wheel_joint/right_wheel_joint/velocity [available] [claimed]
      right_wheel_joint/velocity [available] [claimed]
    state interfaces
      left_wheel_joint/position
      left_wheel_joint/velocity
      pid_controller_left_wheel_joint/left_wheel_joint/velocity
      pid_controller_right_wheel_joint/right_wheel_joint/velocity
      right_wheel_joint/position
      right_wheel_joint/velocity


  .. code-block:: shell

    joint_state_broadcaster          joint_state_broadcaster/JointStateBroadcaster  active
    diffbot_base_controller          diff_drive_controller/DiffDriveController      active
    pid_controller_right_wheel_joint pid_controller/PidController                   active
    pid_controller_left_wheel_joint  pid_controller/PidController                   active



7. Send a command to the left wheel

.. code-block:: shell
  ros2 topic pub -1 /pid_controller_left_wheel_joint/reference control_msgs/msg/MultiDOFCommand "{
    dof_names: ['left_wheel_joint/velocity'],
    values: [1.0],
    values_dot: [0.0]
    }"


8. Send a command to the right wheel

   .. code-block:: shell

    ros2 topic pub -1 /pid_controller_right_wheel_joint/reference control_msgs/msg/MultiDOFCommand "{
      dof_names: ['right_wheel_joint/velocity'],
      values: [1.0],
      values_dot: [0.0]
    }"


7. Change the fixed frame id for rviz to odom


    Look for "Fixed Frame" in the "Global Options" section
  Click on the frame name
  Type or select the new frame ID and change it to odom then click on reset button

8.  If everything is fine, now you can send a command to *Diff Drive Controller* using ROS 2 CLI interface:

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

    [ros2_control_node-1] [INFO] [1721762311.808415917] [controller_manager.resource_manager.hardware_component.system.DiffBot]: Writing commands:
    [ros2_control_node-1]   command 43.33 for 'left_wheel_joint'!
    [ros2_control_node-1]   command 50.00 for 'right_wheel_joint'!

6. Let's introspect the ros2_control hardware component. Calling

  .. code-block:: shell

    ros2 control list_hardware_components

  should give you

  .. code-block:: shell

    Hardware Component 1
            name: DiffBot
            type: system
            plugin name: ros2_control_demo_example_16/DiffBotSystemHardware
            state: id=3 label=active
            command interfaces
                    left_wheel_joint/velocity [available] [claimed]
                    right_wheel_joint/velocity [available] [claimed]

  This shows that the custom hardware interface plugin is loaded and running. If you work on a real
  robot and don't have a simulator running, it is often faster to use the ``mock_components/GenericSystem``
  hardware component instead of writing a custom one. Stop the launch file and start it again with
  an additional parameter

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_16 diffbot.launch.py use_mock_hardware:=True

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
* ``pid_controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/pid_controller>`__): :ref:`doc <pid_controller_userdoc>`

References
--------------------------
https://github.com/ros-controls/roscon_advanced_workshop/blob/9-chaining-controllers/solution/controlko_bringup/config/rrbot_chained_controllers.yaml
