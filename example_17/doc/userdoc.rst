:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_17/doc/userdoc.rst

.. _ros2_control_demos_example_17_userdoc:

********************************************************
DiffBot with Chained Controllers using effort interface.
********************************************************

This example shows how to create chained controllers using diff_drive_controller and pid_controllers to control a differential drive robot using effort interfaces. In contrast to *example_16* which uses velocity interface, control via effort interface is very suitable for simulation because it does not break physics (velocity-controlled objects are not affected by inertia etc.). If you haven't already, you can find the instructions for *example_16* in :ref:`ros2_control_demos_example_16_userdoc`. It is recommended to follow the steps given in that tutorial first before proceeding with this one.

This example demonstrates controller chaining as described in :ref:`controller_chaining`. The control chain flows from the diff_drive_controller through two PID controllers to the DiffBot hardware. The diff_drive_controller converts desired robot twist into wheel velocity commands, which are then processed by the PID controllers to control the wheel motors' effort. Additionally, this example shows how to enable the feedforward mode for the PID controllers.

Furthermore, this example shows how to use plotjuggler to visualize the controller states.

The *DiffBot* URDF files can be found in ``description/urdf`` folder.

.. include:: ../../doc/run_from_docker.rst


Tutorial steps
--------------------------

1. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 diffbot.launch.py

  The launch file loads and starts the robot hardware, controllers and opens *RViz* and *Gazebo GUI*.
  In the starting terminal you will see output from the hardware implementation and simulator.

  If you can see an orange box in *RViz* and *Gazebo*, everything has started properly (the robot is pretty small, so make sure to zoom in on the robot in *Gazebo*). Let's introspect the control system before moving *DiffBot*.

2. Check controllers

  .. code-block:: shell

    ros2 control list_controllers

  You should get

  .. code-block:: shell

    joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active
    diffbot_base_controller diff_drive_controller/DiffDriveController      active
    wheel_pids              pid_controller/PidController                   active


3. Check the hardware interface loaded by opening another terminal and executing

  .. code-block:: shell

    ros2 control list_hardware_interfaces

  You should get

  .. code-block:: shell

    command interfaces
      diffbot_base_controller/angular/velocity [available] [unclaimed]
      diffbot_base_controller/linear/velocity [available] [unclaimed]
      left_wheel_joint/effort [available] [claimed]
      right_wheel_joint/effort [available] [claimed]
      wheel_pids/left_wheel_joint/effort [available] [unclaimed]
      wheel_pids/left_wheel_joint/velocity [available] [claimed]
      wheel_pids/right_wheel_joint/effort [available] [unclaimed]
      wheel_pids/right_wheel_joint/velocity [available] [claimed]
    state interfaces
      left_wheel_joint/effort
      left_wheel_joint/position
      left_wheel_joint/velocity
      right_wheel_joint/effort
      right_wheel_joint/position
      right_wheel_joint/velocity
      wheel_pids/left_wheel_joint/effort
      wheel_pids/left_wheel_joint/velocity
      wheel_pids/right_wheel_joint/effort
      wheel_pids/right_wheel_joint/velocity


  The ``[claimed]`` marker on command interfaces means that a controller has access to command *DiffBot*. There are two ``[claimed]`` interfaces from pid_controller, one for left wheel and one for right wheel. These interfaces are referenced by diff_drive_controller. By referencing them, diff_drive_controller can send commands to these interfaces. If you see these, we've successfully chained the controllers.

  There are also four ``[unclaimed]`` interfaces from diff_drive_controller, wheel_pids. You can ignore them since we don't use them in this example.

4. We specified ``feedforward_gain`` as part of ``gains`` in diffbot_chained_controllers.yaml. To actually enable feedforward mode for the pid_controller, we need to use a service provided by pid_controller. Let's enable it.

  .. code-block:: shell

    ros2 service call /wheel_pids/set_feedforward_control std_srvs/srv/SetBool "data: true"

  You should get

  .. code-block:: shell

    response:
    std_srvs.srv.SetBool_Response(success=True, message='')

5. To see the pid_controller in action, let's subscribe to the controler_state topic, e.g. wheel_pids/controller_state topic.

  .. code-block:: shell

    ros2 topic echo /wheel_pids/controller_state

6. Now we are ready to send a command to move the robot. Send a command to *Diff Drive Controller* by opening another terminal and executing

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

  You should now see robot is moving in circles in *RViz* and *Gazebo*.

7. Let's go back to the terminal where we subscribed to the controller_state topic and see the changing states.

  .. code-block:: shell

    header:
      stamp:
        sec: 307
        nanosec: 909000000
      frame_id: ''
    dof_states:
    - name: left_wheel_joint
      reference: 43.33333333333333
      feedback: 43.309162040088395
      feedback_dot: -0.00912764403084563
      error: 0.024171293244933167
      error_dot: .nan
      time_step: 0.01
      output: 8.598921377572319
    - name: right_wheel_joint
      reference: 50.0
      feedback: 50.029347324692814
      feedback_dot: 0.008966862676684506
      error: -0.029347324692814425
      error_dot: .nan
      time_step: 0.01
      output: 10.067540181319677


Visualize the convergence of DiffBot's wheel velocities and commands
---------------------------------------------------------------------

In the section below, we will use *plotjuggler* to observe the convergence of DiffBot's wheel velocities and commands from PID controllers.

*plotjuggler* is an open-source data visualization tool and widely embraced by ROS2 community. If you don't have it installed, you can find the instructions from `plotjuggler website <https://github.com/facontidavide/PlotJuggler>`__.


Before we proceed, we stop all previous steps from terminal and start from the beginning.

1. To start *DiffBot* example open a terminal, source your ROS2-workspace and execute its launch file with

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 diffbot.launch.py

  Like before, if you can see an orange box in *RViz*, everything has started properly.

2. To start the plotjuggler with a provided layout file(plotjuggler.xml), open another terminal and run following command.

  .. code-block:: shell

    ros2 run plotjuggler plotjuggler --layout $(ros2 pkg prefix ros2_control_demo_example_17 --share)/config/plotjuggler.xml

  After this, you will see a few dialogs popping up. For example:

  .. code-block:: shell

    Start the previously used streaming plugin?

    ROS2 Topic Subscriber

  Click 'Yes' for the first dialog and 'OK" to the following two dialogs, then you will see the plotjuggler window.

3. To enable feedforward mode and published a command to move the robot, instead of doing these manually, we will use the demo_test.launch.py. Open another terminal and execute

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_17 demo_test.launch.py

4. From the plotjuggler, you can see the controllers' states and commands being plotted, similar to following figure. From the figure, the DiffBot's wheel velocities and commands from PID controllers are converged to the target velocity fairly quickly.

  .. image:: diffbot_velocities.png
    :width: 600
    :alt: Plotjuggler visualization of DiffBot velocities and commands

5. Change the ``gains`` in the ``diffbot_chained_controllers.yaml`` file with some different values, repeat above steps and observe its effect to the pid_controller commands. For example, to change the ``feedforward_gain`` of the right wheel to 0.50, you can use the following command:

  .. code-block:: shell

    ros2 param set /wheel_pids gains.right_wheel_joint.feedforward_gain 0.50


Files used for this demo
--------------------------

* Launch file: `diffbot.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/launch/diffbot.launch.py>`__
* Controllers yaml: `diffbot_chained_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/config/diffbot_chained_controllers.yaml>`__
* URDF file: `diffbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/urdf/diffbot.urdf.xacro>`__

  * Description: `diffbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/urdf/diffbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `diffbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/ros2_control/diffbot.ros2_control.xacro>`__
  * ``gazebo`` tags: `diffbot.gazebo.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/description/gazebo/diffbot.gazebo.xacro>`__

* RViz configuration: `diffbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/rviz/diffbot.rviz>`__

* Demo helper utility:

  + demo test helper node: `demo_test_helper.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/launch/demo_test_helper.py>`__
  + demo test launch file: `demo_test.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_17/bringup/launch/demo_test.launch.py>`__

Controllers from this demo
--------------------------

* ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): :ref:`doc <joint_state_broadcaster_userdoc>`
* ``Diff Drive Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/diff_drive_controller>`__): :ref:`doc <diff_drive_controller_userdoc>`
* ``pid_controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/pid_controller>`__): :ref:`doc <pid_controller_userdoc>`
