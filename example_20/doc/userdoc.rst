:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_20/doc/userdoc.rst

.. _ros2_control_demos_example_20_userdoc:

Example 20: Cartesian trajectory execution (Cartesian Trajectory Controller)
=============================================================================

This example demonstrates the ``cartesian_trajectory_controller``. It ingests end-effector *pose*
chunks (``trajectory_msgs/MultiDOFJointTrajectory`` on ``~/cartesian_reference``), interpolates them
in Cartesian space (cubic-spline translation + SLERP orientation), runs differential inverse
kinematics via ``kinematics_interface`` (KDL) to produce an ordinary joint trajectory, and executes
it on top of the joint trajectory controller. A mock Cartesian policy streams pose chunks to a 6-DOF
r6bot on mock hardware so the tool center point (TCP) traces a geometric pattern.

What makes this different from the joint trajectory controller
--------------------------------------------------------------

A ``joint_trajectory_controller`` interpolates in *joint* space. If you give it two joint
configurations and ask it to move between them, the TCP does **not** travel in a straight Cartesian
line - it bows off the line by several centimeters, because linear joint motion maps to a curved
Cartesian path. The ``cartesian_trajectory_controller`` interpolates in *Cartesian* space and IKs
each densely-sampled pose, so the **TCP stays on the commanded path** (a straight LIN line, an arc,
or a smooth reorientation). This example verifies exactly that.

Hardware and interfaces
-----------------------

- MockSystem (``mock_components/GenericSystem``), 6-DOF r6bot

  - Command interfaces: ``joint_1..joint_6/position``
  - State interfaces: ``joint_1..joint_6/position``

- Kinematics: ``kinematics_interface_kdl/KinematicsInterfaceKDL``, chain ``base_link`` -> ``tool0``

Available controllers
---------------------

- ``joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``
- ``cartesian_motion[cartesian_trajectory_controller/CartesianTrajectoryController]``

Tutorial steps
--------------

.. include:: ../../doc/run_from_docker.rst

1. Start the demo (the arm's TCP sweeps a circle in RViz, with the commanded path drawn in green):

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_20 cartesian_demo.launch.py

  Try ``pattern:=square`` or ``pattern:=line`` for other motions.

2. Check controllers:

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    cartesian_motion[cartesian_trajectory_controller/CartesianTrajectoryController] active

3. Verify Cartesian path fidelity (stop the policy first):

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_20 cartesian_demo.launch.py run_policy:=false gui:=false
    ros2 run ros2_control_demo_example_20 verify_cartesian_tracking.py

  The verifier drives the controller through five scenarios and reports PASS/FAIL:

  - **A - LIN straight line:** the TCP stays on the commanded start->goal segment (max perpendicular
    deviation < 5 mm) and reaches the goal.
  - **B - orientation SLERP:** the TCP reorients in place - position held, orientation converging
    monotonically along the shortest arc.
  - **C - multi-pose chunk:** the TCP passes through every waypoint of a Cartesian arc (the
    policy/streaming case).
  - **D - Cartesian smoothness:** the TCP velocity is continuous (no steps).
  - **E - joint-limit violations:** count of samples past a joint limit (expected 0; surfaces the
    deferred joint-limit handling).

  For contrast, a plain ``joint_trajectory_controller`` given the same endpoints in joint space would
  bow the TCP off the straight line by several centimeters; the Cartesian controller keeps it on the
  line (that is the whole point of scenario A).

4. Write the word "ROS" with the tool tip (stop the policy first):

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_20 cartesian_demo.launch.py run_policy:=false
    ros2 run ros2_control_demo_example_20 ros_writer.py

  Each letter is a single continuous Cartesian stroke; the green markers show the intended path.

Files used for this demo
------------------------

- Launch file: `cartesian_demo.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/bringup/launch/cartesian_demo.launch.py>`__
- Controllers yaml: `cartesian_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/bringup/config/cartesian_controllers.yaml>`__
- URDF (xacro): `r6bot_mock.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/description/urdf/r6bot_mock.urdf.xacro>`__ (+ `r6bot_mock.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/description/ros2_control/r6bot_mock.ros2_control.xacro>`__)
- RViz configuration: `r6bot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/description/rviz/r6bot.rviz>`__
- Mock Cartesian policy: `mock_cartesian_policy.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/scripts/mock_cartesian_policy.py>`__
- ROS writer: `ros_writer.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/scripts/ros_writer.py>`__
- Cartesian tracking verification: `verify_cartesian_tracking.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_20/scripts/verify_cartesian_tracking.py>`__

Controllers from this demo
--------------------------

- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Cartesian Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/cartesian_trajectory_controller>`__)
