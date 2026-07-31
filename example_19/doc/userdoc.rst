:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_19/doc/userdoc.rst

.. _ros2_control_demos_example_19_userdoc:

Example 19: Positions-only action-chunk upsampling (JTC)
=====================================================================

This example demonstrates the ``joint_trajectory_controller``'s ``spline_upsampling`` feature, which
ingests positions-only action chunks and upsamples them into smooth C2 cubic-spline commands. A mock
VLA policy streams chunks to a 2-DOF RRBot on mock hardware.

.. note::

  ``spline_upsampling`` is a ``joint_trajectory_controller`` feature (off by default). It is not yet
  in a released ``ros2_controllers``; build ``ros2_controllers`` from the branch carrying the
  feature (`PR #2443 <https://github.com/ros-controls/ros2_controllers/pull/2443>`__).

Hardware and interfaces
-------------------------

- MockSystem (mock_components/GenericSystem)

  - Command interfaces:

    - joint1/position
    - joint1/velocity
    - joint2/position
    - joint2/velocity

  - State interfaces:

    - joint1/position
    - joint1/velocity
    - joint2/position
    - joint2/velocity

Available controllers
-------------------------

- ``joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``
- ``inference_bridge[joint_trajectory_controller/JointTrajectoryController]``

Tutorial steps
--------------------------

.. include:: ../../doc/run_from_docker.rst

1. Start the demo:

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_19 bridge_demo.launch.py

  The mock VLA policy starts streaming positions-only chunks and the arm sweeps smoothly in RViz.

2. Check controllers:

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    inference_bridge[joint_trajectory_controller/JointTrajectoryController] active

3. Run the smoothness verification (stop the policy first):

  .. code-block:: shell

    ros2 launch ros2_control_demo_example_19 bridge_demo.launch.py run_policy:=false gui:=false
    ros2 run ros2_control_demo_example_19 verify_smoothness.py

  Reports C1/C2 continuity and cross-chunk seam metrics for single, sequential, and overlapping chunks.

Files used for this demo
-------------------------

- Launch file: `bridge_demo.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/launch/bridge_demo.launch.py>`__
- Controllers yaml: `bridge_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/bringup/config/bridge_controllers.yaml>`__
- URDF file: `rrbot.urdf <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/description/urdf/rrbot.urdf>`__
- RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/description/rviz/rrbot.rviz>`__
- Mock VLA policy: `mock_vla_policy.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/scripts/mock_vla_policy.py>`__
- Smoothness verification: `verify_smoothness.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_19/scripts/verify_smoothness.py>`__

Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__
