:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_15/doc/userdoc.rst

.. _ros2_control_demos_example_15_userdoc:

Example 15: Using multiple controller managers under different namespaces
=========================================================================

This example shows how to include multiple robots in namespaced controller manager instances.

.. include:: ../../doc/run_from_docker.rst

Scenario showcase: Using ros2_control within a local namespace
----------------------------------------------------------------

* Launch file: `rrbot_namespace.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/launch/rrbot_namespace.launch.py>`__
* Controllers yaml: `rrbot_namespace_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_namespace_controllers.yaml>`__
* URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/description/urdf/rrbot.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/description/ros2_control/rrbot.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
* Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_forward_position_publisher.yaml>`__
  + `rrbot_joint_trajectory_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_joint_trajectory_publisher.yaml>`__

* Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/hardware/rrbot.cpp>`__

.. note::

  When running ``ros2 control`` CLI commands you have to use additional parameter with exact controller manager node name, i.e., ``-c /rrbot/controller_manager``.

- Command interfaces:

  - joint1/position
  - joint2/position

- State interfaces:

  - joint1/position
  - joint2/position

Available controllers: (nodes under namespace "/rrbot")

- ``forward_position_controller[forward_command_controller/ForwardCommandController]``
- ``joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``

List controllers:

.. code-block:: shell

  ros2 control list_controllers -c /rrbot/controller_manager


Commanding the robot using ``/rrbot/forward_position_controller`` (a ``ForwardCommandController``))

.. code-block:: shell

  ros2 launch ros2_control_demo_bringup test_forward_position_controller.launch.py publisher_config:=rrbot_namespace_forward_position_publisher.yaml


Switch controller to use ``JointTrajectoryController`` (name: ``/rrbot/position_trajectory_controller``):

.. code-block:: shell

  ros2 control switch_controllers -c /rrbot/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller

Commanding the robot using ``JointTrajectoryController`` (name: ``/rrbot/position_trajectory_controller``)

.. code-block:: shell

  ros2 launch ros2_control_demo_bringup test_joint_trajectory_controller.launch.py publisher_config:=rrbot_namespace_joint_trajectory_publisher.yaml

Scenario showcase: Using multiple controller managers on the same machine
-------------------------------------------------------------------------

* Launch file: `multi_controller_manager_example_two_rrbots.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/launch/multi_controller_manager_example_two_rrbots.launch.py>`__
* Controllers yaml:
  - `multi_controller_manager_rrbot_1_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/multi_controller_manager_rrbot_1_controllers.yaml>`__
  - `multi_controller_manager_rrbot_2_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/multi_controller_manager_rrbot_2_controllers.yaml>`__
* URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/description/urdf/rrbot.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/description/ros2_control/rrbot.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
* Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_forward_position_publisher.yaml>`__
  + `rrbot_joint_trajectory_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_joint_trajectory_publisher.yaml>`__

* Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/hardware/rrbot.cpp>`__


.. note::

  When running ``ros2 control`` CLI commands you have to use additional parameter with exact controller manager node name, e.g., ``-c /rrbot_1/controller_manager`` or ``-c /rrbot_2/controller_manager``.

``rrbot_1`` namespace:

  - Command interfaces:

    - rrbot_1_joint1/position
    - rrbot_1_joint2/position

  - State interfaces:

    - rrbot_1_joint1/position
    - rrbot_1_joint2/position

``rrbot_2`` namespace:

  - Command interfaces:

    - rrbot_2_joint1/position
    - rrbot_2_joint2/position

  - State interfaces:

    - rrbot_2_joint1/position
    - rrbot_2_joint2/position

Available controllers (nodes under namespace "/rrbot_1" and "/rrbot_2"):

- ``forward_position_controller[forward_command_controller/ForwardCommandController]``
- ``joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``

List controllers:

.. code-block:: shell

  ros2 control list_controllers -c /rrbot_1/controller_manager
  ros2 control list_controllers -c /rrbot_2/controller_manager

Commanding the robot using ``forward_position_controller`` (a ``ForwardCommandController``)


.. code-block:: shell

  ros2 launch ros2_control_demo_bringup test_multi_controller_manager_forward_position_controller.launch.py

Switch controller to use ``position_trajectory_controller`` (a ``JointTrajectoryController``) - alternatively start main launch file with argument ``robot_controller:=position_trajectory_controller``:


.. code-block:: shell

  ros2 control switch_controllers -c /rrbot_1/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller
  ros2 control switch_controllers -c /rrbot_2/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller

Commanding the robot using ``position_trajectory_controller`` (a ``JointTrajectoryController``):

.. code-block:: shell

  ros2 launch ros2_control_demo_bringup test_multi_controller_manager_joint_trajectory_controller.launch.py

Controllers from this demo
--------------------------
  * ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
  * ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
  * ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__
