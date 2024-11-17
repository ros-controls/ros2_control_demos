:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_15/doc/userdoc.rst

.. _ros2_control_demos_example_15_userdoc:

Example 15: Using multiple controller managers
==============================================

This example shows how to integrate multiple robots under different controller manager instances.

.. include:: ../../doc/run_from_docker.rst

Scenario: Using ros2_control within a local namespace
-----------------------------------------------------

.. note::

  When running ``ros2 control`` CLI commands you have to use additional parameter with exact controller manager node name, i.e., ``-c /rrbot/controller_manager``.

Launch the example with

.. code-block:: shell

  ros2 launch ros2_control_demo_example_15 rrbot_namespace.launch.py

- Command interfaces:

  - joint1/position
  - joint2/position

- State interfaces:

  - joint1/position
  - joint2/position

Available controllers: (nodes under namespace "/rrbot")

.. code-block:: shell

  $ ros2 control list_controllers -c /rrbot/controller_manager
  joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
  forward_position_controller[forward_command_controller/ForwardCommandController] active
  position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive


Commanding the robot using a ``ForwardCommandController`` (name: ``/rrbot/forward_position_controller``)

.. code-block:: shell

  ros2 launch ros2_control_demo_example_15 test_forward_position_controller.launch.py publisher_config:=rrbot_namespace_forward_position_publisher.yaml

Abort the command and switch controller to use ``JointTrajectoryController`` (name: ``/rrbot/position_trajectory_controller``):

.. code-block:: shell

  ros2 control switch_controllers -c /rrbot/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller

Commanding the robot using ``JointTrajectoryController`` (name: ``/rrbot/position_trajectory_controller``)

.. code-block:: shell

  ros2 launch ros2_control_demo_example_15 test_joint_trajectory_controller.launch.py publisher_config:=rrbot_namespace_joint_trajectory_publisher.yaml

Files used for this demo:

* Launch file: `rrbot_namespace.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/launch/rrbot_namespace.launch.py>`__
* Controllers yaml: `rrbot_namespace_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_namespace_controllers.yaml>`__
* URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/description/urdf/rrbot.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/description/ros2_control/rrbot.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
* Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_namespace_forward_position_publisher.yaml>`__
  + `rrbot_joint_trajectory_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/rrbot_namespace_joint_trajectory_publisher.yaml>`__

* Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/hardware/rrbot.cpp>`__


Scenario: Using multiple controller managers on the same machine
----------------------------------------------------------------


.. note::

  When running ``ros2 control`` CLI commands you have to use additional parameter with exact controller manager node name, e.g., ``-c /rrbot_1/controller_manager`` or ``-c /rrbot_2/controller_manager``.

Launch the example with

.. code-block:: shell

  ros2 launch ros2_control_demo_example_15 multi_controller_manager_example_two_rrbots.launch.py

You should see two robots in RViz:

   .. image:: two_rrbot.png
    :width: 400
    :alt: Two Revolute-Revolute Manipulator Robot

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

Available controllers (nodes under namespace ``/rrbot_1`` and ``/rrbot_2``):

.. code-block:: shell

  $ ros2 control list_controllers -c /rrbot_1/controller_manager
  position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
  joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
  forward_position_controller[forward_command_controller/ForwardCommandController] active

  $ ros2 control list_controllers -c /rrbot_2/controller_manager
  joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
  position_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
  forward_position_controller[forward_command_controller/ForwardCommandController] active

Commanding the robots using the ``forward_position_controller`` (of type ``ForwardCommandController``)

.. code-block:: shell

  ros2 launch ros2_control_demo_example_15 test_multi_controller_manager_forward_position_controller.launch.py

Switch controller to use the ``position_trajectory_controller`` (of type ``JointTrajectoryController``) - alternatively start main launch file with argument ``robot_controller:=position_trajectory_controller``:

.. code-block:: shell

  ros2 control switch_controllers -c /rrbot_1/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller
  ros2 control switch_controllers -c /rrbot_2/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller

Commanding the robots using the now activated ``position_trajectory_controller``:

.. code-block:: shell

  ros2 launch ros2_control_demo_example_15 test_multi_controller_manager_joint_trajectory_controller.launch.py


Files used for this demo:

* Launch file: `multi_controller_manager_example_two_rrbots.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/launch/multi_controller_manager_example_two_rrbots.launch.py>`__
* Controllers yaml:
  - `multi_controller_manager_rrbot_1_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/multi_controller_manager_rrbot_1_controllers.yaml>`__
  - `multi_controller_manager_rrbot_2_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/multi_controller_manager_rrbot_2_controllers.yaml>`__
* URDF file: `rrbot.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/description/urdf/rrbot.urdf.xacro>`__

  * Description: `rrbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/urdf/rrbot_description.urdf.xacro>`__
  * ``ros2_control`` tag: `rrbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/description/ros2_control/rrbot.ros2_control.xacro>`__

* RViz configuration: `rrbot.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/rrbot/rviz/rrbot.rviz>`__
* Test nodes goals configuration:

  + `rrbot_forward_position_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/multi_controller_manager_forward_position_publisher.yaml>`__
  + `rrbot_joint_trajectory_publisher <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_15/bringup/config/multi_controller_manager_joint_trajectory_publisher.yaml>`__

* Hardware interface plugin: `rrbot.cpp <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_1/hardware/rrbot.cpp>`__


Controllers from this demo
--------------------------
  * ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
  * ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
  * ``Joint Trajectory Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__
