:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_13/doc/userdoc.rst

.. _ros2_control_demos_example_13_userdoc:

Example 13: Multi-robot system with lifecycle management
==========================================================

This example shows how to include multiple robots in a single controller manager instance.
Additionally, hardware lifecycle management is demonstrated.

Hardware and interfaces
-------------------------

- RRBotSystemPositionOnly (auto-start)

  - Command interfaces:

    - rrbot_joint1/position
    - rrbot_joint2/position

  - State interfaces:

    - rrbot_joint1/position
    - rrbot_joint2/position

- ExternalRRBotFTSensor (auto-start)

  - State interfaces:

    - rrbot_tcp_fts_sensor/force.x
    - rrbot_tcp_fts_sensor/force.y
    - rrbot_tcp_fts_sensor/force.z
    - rrbot_tcp_fts_sensor/torque.x
    - rrbot_tcp_fts_sensor/torque.y
    - rrbot_tcp_fts_sensor/torque.z

- RRBotSystemWithSensor (auto-configure)

  - Command interfaces:

    - rrbot_with_sensor_joint1/position
    - rrbot_with_sensor_joint2/position

  - State interfaces:

    - rrbot_with_sensor_joint1/position
    - rrbot_with_sensor_joint2/position
    - rrbot_with_sensor_tcp_fts_sensor/force.x
    - rrbot_with_sensor_tcp_fts_sensor/torque.z

- ThreeDofBot

  - Command interfaces

    - threedofbot_joint1/position
    - threedofbot_joint1/pid_gain
    - threedofbot_joint2/position
    - threedofbot_joint2/pid_gain
    - threedofbot_joint3/position
    - threedofbot_joint3/pid_gain

  - State interfaces:

    - threedofbot_joint1/position
    - threedofbot_joint1/pid_gain
    - threedofbot_joint2/position
    - threedofbot_joint2/pid_gain
    - threedofbot_joint3/position
    - threedofbot_joint3/pid_gain

Available controllers
-------------------------

- Global

  - ``joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``

- RRBotSystemPositionOnly

  - ``rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``
  - ``rrbot_position_controller[forward_command_controller/ForwardCommandController]``

- ExternalRRBotFTSensor

  - ``rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster]``

- RRBotSystemPositionOnly

  - ``rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``
  - ``rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController]``
  - ``rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster]``

- FakeThreeDofBot

  - ``threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]``
  - ``threedofbot_position_controller[forward_command_controller/ForwardCommandController]``
  - ``threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController]``


Caveats on hardware lifecycling
--------------------------------

- There is currently no synchronization between available interface and controllers using them.
  This means that you should stop controller before making interfaces they are using unavailable.
  If you don't do this and deactivate/cleanup your interface first your computer will catch fire!
- Global Joint State Broadcaster will not broadcast interfaces that become available after it is started.
  To solve this restart it manually, for now. During restart TF-transforms are not available.
- There is a possibility that hardware lifecycling (state changes) interfere with the ``update``-loop
  if you are trying to start/stop a controller at the same time.


Tutorial steps
--------------------------

.. include:: ../../doc/run_from_docker.rst

1. After starting the example with

  .. code-block:: shell

   ros2 launch ros2_control_demo_example_13 three_robots.launch.py

  there should be the following scene:

  - right robot is moving (RRBotSystemPositionOnly - using auto-start)

    - All interfaces are available and position controller is started and receives commands
    - all controllers running

  - left robot is standing upright (RRBotWithSensor - using auto-configure)

    - only state interfaces are available therefore it can visualized, but not moved
    - only position command controller is not running

  - middle robot is "broken" (FakeThreeDofBot - it is only initialized)

    - no interfaces are available
    - all controllers inactive

  Hardware status:

  .. code-block:: shell

    $ ros2 control list_hardware_components
    Hardware Component 1
      name: FakeThreeDofBot
      type: system
      plugin name: mock_components/GenericSystem
      state: id=1 label=unconfigured
      command interfaces
              threedofbot_joint1/position [unavailable] [unclaimed]
              threedofbot_joint1/pid_gain [unavailable] [unclaimed]
              threedofbot_joint2/position [unavailable] [unclaimed]
              threedofbot_joint2/pid_gain [unavailable] [unclaimed]
              threedofbot_joint3/position [unavailable] [unclaimed]
              threedofbot_joint3/pid_gain [unavailable] [unclaimed]
    Hardware Component 2
            name: RRBotSystemWithSensor
            type: system
            plugin name: ros2_control_demo_hardware/RRBotSystemWithSensorHardware
            state: id=2 label=inactive
            command interfaces
                    rrbot_with_sensor_joint1/position [available] [unclaimed]
                    rrbot_with_sensor_joint2/position [available] [unclaimed]
    Hardware Component 3
            name: ExternalRRBotFTSensor
            type: sensor
            plugin name: ros2_control_demo_hardware/ExternalRRBotForceTorqueSensorHardware
            state: id=3 label=active
            command interfaces
    Hardware Component 4
            name: RRBotSystemPositionOnly
            type: system
            plugin name: ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware
            state: id=3 label=active
            command interfaces
                    rrbot_joint1/position [available] [claimed]
                    rrbot_joint2/position [available] [claimed]

  Controllers status:

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_position_controller[forward_command_controller/ForwardCommandController] active
    rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController] inactive
    threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] inactive
    threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController] inactive
    threedofbot_position_controller[forward_command_controller/ForwardCommandController] inactive


2. Activate ``RRBotWithSensor`` hardware component. Use either the ros2controlcli

  .. code-block:: shell

    ros2 control set_hardware_component_state RRBotSystemWithSensor active

  or call the service manually

  .. code-block:: shell

    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: RRBotSystemWithSensor
    target_state:
      id: 0
      label: active"

  Then activate its position controller via

  .. code-block:: shell

    ros2 control switch_controllers --activate rrbot_with_sensor_position_controller

  Scenario state:

  - right robot is moving
  - left robot is moving
  - middle robot is "broken"

  Hardware status: ``RRBotSystemWithSensor`` is now in active state

  .. code-block:: shell

    $ ros2 control list_hardware_components
    ...
    Hardware Component 2
        name: RRBotSystemWithSensor
        type: system
        plugin name: ros2_control_demo_example_4/RRBotSystemWithSensorHardware
        state: id=3 label=active
        command interfaces
                rrbot_with_sensor_joint1/position [available] [claimed]
                rrbot_with_sensor_joint2/position [available] [claimed]
    ...

  Controllers status: ``rrbot_with_sensor_position_controller`` is now in active state:

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_position_controller[forward_command_controller/ForwardCommandController] active
    rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] inactive
    threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController] inactive
    threedofbot_position_controller[forward_command_controller/ForwardCommandController] inactive


3. Configure ``FakeThreeDofBot`` and its joint state broadcaster and non-movement command interfaces. Call

  .. code-block:: shell

    ros2 control set_hardware_component_state FakeThreeDofBot inactive
    ros2 control switch_controllers --activate threedofbot_joint_state_broadcaster threedofbot_pid_gain_controller

  Scenario state:

  - right robot is moving
  - left robot is moving
  - middle robot is still "broken"

  Hardware status: ``FakeThreeDofBot`` is in inactive state.

  .. code-block:: shell

    $ ros2 control list_hardware_components
    Hardware Component 1
          name: FakeThreeDofBot
          type: system
          plugin name: mock_components/GenericSystem
          state: id=2 label=inactive
          command interfaces
                  threedofbot_joint1/position [available] [unclaimed]
                  threedofbot_joint1/pid_gain [available] [claimed]
                  threedofbot_joint2/position [available] [unclaimed]
                  threedofbot_joint2/pid_gain [available] [claimed]
                  threedofbot_joint3/position [available] [unclaimed]
                  threedofbot_joint3/pid_gain [available] [claimed]
    ...

  Controllers status, ``threedofbot_joint_state_broadcaster`` and ``threedofbot_pid_gain_controller`` are in active state now:

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_position_controller[forward_command_controller/ForwardCommandController] active
    rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_position_controller[forward_command_controller/ForwardCommandController] inactive


4. Restart global joint state broadcaster to broadcast all available states from the framework. First check output to have comparison:

  .. code-block:: shell

    ros2 topic echo /joint_states --once

  Restart:

  .. code-block:: shell

    ros2 control switch_controllers --deactivate joint_state_broadcaster
    ros2 control switch_controllers --activate joint_state_broadcaster

  Check output for comparison, now the joint_states of ``threedofbot`` and ``rrbot_with_sensor`` are broadcasted, too.

  .. code-block:: shell

    ros2 topic echo /joint_states --once

  Scenario state (everything is broken during ``joint_state_broadcaster`` restart):

  - right robot is moving
  - left robot is moving
  - middle robot is now still "standing"

5. Activate ``FakeThreeDofBot`` and its controller. Call

  .. code-block:: shell

    ros2 control set_hardware_component_state FakeThreeDofBot active
    ros2 control switch_controllers --activate threedofbot_position_controller

  Scenario state:

  - right robot is moving
  - left robot is moving
  - middle robot is moving

  Hardware status: ``FakeThreeDofBot`` is in active state.

  .. code-block:: shell

    $ ros2 control list_hardware_components
    Hardware Component 1
        name: FakeThreeDofBot
        type: system
        plugin name: mock_components/GenericSystem
        state: id=3 label=active
        command interfaces
                threedofbot_joint1/position [available] [claimed]
                threedofbot_joint1/pid_gain [available] [claimed]
                threedofbot_joint2/position [available] [claimed]
                threedofbot_joint2/pid_gain [available] [claimed]
                threedofbot_joint3/position [available] [claimed]
                threedofbot_joint3/pid_gain [available] [claimed]
    ...

  Controllers status (all active now):

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_position_controller[forward_command_controller/ForwardCommandController] active
    rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_position_controller[forward_command_controller/ForwardCommandController] active


6. Deactivate ``RRBotSystemPositionOnly`` and its position controller (first). Call

  .. code-block:: shell

    ros2 control switch_controllers --deactivate rrbot_position_controller
    ros2 control set_hardware_component_state RRBotSystemPositionOnly inactive

  Scenario state:

  - right robot is now "standing" at the last position
  - left robot is moving
  - middle robot is moving

  Hardware status: ``RRBotSystemPositionOnly`` is in inactive state.

  .. code-block:: shell

    $ ros2 control list_hardware_components
    ...
    Hardware Component 4
        name: RRBotSystemPositionOnly
        type: system
        plugin name: ros2_control_demo_example_5/RRBotSystemPositionOnlyHardware
        state: id=2 label=inactive
        command interfaces
                rrbot_joint1/position [available] [unclaimed]
                rrbot_joint2/position [available] [unclaimed]
    ...

  Controllers status: ``rrbot_position_controller`` is now in inactive state

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_position_controller[forward_command_controller/ForwardCommandController] inactive
    rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_position_controller[forward_command_controller/ForwardCommandController] active


7. Set ``RRBotSystemPositionOnly`` in unconfigured state, and deactivate its joint state broadcaster. Also restart global joint state broadcaster. Call

  .. code-block:: shell

    ros2 control switch_controllers --deactivate rrbot_joint_state_broadcaster joint_state_broadcaster
    ros2 control set_hardware_component_state RRBotSystemPositionOnly unconfigured
    ros2 control switch_controllers --activate joint_state_broadcaster

  Scenario state (everything is broken during ``joint_state_broadcaster`` restart):

  - right robot is standing still.
  - left robot is moving
  - middle robot is moving

  Controllers status:

  .. code-block:: shell

    $ ros2 control list_controllers
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_external_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] inactive
    rrbot_position_controller[forward_command_controller/ForwardCommandController] inactive
    rrbot_with_sensor_fts_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster] active
    rrbot_with_sensor_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    rrbot_with_sensor_position_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    threedofbot_pid_gain_controller[forward_command_controller/ForwardCommandController] active
    threedofbot_position_controller[forward_command_controller/ForwardCommandController] active

Files used for this demos
-------------------------

- Launch file: `three_robots.launch.py <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_13/bringup/launch/three_robots.launch.py>`__
- Controllers yaml: `three_robots_controllers.yaml <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_13/bringup/config/three_robots_controllers.yaml>`__
- URDF file: `three_robots.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_13/description/urdf/three_robots.urdf.xacro>`__

  + Description: `threedofbot_description.urdf.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/r3bot/urdf/threedofbot_description.urdf.xacro>`__
  + ``ros2_control`` tag:
    + `threedofbot.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_13/description/ros2_control/threedofbot.ros2_control.xacro>`__
    + `rrbot_system_position_only.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_5/description/ros2_control/rrbot_system_position_only.ros2_control.xacro>`__
    + `rrbot_system_with_sensor.ros2_control.xacro <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_4/description/ros2_control/rrbot_system_with_sensor.ros2_control.xacro>`__
- RViz configuration: `three_robots.rviz <https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_13/description/rviz/three_robots.rviz>`__

Controllers from this demo
--------------------------
- ``Joint State Broadcaster`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__
- ``Forward Command Controller`` (`ros2_controllers repository <https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller>`__): `doc <https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`__
