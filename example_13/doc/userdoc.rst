:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_13/doc/userdoc.rst

.. _ros2_control_demos_example_13_userdoc:

Example 13: Multi-robot system with lifecycle management
==========================================================

- Launch file: [three_robots.launch.py](ros2_control_demo_bringup/launch/three_robots.launch.py)
- Controllers: [three_robots_controllers.yaml](ros2_control_demo_bringup/config/three_robots_controllers.yaml)
- URDF: [three_robots.urdf.xacro](ros2_control_demo_bringup/config/three_robots.urdf.xacro)
- ros2_control URDF: [three_robots.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/three_robots.ros2_control.xacro)


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

1. After starting the example there should be the following scene:
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


2. Activate ``RRBotWithSensor`` and its position controller. Call

  .. code-block:: shell

    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: RRBotSystemWithSensor
    target_state:
      id: 0
      label: active"
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

    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: FakeThreeDofBot
    target_state:
      id: 0
      label: inactive"
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

    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: FakeThreeDofBot
    target_state:
      id: 0
      label: active"
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
    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: RRBotSystemPositionOnly
    target_state:
      id: 0
      label: inactive"

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
    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: RRBotSystemPositionOnly
    target_state:
      id: 0
      label: unconfigured"
    ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
    name: RRBotSystemPositionOnly
    target_state:
      id: 0
      label: finalized"
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
