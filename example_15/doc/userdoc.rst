:github_url: https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_15/doc/userdoc.rst

.. _ros2_control_demos_example_15_userdoc:

Example 15: Multi-robot system with namepaced controller_manager
=================================================================

Scenario showcase: Using ros2_control within a local namespace
----------------------------------------------------------------

- Launch file: [rrbot_namespace.launch.py](ros2_control_demo_bringup/launch/rrbot_namespace.launch.py)
- URDF: [rrbot.urdf.xacro](ros2_control_demo_bringup/config/rrbot.yaml)
- ros2_control URDF: [rrbot.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot.ros2_control.xacro)
- Controllers config: [rrbot_namespace_controllers.yaml](ros2_control_demo_bringup/config/rrbot_namespace_controllers.yaml)

**NOTE:**when running `ros2 control` CLI commands you have to use additional parameter with exact controller manager node name, i.e., `-c /rrbot/controller_manager`.

- Command interfaces:
  - joint1/position
  - joint2/position
- State interfaces:
  - joint1/position
  - joint2/position

Available controllers: (nodes under namespace "/rrbot")
- `forward_position_controller[forward_command_controller/ForwardCommandController]`
- `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`

List controllers:
```
ros2 control list_controllers -c /rrbot/controller_manager
```

Commanding the robot using `ForwardCommandController` (name: `/rrbot/forward_position_controller`)
```
ros2 launch ros2_control_demo_bringup test_forward_position_controller.launch.py publisher_config:=rrbot_namespace_forward_position_publisher.yaml
```

Switch controller to use `position_trajectory_controller` (`JointTrajectoryController`):
```
ros2 control switch_controllers -c /rrbot/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller
```

Commanding the robot using `JointTrajectoryController` (name: `/rrbot/position_trajectory_controller`)
```
ros2 launch ros2_control_demo_bringup test_joint_trajectory_controller.launch.py publisher_config:=rrbot_namespace_joint_trajectory_publisher.yaml
```

Scenario showcase: Using multiple controller managers on the same machine
-------------------------------------------------------------------------

- Launch file: [multi_controller_manager_example_two_rrbots.launch.py](ros2_control_demo_bringup/launch/multi_controller_manager_example_two_rrbots.launch.py)
- URDF: [rrbot.urdf.xacro](ros2_control_demo_bringup/config/rrbot.yaml)
- ros2_control URDF: [rrbot.ros2_control.xacro](ros2_control_demo_description/rrbot_description/ros2_control/rrbot.ros2_control.xacro)
- Controllers config 1: [multi_controller_manager_rrbot_1_controllers.yaml](ros2_control_demo_bringup/config/multi_controller_manager_rrbot_1_controllers.yaml)
- Controllers config 2: [multi_controller_manager_rrbot_2_controllers.yaml](ros2_control_demo_bringup/config/multi_controller_manager_rrbot_2_controllers.yaml)

**NOTE:**when running `ros2 control` CLI commands you have to use additional parameter with exact controller manager node name, e.g., `-c /rrbot_1/controller_manager` or `-c /rrbot_2/controller_manager`.

`rrbot_1` namespace:
- Command interfaces:
  - rrbot_1_joint1/position
  - rrbot_1_joint2/position
- State interfaces:
  - rrbot_1_joint1/position
  - rrbot_1_joint2/position

`rrbot_2` namespace:
- Command interfaces:
  - rrbot_2_joint1/position
  - rrbot_2_joint2/position
- State interfaces:
  - rrbot_2_joint1/position
  - rrbot_2_joint2/position

Available controllers: (nodes under namespace "/rrbot_1" and "/rrbot_2")
- `forward_position_controller[forward_command_controller/ForwardCommandController]`
- `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]`

List controllers:
```
ros2 control list_controllers -c /rrbot_1/controller_manager
ros2 control list_controllers -c /rrbot_2/controller_manager
```

Commanding the robot using `ForwardCommandController`s (`forward_position_controller`)
```
ros2 launch ros2_control_demo_bringup test_multi_controller_manager_forward_position_controller.launch.py
```

Switch controller to use `position_trajectory_controller`s (`JointTrajectoryController`) - alternatively start main launch file with argument `robot_controller:=position_trajectory_controller`:
```
ros2 control switch_controllers -c /rrbot_1/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller
ros2 control switch_controllers -c /rrbot_2/controller_manager --deactivate forward_position_controller --activate position_trajectory_controller
```

Commanding the robot using `JointTrajectoryController` (`position_trajectory_controller`):
```
ros2 launch ros2_control_demo_bringup test_multi_controller_manager_joint_trajectory_controller.launch.py
```
