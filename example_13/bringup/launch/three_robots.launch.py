# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Authors: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from controller_manager.launch_utils import generate_controllers_spawner_launch_description


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown",
            default_value="50.0",
            description="Slowdown factor of the RRbot.",
        )
    )
    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    slowdown = LaunchConfiguration("slowdown")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_13"),
                    "urdf",
                    "three_robots.urdf.xacro",
                ]
            ),
            " slowdown:=",
            slowdown,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_13"),
            "config",
            "three_robots_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_example_13"), "rviz", "three_robots.rviz"]
    )

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_13"),
            "config",
            "three_robots_position_command_publishers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # global broadcaster and initially active controllers from RRBot
    general_ctrl_spawner = generate_controllers_spawner_launch_description(
        [
            # Global joint state broadcaster
            "joint_state_broadcaster",
            # RRBot controllers
            "rrbot_joint_state_broadcaster",
            "rrbot_position_controller",
            # External FTS broadcaster
            "rrbot_external_fts_broadcaster",
        ],
        controller_params_files=[robot_controllers],
    )

    # RRBot with sensors controllers, initially active
    rrbot_sensor_ctrl_spawner_active = generate_controllers_spawner_launch_description(
        ["rrbot_with_sensor_joint_state_broadcaster", "rrbot_with_sensor_fts_broadcaster"],
        controller_params_files=[robot_controllers],
    )

    # RRBot with sensors controllers, initially inactive
    rrbot_sensor_ctrl_spawner_inactive = generate_controllers_spawner_launch_description(
        [
            "rrbot_with_sensor_position_controller",
        ],
        controller_params_files=[robot_controllers],
        extra_spawner_args=["--inactive"],
    )

    # ThreeDofBot controllers, initially inactive
    threedofbot_ctrl_spawner = generate_controllers_spawner_launch_description(
        [
            "threedofbot_joint_state_broadcaster",
            "threedofbot_position_controller",
            "threedofbot_pid_gain_controller",
        ],
        controller_params_files=[robot_controllers],
        extra_spawner_args=["--inactive"],
    )

    # Command publishers
    rrbot_position_command_publisher = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name="rrbot_position_command_publisher",
        parameters=[position_goals],
    )
    rrbot_with_sensor_position_command_publisher = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name="rrbot_with_sensor_position_command_publisher",
        parameters=[position_goals],
    )
    threedofbot_position_command_publisher = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name="threedofbot_position_command_publisher",
        parameters=[position_goals],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        general_ctrl_spawner,
        rrbot_sensor_ctrl_spawner_active,
        rrbot_sensor_ctrl_spawner_inactive,
        threedofbot_ctrl_spawner,
        rrbot_position_command_publisher,
        rrbot_with_sensor_position_command_publisher,
        threedofbot_position_command_publisher,
    ]

    return LaunchDescription(declared_arguments + nodes)
