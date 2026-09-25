#!/usr/bin/env python3
# Copyright 2026 ros2_control Development Team
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

"""Bring up the Cartesian trajectory controller demo on a mock 6-DOF r6bot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("ros2_control_demo_example_20")
    rviz = os.path.join(pkg, "description", "rviz", "r6bot.rviz")
    controllers = os.path.join(pkg, "bringup", "config", "cartesian_controllers.yaml")

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_example_20"),
                        "description",
                        "urdf",
                        "r6bot_mock.urdf.xacro",
                    ]
                ),
            ]
        )
    }

    gui = LaunchConfiguration("gui")
    run_policy = LaunchConfiguration("run_policy")
    pattern = LaunchConfiguration("pattern")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Start RViz2 to visualize the arm."
            ),
            DeclareLaunchArgument(
                "run_policy",
                default_value="true",
                description="Stream the mock Cartesian policy. Set false to drive cartesian_motion "
                "yourself (e.g. verify_cartesian_tracking.py) without a competing publisher.",
            ),
            DeclareLaunchArgument(
                "pattern",
                default_value="circle",
                description="Default motion streamed by the mock policy: circle | square | line.",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz],
                condition=IfCondition(gui),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "cartesian_motion",
                    "-c",
                    "/controller_manager",
                    "--param-file",
                    controllers,
                ],
                output="screen",
            ),
            Node(
                package="ros2_control_demo_example_20",
                executable="mock_cartesian_policy.py",
                name="mock_cartesian_policy",
                condition=IfCondition(run_policy),
                parameters=[{"controller": "cartesian_motion", "pattern": pattern}],
                output="screen",
            ),
        ]
    )
