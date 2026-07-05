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

"""Bring up the InferenceBridgeController demo on a mock 2-DOF RRBot."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ros2_control_demo_example_19")
    urdf = os.path.join(pkg, "description", "urdf", "rrbot.urdf")
    rviz = os.path.join(pkg, "description", "rviz", "rrbot.rviz")
    controllers = os.path.join(pkg, "bringup", "config", "bridge_controllers.yaml")
    robot_description = {"robot_description": Path(urdf).read_text()}

    gui = LaunchConfiguration("gui")
    run_policy = LaunchConfiguration("run_policy")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Start RViz2 to visualize the arm."
            ),
            DeclareLaunchArgument(
                "run_policy",
                default_value="true",
                description="Run the mock VLA policy. Set false to drive inference_bridge yourself "
                "(e.g. verify_smoothness.py) without a competing publisher.",
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
                arguments=["inference_bridge", "-c", "/controller_manager", "--param-file", controllers],
                output="screen",
            ),
            # policy_hz MUST equal inference_bridge's policy_frequency (25.0).
            Node(
                package="ros2_control_demo_example_19",
                executable="mock_vla_policy.py",
                name="mock_vla_policy",
                condition=IfCondition(run_policy),
                parameters=[
                    {
                        "controller": "inference_bridge",
                        "joints": ["joint1", "joint2"],
                        "policy_hz": 25.0,
                        "chunk_size": 30,
                        "replan_period": 0.6,
                        "amplitude": 0.5,
                    }
                ],
                output="screen",
            ),
        ]
    )
