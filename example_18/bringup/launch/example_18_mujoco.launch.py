#!/usr/bin/env python3
#
# Copyright (C) 2026 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Julia Jia

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            "xacro",
            " ",
            PathSubstitution(FindPackageShare("ros2_control_demo_description"))
            / "openduckmini"
            / "urdf"
            / "open_duck_mini.urdf.xacro",
            " ",
            "headless:=",
            LaunchConfiguration("headless"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": True,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run simulation without visualization window (use true for CI/headless)",
            ),
            # Publishes TF from joint states; simulation model from MuJoCo
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                output="both",
                parameters=[
                    {"use_sim_time": True},
                    ParameterFile(
                        PathSubstitution(FindPackageShare("ros2_control_demo_example_18"))
                        / "config"
                        / "open_duck_mini_controllers.yaml",
                    ),
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawn_joint_state_broadcaster",
                arguments=[
                    "joint_state_broadcaster",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_18"))
                    / "config"
                    / "open_duck_mini_controllers.yaml",
                ],
                output="both",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawn_state_interfaces_broadcaster",
                arguments=[
                    "state_interfaces_broadcaster",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_18"))
                    / "config"
                    / "open_duck_mini_controllers.yaml",
                ],
                output="both",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawn_motion_controller",
                arguments=[
                    "motion_controller",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_18"))
                    / "config"
                    / "open_duck_mini_controllers.yaml",
                ],
                output="both",
            ),
        ]
    )
