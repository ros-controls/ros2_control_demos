# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix",
                default_value='""',
                description=(
                    "Prefix of the joint names, useful for "
                    "multi-robot setup. If changed than also joint names in the controllers' configuration "
                    "have to be updated."
                ),
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "mock_sensor_commands",
                default_value="false",
                description=(
                    "Enable mocked command interfaces for sensors used for simple simulations. "
                    "Used only if 'use_mock_hardware' parameter is true."
                ),
            ),
            DeclareLaunchArgument(
                "slowdown",
                default_value="50.0",
                description="Slowdown factor of the RRbot.",
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="forward_position_controller",
                description="Robot controller to start.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            # Control node
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_6"))
                    / "config"
                    / "rrbot_modular_actuators.yaml"
                ],
                output="both",
            ),
            # robot_state_publisher with robot_description from xacro
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_6"))
                                / "urdf"
                                / "rrbot_modular_actuators.urdf.xacro",
                                " ",
                                "prefix:=",
                                LaunchConfiguration("prefix"),
                                " ",
                                "use_mock_hardware:=",
                                LaunchConfiguration("use_mock_hardware"),
                                " ",
                                "mock_sensor_commands:=",
                                LaunchConfiguration("mock_sensor_commands"),
                                " ",
                                "slowdown:=",
                                LaunchConfiguration("slowdown"),
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("ros2_control_demo_description"))
                    / "rrbot/rviz"
                    / "rrbot.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    LaunchConfiguration("robot_controller"),
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_6"))
                    / "config"
                    / "rrbot_modular_actuators.yaml",
                ],
            ),
        ]
    )
