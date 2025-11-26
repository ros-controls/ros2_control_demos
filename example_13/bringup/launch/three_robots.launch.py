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
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from controller_manager.launch_utils import generate_controllers_spawner_launch_description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slowdown",
                default_value="50.0",
                description="Slowdown factor of the RRbot.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            # controller manager node
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_controllers.yaml",
                ],
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
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                                / "urdf"
                                / "three_robots.urdf.xacro",
                                " slowdown:=",
                                LaunchConfiguration("slowdown"),
                            ]
                        )
                    }
                ],
            ),
            # RViz (conditional)
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "rviz"
                    / "three_robots.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            # spawners: use helper that generates spawner launch descriptions
            generate_controllers_spawner_launch_description(
                [
                    "joint_state_broadcaster",
                    "rrbot_joint_state_broadcaster",
                    "rrbot_position_controller",
                    "rrbot_external_fts_broadcaster",
                ],
                controller_params_files=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_controllers.yaml",
                ],
            ),
            generate_controllers_spawner_launch_description(
                ["rrbot_with_sensor_joint_state_broadcaster", "rrbot_with_sensor_fts_broadcaster"],
                controller_params_files=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_controllers.yaml",
                ],
            ),
            generate_controllers_spawner_launch_description(
                ["rrbot_with_sensor_position_controller"],
                controller_params_files=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_controllers.yaml",
                ],
                extra_spawner_args=["--inactive"],
            ),
            generate_controllers_spawner_launch_description(
                [
                    "threedofbot_joint_state_broadcaster",
                    "threedofbot_position_controller",
                    "threedofbot_pid_gain_controller",
                ],
                controller_params_files=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_controllers.yaml",
                ],
                extra_spawner_args=["--inactive"],
            ),
            # Command publishers
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="rrbot_position_command_publisher",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_position_command_publishers.yaml",
                ],
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="rrbot_with_sensor_position_command_publisher",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_position_command_publishers.yaml",
                ],
            ),
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="threedofbot_position_command_publisher",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_13"))
                    / "config"
                    / "three_robots_position_command_publishers.yaml",
                ],
            ),
        ]
    )
