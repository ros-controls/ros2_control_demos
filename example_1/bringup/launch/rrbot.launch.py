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
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            DeclareLaunchArgument(
                "joint_prefix",
                default_value="joint",
                description="Prefix for joint names (used with allow_substs in spawner).",
            ),
            # Control node
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_1"))
                    / "config"
                    / "controller_manager.yaml"
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
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_1"))
                                / "urdf"
                                / "rrbot.urdf.xacro",
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
                    / "rrbot/rviz/rrbot.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_1"))
                    / "config"
                    / "rrbot_controllers.yaml",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                parameters=[
                    {"joint_prefix": LaunchConfiguration("joint_prefix")},
                    ParameterFile(
                        PathSubstitution(FindPackageShare("ros2_control_demo_example_1"))
                        / "config"
                        / "rrbot_controllers.yaml",
                        allow_substs=True,
                    ),
                ],
                arguments=[
                    "forward_position_controller",
                ],
            ),
        ]
    )
