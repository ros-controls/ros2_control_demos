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
                "description_package",
                default_value="ros2_control_demo_description",
                description=(
                    "Description package with robot URDF/xacro files. Usually the argument "
                    "is not set, it enables use of a custom description."
                ),
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value="rrbot_modular_actuators.urdf.xacro",
                description="URDF/XACRO description file with the robot.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description=(
                    "Start Rviz2 and Joint State Publisher gui automatically "
                    "with this launch file."
                ),
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value='""',
                description=(
                    "Prefix of the joint names, useful for "
                    "multi-robot setup. If changed than also joint names in the controllers' configuration "
                    "have to be updated."
                ),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
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
                                / LaunchConfiguration("description_file"),
                                " ",
                                "prefix:=",
                                LaunchConfiguration("prefix"),
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
                    PathSubstitution(FindPackageShare(LaunchConfiguration("description_package")))
                    / "rrbot/rviz"
                    / "rrbot.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )
