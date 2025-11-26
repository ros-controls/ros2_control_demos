# Copyright 2025 ros2_control Development Team
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
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "fixed_frame_id",
                default_value="odom",
                description="Fixed frame id of the robot.",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_16"))
                    / "config"
                    / "diffbot_chained_controllers.yaml"
                ],
                output="both",
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
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_16"))
                                / "urdf"
                                / "diffbot.urdf.xacro",
                                " ",
                                "use_mock_hardware:=",
                                LaunchConfiguration("use_mock_hardware"),
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
                    / "diffbot/rviz"
                    / "diffbot.rviz",
                    "-f",
                    LaunchConfiguration("fixed_frame_id"),
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
                    "pid_controller_left_wheel_joint",
                    "pid_controller_right_wheel_joint",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_16"))
                    / "config"
                    / "diffbot_chained_controllers.yaml",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diffbot_base_controller",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_16"))
                    / "config"
                    / "diffbot_chained_controllers.yaml",
                    "--controller-ros-args",
                    "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
                ],
            ),
        ]
    )
