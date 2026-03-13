# Copyright 2025 Jasper van Brakel
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
# Based on ros-controls/ros2_control_demos/example16/bringup/launch/diffbot.launch.py

# Launch-file for ros2_control_demos Example 2 and 16
#   Provide optional PID controllers

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare
from launch_ros2_control.actions import SpawnControllers
from launch_ros2_control.descriptions import Controller


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "fixed_frame_id", default_value="odom", description="Fixed frame id of the robot."
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_pid", description="If true the PID controller for the wheels gets configured."
        )
    )

    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    fixed_frame_id = LaunchConfiguration("fixed_frame_id")
    use_pid = LaunchConfiguration("use_pid")

    if_use_pid = IfCondition(use_pid)

    ld.add_action(
        SetLaunchConfiguration(
            "robot_description_content",
            Command(
                [
                    "xacro ",
                    FindPackageShare("ros2_control_demo_example_16")
                    / "urdf"
                    / "diffbot.urdf.xacro",
                    " use_mock_hardware:=",
                    use_mock_hardware,
                ]
            ),
        )
    )

    robot_description_content = LaunchConfiguration("robot_description_content")

    diffbot_controller_parameters = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_example_2"), "config", "diffbot_controllers.yaml"]
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[diffbot_controller_parameters],
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description_content}],
        )
    )

    pid_controller_config_file = (
        FindPackageShare("ros2_control_demo_example_19")
        / "config"
        / "diffbot_pid_controllers.yaml"
    )

    # Use global parameters to ensure parameter overriding happens in the correct order
    ld.add_action(SetParametersFromFile(diffbot_controller_parameters))

    ld.add_action(
        SpawnControllers(
            [
                Controller(
                    name="pid_controller_left_wheel_joint",
                    # Load the config for the PID controllers,
                    #  and small changes for diffbot_base_controller
                    parameters=[pid_controller_config_file],
                    condition=if_use_pid,
                ),
                Controller(
                    name="pid_controller_right_wheel_joint",
                    # This file can technically be omitted, since it is loaded by the previous controller
                    parameters=[pid_controller_config_file],
                    condition=if_use_pid,
                ),
                Controller(
                    name="diffbot_base_controller",
                    # Need to load controller type, since it is required if providing
                    # controller configs at spawn time
                    parameters=[
                        {
                            "type": "diff_drive_controller/DiffDriveController",
                            "publish_limited_velocity": True,
                        }
                    ],
                    remappings=[("~/cmd_vel", "/cmd_vel")],
                ),
                Controller("joint_state_broadcaster"),
            ]
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_description"),
                        "diffbot",
                        "rviz",
                        "diffbot.rviz",
                    ]
                ),
                "-f",
                fixed_frame_id,
            ],
            condition=IfCondition(gui),
        )
    )

    return ld
