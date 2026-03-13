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
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fixed_frame_id",
            default_value="odom",
            description="Fixed frame id of the robot.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    fixed_frame_id = LaunchConfiguration("fixed_frame_id")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_demo_example_16"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_description"), "diffbot/rviz", "diffbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_16"),
                    "config",
                    "diffbot_cm.yaml",
                ]
            )
        ],
        output="both",
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
        arguments=["-d", rviz_config_file, "-f", fixed_frame_id],
        condition=IfCondition(gui),
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller",
            "joint_state_broadcaster",
            "--param-file",
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_example_16"),
                        "config",
                        "diffbot_cm.yaml",
                    ]
                )
            ],
            "--controller",
            "pid_controller_right_wheel_joint",
            "--param-file",
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_example_16"),
                        "config",
                        "pid_controllers.yaml",
                    ]
                )
            ],
            "--controller",
            "pid_controller_left_wheel_joint",
            "--param-file",
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_example_16"),
                        "config",
                        "pid_controllers.yaml",
                    ]
                )
            ],
            "--controller",
            "diffbot_base_controller",
            "--param-file",
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_example_16"),
                        "config",
                        "diff_drive_controller.yaml",
                    ]
                )
            ],
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
