# Copyright 2020 ros2_control Development Team
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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="start RViz automatically with the launch file",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffbot_description"), "urdf", "diffbot_system.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    diffbot_diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_bringup"),
            "config",
            "diffbot_diff_drive_controller.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, diffbot_diff_drive_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller"],
        output="screen",
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diffbot_description"), "config", "diffbot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            arg_show_rviz,
            node_robot_state_publisher,
            controller_manager_node,
            spawn_dd_controller,
            spawn_jsb_controller,
            rviz_node,
        ]
    )
