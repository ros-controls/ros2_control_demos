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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown", default_value="50.0", description="Slowdown factor of the RRbot."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description="Robot controller to start.",
        )
    )

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    slowdown = LaunchConfiguration("slowdown")
    robot_controller = LaunchConfiguration("robot_controller")

    rrbot_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rrbot_base.launch.py"]),
        launch_arguments={
            "namespace": "rrbot_1",
            "controllers_file": "multi_controller_manager_rrbot_1_controllers.yaml",
            "description_file": "rrbot.urdf.xacro",
            "prefix": "rrbot_1_",
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "slowdown": slowdown,
            "controller_manager_name": "/rrbot_1/controller_manager",
            "robot_controller": robot_controller,
            "start_rviz": "false",
        }.items(),
    )

    rrbot_1_position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_trajectory_controller",
            "-c",
            "/rrbot_1/controller_manager",
            "--inactive",
        ],
    )

    rrbot_2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rrbot_base.launch.py"]),
        launch_arguments={
            "namespace": "rrbot_2",
            "controllers_file": "multi_controller_manager_rrbot_2_controllers.yaml",
            "description_file": "rrbot_system_position_only.urdf.xacro",
            "prefix": "rrbot_2_",
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "slowdown": slowdown,
            "controller_manager_name": "/rrbot_2/controller_manager",
            "robot_controller": robot_controller,
            "start_rviz": "false",
        }.items(),
    )

    rrbot_2_position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_trajectory_controller",
            "-c",
            "/rrbot_2/controller_manager",
            "--inactive",
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rrbot_description"), "config", "multi_controller_manager.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    included_launch_files = [
        rrbot_1_launch,
        rrbot_2_launch,
    ]

    nodes_to_start = [
        rrbot_1_position_trajectory_controller_spawner,
        rrbot_2_position_trajectory_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + included_launch_files + nodes_to_start)
