# Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
# Authors: Subhas Das, Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated.",
        )
    )
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
            "slowdown",
            default_value="50.0",
            description="Slowdown factor of the RRbot.",
        )
    )

    # Initialize Arguments
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    slowdown = LaunchConfiguration("slowdown")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/rrbot_base.launch.py"]),
        launch_arguments={
            "controllers_file": "rrbot_with_sensor_controllers.yaml",
            "description_file": "rrbot_system_with_sensor.urdf.xacro",
            "prefix": prefix,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "slowdown": slowdown,
        }.items(),
    )

    # add the spawner node for the fts_broadcaster
    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # collect all the nodes here
    nodes = [
        fts_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + [base_launch] + nodes)
