# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start robot with simulation from Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown", default_value="3.0", description="Slowdown factor of the RRbot."
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
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    slowdown = LaunchConfiguration("slowdown")
    robot_controller = LaunchConfiguration("robot_controller")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rrbot_system_position"],
        output="screen",
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros2_control_demo_bringup"),
                        "launch",
                        "rrbot.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "description_file": "rrbot_system_position_only.urdf.xacro",
            "prefix": prefix,
            "use_sim": use_sim,
            "slowdown": slowdown,
            "robot_controller": robot_controller,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [gazebo, base_launch, spawn_entity])
