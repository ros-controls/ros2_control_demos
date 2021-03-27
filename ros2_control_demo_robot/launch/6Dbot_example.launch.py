# Copyright 2021 Stogl Denis Stogl (Stogl Robotics Consulting)
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

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'use_fake_hardware', default_value='true',
        description='Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(DeclareLaunchArgument(
        'fake_sensor_commands', default_value='true',
        description='Enable fake command interfaces for sensors used for simple simulations. \
            Used only if \'use_fake_hardware\' parameter is true.'))

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([get_package_prefix('xacro'), 'bin', 'xacro']),
        ' ',
        PathJoinSubstitution(
            [get_package_share_directory(
                'ros2_control_demo_robot'), 'description', '6Dbot_example.urdf.xacro']),
        ' ',
        'use_fake_hardware:=', use_fake_hardware, ' ',
        'fake_sensor_commands:=', fake_sensor_commands
        ])

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = os.path.join(
        get_package_share_directory('ros2_control_demo_robot'),
        'config',
        '6Dbot_controllers.yaml'
        )
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_control_demo_robot'),
        'rviz',
        'rrbot.rviz'
        )

    control_node = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[robot_description, robot_controllers],
      output={
          'stdout': 'screen',
          'stderr': 'screen',
        },
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # start controller until PR #310 in ros2-controls/ros2_contol is merged
    load_start_joint_state_controller = ExecuteProcess(
      cmd=['ros2 control load_start_controller joint_state_controller'],
      shell=True, output='screen')
    controller = 'forward_position_controller'
    load_start_robot_controller = ExecuteProcess(
      cmd=['ros2 control load_start_controller',  controller], shell=True, output='screen')

    return LaunchDescription(
        declared_arguments +
        [
            control_node,
            robot_state_publisher_node,
            rviz_node,
            load_start_joint_state_controller,
            load_start_robot_controller,
        ])
