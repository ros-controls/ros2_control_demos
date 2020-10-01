# Copyright 2020 ROS2-Control Development Team
#
# license Apache 2.0
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    ld = LaunchDescription()

# TODO(anyone): This does not work... why?
#    robot_description_file = os.path.join(
#        get_package_share_directory('ros2_control_demo_robot'),
#        'description',
#        'rrbot_system_position_only.urdf.xacro'
#        )
#    descr = xacro.process_file(robot_description_file)

    robot_description_file = os.path.join(
        get_package_share_directory('ros2_control_demo_robot'),
        'description',
        'rrbot_system_position_only.urdf'
        )
    with open(robot_description_file, 'r') as infile:
        descr = infile.read()

    robot_description = {'robot_description': descr}

    rrbot_forward_controller = os.path.join(
        get_package_share_directory('ros2_control_demo_robot'),
        'controllers',
        'rrbot_forward_controller_position.yaml'
        )

    return LaunchDescription([
      Node(
        package='controller_manager',
        name='ros2_control_node',
        executable='ros2_control_node',
        parameters=[robot_description, rrbot_forward_controller],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        #log_level={
          #'logger_name': '',
          #'level': 'DEBUG'
          #}
        )

    ])
