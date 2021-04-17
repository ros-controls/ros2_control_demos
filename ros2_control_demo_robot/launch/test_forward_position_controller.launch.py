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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    position_goals = os.path.join(
        get_package_share_directory('ros2_control_test_nodes'),
        'configs',
        'rrbot_forward_position_publisher.yaml'
        )

    return LaunchDescription([
      Node(
        package='ros2_control_test_nodes',
        executable='publisher_forward_position_controller',
        parameters=[position_goals],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    ])
