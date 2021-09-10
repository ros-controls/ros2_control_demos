# Copyright (c) 2021 PickNik, Inc.
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
# Author: Lovro Ivanov
#
# Description: After a robot has been loaded, this will execute a series of trajectories.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    velocity_goals = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_bringup"), "config", "rrbot_forward_velocity_publisher.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_control_test_nodes",
                executable="publisher_forward_position_controller",
                name="publisher_forward_velocity_controller",
                parameters=[velocity_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )
