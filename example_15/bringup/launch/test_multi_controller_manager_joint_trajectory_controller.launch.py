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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # For example: the parameters for different node may be placed into the same yaml file
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_15"),
            "config",
            "multi_controller_manager_joint_trajectory_publisher.yaml",
        ]
    )

    rrbot_1_publisher_joint_trajectory_controller_node = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        namespace="rrbot_1",
        name="publisher_joint_trajectory_controller",
        parameters=[position_goals],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    rrbot_2_publisher_joint_trajectory_controller_node = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        namespace="rrbot_2",
        name="publisher_joint_trajectory_controller",
        parameters=[position_goals],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    return LaunchDescription(
        [
            rrbot_1_publisher_joint_trajectory_controller_node,
            rrbot_2_publisher_joint_trajectory_controller_node,
        ]
    )
