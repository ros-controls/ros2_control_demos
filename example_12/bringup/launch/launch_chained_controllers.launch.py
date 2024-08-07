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
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_12"),
            "config",
            "rrbot_chained_controllers.yaml",
        ]
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--param-file", robot_controllers],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )

    # Delay start of forward_position_controller_spawner after `position_controller_spawner`
    delay_forward_position_controller_spawner_after_position_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=position_controller_spawner,
                on_exit=[forward_position_controller_spawner],
            )
        )
    )

    nodes = [
        position_controller_spawner,
        delay_forward_position_controller_spawner_after_position_controller_spawner,
    ]

    return LaunchDescription(nodes)
