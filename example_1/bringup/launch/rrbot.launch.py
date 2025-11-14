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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define Node action objects needed to be referenced by RegisterEventHandler
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--param-file",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_1"),
                    "config",
                    "rrbot_controllers.yaml",
                ]
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            # Control node
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros2_control_demo_example_1"),
                            "config",
                            "rrbot_controllers.yaml",
                        ]
                    )
                ],
                output="both",
            ),
            # robot_state_publisher with robot_description from xacro
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("ros2_control_demo_example_1"),
                                        "urdf",
                                        "rrbot.urdf.xacro",
                                    ]
                                ),
                            ]
                        )
                    }
                ],
            ),
            # Include the robot_controller_spawner action (starts at launch)
            robot_controller_spawner,
            # When robot_controller_spawner exits, start the joint_state_broadcaster
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawner,
                    on_exit=[
                        joint_state_broadcaster_spawner := Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=["joint_state_broadcaster"],
                        )
                    ],
                )
            ),
            # When joint_state_broadcaster_spawner exits, start RViz
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        Node(
                            package="rviz2",
                            executable="rviz2",
                            name="rviz2",
                            output="log",
                            arguments=[
                                "-d",
                                PathSubstitution(FindPackageShare("ros2_control_demo_description")) / "rrbot" / "rviz" / "rrbot.rviz",
                            ],
                            condition=IfCondition(LaunchConfiguration("gui")),
                        )
                    ],
                )
            ),
        ]
    )
