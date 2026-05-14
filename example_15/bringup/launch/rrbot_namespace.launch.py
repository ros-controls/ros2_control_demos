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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="/rrbot",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_1"))
                                / "urdf"
                                / "rrbot.urdf.xacro",
                            ]
                        )
                    },
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_15"))
                    / "config"
                    / "rrbot_namespace_controllers.yaml",
                ],
                output={"stdout": "screen", "stderr": "screen"},
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="/rrbot",
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(FindPackageShare("ros2_control_demo_example_1"))
                                / "urdf"
                                / "rrbot.urdf.xacro",
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_15"))
                    / "rviz"
                    / "rrbot_namespace.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="rrbot",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="rrbot",
                arguments=[
                    "forward_position_controller",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_15"))
                    / "config"
                    / "rrbot_namespace_controllers.yaml",
                    "--controller-ros-args",
                    # we use the remapping from a relative name to FQN /position_commands
                    "-r forward_position_controller/commands:=/position_commands",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="rrbot",
                arguments=[
                    "position_trajectory_controller",
                    "--inactive",
                    "--param-file",
                    PathSubstitution(FindPackageShare("ros2_control_demo_example_15"))
                    / "config"
                    / "rrbot_namespace_controllers.yaml",
                ],
            ),
        ]
    )
<<<<<<< HEAD
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_15"),
            "config",
            "rrbot_namespace_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_example_15"), "rviz", "rrbot_namespace.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="/rrbot",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                # we use the remapping from a relative name to FQN /position_commands
                "forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/rrbot",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/rrbot/controller_manager"],
    )

    robot_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/rrbot/controller_manager"],
    )

    robot_position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_trajectory_controller",
            "-c",
            "/rrbot/controller_manager",
            "--inactive",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        robot_forward_position_controller_spawner,
        robot_position_trajectory_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
=======
>>>>>>> 1d4a17a ([Fix] Cleanup Launch Files (#982))
