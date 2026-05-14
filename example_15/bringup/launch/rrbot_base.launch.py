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
                "namespace",
                default_value="/",
                description="Namespace of controller manager and controllers. This is useful for multi-robot scenarios.",
            ),
            DeclareLaunchArgument(
                "runtime_config_package",
                default_value="ros2_control_demo_bringup",
                description='Package with the controller\'s configuration in "config" folder. Usually the argument is not set, it enables use of a custom setup.',
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="rrbot_controllers.yaml",
                description="YAML file with the controllers configuration.",
            ),
            DeclareLaunchArgument(
                "description_package",
                default_value="ros2_control_demo_description",
                description="Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.",
            ),
            DeclareLaunchArgument(
                "description_file",
                description="URDF/XACRO description file with the robot.",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value='""',
                description="Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated.",
            ),
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="false",
                description="Start robot in Gazebo simulation.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            DeclareLaunchArgument(
                "mock_sensor_commands",
                default_value="false",
                description="Enable mock command interfaces for sensors used for simple simulations. Used only if 'use_mock_hardware' parameter is true.",
            ),
            DeclareLaunchArgument(
                "slowdown", default_value="3.0", description="Slowdown factor of the RRbot."
            ),
            DeclareLaunchArgument(
                "controller_manager_name",
                default_value="/controller_manager",
                description="Full name of the controller manager. This values should be set if controller manager is used under a namespace.",
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="forward_position_controller",
                description="Robot controller to start.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            # robot description from xacro
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(
                                    FindPackageShare(LaunchConfiguration("description_package"))
                                )
                                / "urdf"
                                / LaunchConfiguration("description_file"),
                                " ",
                                "prefix:=",
                                LaunchConfiguration("prefix"),
                                " ",
                                "use_gazebo:=",
                                LaunchConfiguration("use_gazebo"),
                                " ",
                                "use_mock_hardware:=",
                                LaunchConfiguration("use_mock_hardware"),
                                " ",
                                "mock_sensor_commands:=",
                                LaunchConfiguration("mock_sensor_commands"),
                                " ",
                                "slowdown:=",
                                LaunchConfiguration("slowdown"),
                            ]
                        )
                    },
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("runtime_config_package"))
                    )
                    / "config"
                    / LaunchConfiguration("controllers_file"),
                ],
                output="both",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=LaunchConfiguration("namespace"),
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(
                                    FindPackageShare(LaunchConfiguration("description_package"))
                                )
                                / "urdf"
                                / LaunchConfiguration("description_file"),
                                " ",
                                "prefix:=",
                                LaunchConfiguration("prefix"),
                                " ",
                                "use_gazebo:=",
                                LaunchConfiguration("use_gazebo"),
                                " ",
                                "use_mock_hardware:=",
                                LaunchConfiguration("use_mock_hardware"),
                                " ",
                                "mock_sensor_commands:=",
                                LaunchConfiguration("mock_sensor_commands"),
                                " ",
                                "slowdown:=",
                                LaunchConfiguration("slowdown"),
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=LaunchConfiguration("namespace"),
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare(LaunchConfiguration("description_package")))
                    / "config"
                    / "rrbot.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=LaunchConfiguration("namespace"),
                arguments=[
                    "joint_state_broadcaster",
                    "-c",
                    LaunchConfiguration("controller_manager_name"),
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=LaunchConfiguration("namespace"),
                arguments=[
                    LaunchConfiguration("robot_controller"),
                    "-c",
                    LaunchConfiguration("controller_manager_name"),
                    "--param-file",
                    PathSubstitution(
                        FindPackageShare(LaunchConfiguration("runtime_config_package"))
                    )
                    / "config"
                    / LaunchConfiguration("controllers_file"),
                ],
            ),
        ]
    )
