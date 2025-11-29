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
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mocked command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown", default_value="50.0", description="Slowdown factor of the RRbot."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_wrench_transformer",
            default_value="false",
            description="Enable the wrench transformer node to transform wrench messages to different frames.",
        )
    )

    # Initialize Arguments
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    slowdown = LaunchConfiguration("slowdown")
    gui = LaunchConfiguration("gui")
    use_wrench_transformer = LaunchConfiguration("use_wrench_transformer")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            "xacro",
            " ",
            PathSubstitution(FindPackageShare("ros2_control_demo_example_5"))
            / "urdf"
            / "rrbot_system_with_external_sensor.urdf.xacro",
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "slowdown:=",
            slowdown,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = (
        PathSubstitution(FindPackageShare("ros2_control_demo_example_5"))
        / "config"
        / "rrbot_with_external_sensor_controllers.yaml"
    )
    wrench_transformer_params = (
        PathSubstitution(FindPackageShare("ros2_control_demo_example_5"))
        / "config"
        / "wrench_transformer_params.yaml"
    )
    rviz_config_file = (
        PathSubstitution(FindPackageShare("ros2_control_demo_example_5")) / "rviz" / "rrbot.rviz"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers],
    )

    # add the spawner node for the fts_broadcaster
    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fts_broadcaster", "--param-file", robot_controllers],
    )

    # add the wrench transformer node (optional)
    wrench_transformer_node = Node(
        package="force_torque_sensor_broadcaster",
        executable="wrench_transformer_node",
        name="fts_wrench_transformer",
        parameters=[wrench_transformer_params],
        remappings=[("~/wrench", "/fts_broadcaster/wrench")],
        output="both",
        condition=IfCondition(use_wrench_transformer),
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            rviz_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            fts_broadcaster_spawner,
            wrench_transformer_node,
        ]
    )
