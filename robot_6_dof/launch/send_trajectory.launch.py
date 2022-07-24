
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_6_dof"),
                    "urdf",
                    "robot_6_dof.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    send_trajectory_node = Node(
        package="robot_6_dof",
        executable="send_trajectory",
        name="send_trajectory_node",
        parameters=[robot_description]
    )

    nodes_to_start = [send_trajectory_node]
    return LaunchDescription(nodes_to_start)
