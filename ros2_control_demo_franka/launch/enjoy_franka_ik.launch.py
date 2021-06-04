import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"),
                "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    robot_description_path = os.path.join(
        get_package_share_directory("ros2_control_demo_franka"),
        "description",
        "panda.urdf.xml",
    )
    
    with open(robot_description_path, 'r') as infp:
        robot_description = {"robot_description": infp.read()}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameteres=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "rrbot_system_position"],
        output="screen",
    )

    joint_group_position_controller = os.path.join(
        get_package_share_directory("ros2_control_demo_franka"), "config",
        "ros_control.yaml"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, joint_group_position_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"], # NOTE: joint_state_broadcaster in ROS2 serves the same function as joint_state_controller in ROS1
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            spawn_entity,
            node_robot_state_publisher,
            controller_manager_node,
            spawn_controller,
        ]
    )
