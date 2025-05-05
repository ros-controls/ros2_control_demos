from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_control_demo_example_2',
            executable='mavlink_subscriber_node',
            name='mavlink_subscriber_node',
            output='screen'
        )
    ])
