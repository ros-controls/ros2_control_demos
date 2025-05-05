from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'serial:///dev/ttyUSB0:57600',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'system_id': 1,
                'component_id': 1,
            }],
        ),
    ])
