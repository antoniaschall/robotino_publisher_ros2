from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotino_publisher',
            executable='robotino_publisher',
            name='robotino_publisher',
            output='screen',
            parameters=[
                {'ip_address': '127.0.0.1'}
            ]
        )
    ])
