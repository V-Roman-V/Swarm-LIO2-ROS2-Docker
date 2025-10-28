from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='udp_bridge',
            executable='udp_online',
            name='udp_online',
            output='screen',
            parameters=[],
        ),
    ])


