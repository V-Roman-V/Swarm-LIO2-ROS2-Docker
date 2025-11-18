from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('rviz', default_value='true'))

    params = [
        {'imu_topic': '/livox/imu'},
        {'map_file_path': ' '},
        {'max_iteration': 4},
        {'dense_map_enable': True},
        {'fov_degree': 75.0},
        {'filter_size_corner': 0.2},
        {'filter_size_surf': 0.2},
        {'filter_size_map': 0.5},
        {'cube_side_length': 2000.0},
    ]
    rviz_cfg = os.path.join(
        get_package_share_directory('swarm_lio'),
        'rviz_cfg/ros2.rviz')

    ld.add_action(
        Node(
            package='swarm_lio',
            executable='swarm_lio',
            name='laserMapping',
            output='screen',
            parameters=params,
            prefix=['gdb -ex run --args '],
        )
    )

    ld.add_action(
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_cfg],
                prefix=['rviz2 '],
            )
        ], condition=IfCondition(LaunchConfiguration('rviz')))
    )

    return ld
