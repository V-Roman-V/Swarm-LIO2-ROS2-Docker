from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_lio')
    rviz_file = os.path.join(pkg_share, 'rviz_cfg/mid360.rviz')
    
    params_file = os.path.join(pkg_share, 'config/mid360.yaml')
    params = [
        params_file,
        {'sub_gt_pose_topic': '/vrpn_client_node/mars_multi_01/pose'}
    ]

    return LaunchDescription([
        Node(
            package='swarm_lio',
            executable='swarm_lio',
            name='laserMapping_quad',
            output='screen',
            parameters=params,
            emulate_tty=True,
        ),
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_mid360',
                arguments=['-d', rviz_file],
                prefix=['nice '],
            )
        ])
    ])
