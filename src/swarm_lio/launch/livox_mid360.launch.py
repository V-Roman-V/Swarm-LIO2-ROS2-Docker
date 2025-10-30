from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def flatten_dict(d, parent_key='', sep='/'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_lio')
    rviz_file = os.path.join(pkg_share, 'rviz_cfg/mid360.rviz')

    params_file = os.path.join(pkg_share, 'config/mid360.yaml')
    # Load YAML and flatten dict
    with open(params_file, 'r') as f:
        yaml_data = yaml.safe_load(f)
    params = flatten_dict(yaml_data.get('ros__parameters', {}))
    # Manually add or override any extra params
    params['sub_gt_pose_topic'] = '/vrpn_client_node/mars_multi_01/pose'

    return LaunchDescription([
        Node(
            package='swarm_lio',
            executable='swarm_lio',
            name='laserMapping_quad',
            output='screen',
            parameters=[params],
            emulate_tty=True,
        ),
        GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_mid360',
                arguments=['-d', rviz_file],
                prefix=['rviz2'],
            )
        ])
    ])
