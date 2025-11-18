from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
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

def launch_setup(context, *args, **kwargs):
    # Resolve launch arguments in the current context
    drone_id = LaunchConfiguration('drone_id').perform(context)
    output_mode = LaunchConfiguration('output_mode').perform(context)

    pkg_share = get_package_share_directory('swarm_lio')

    # Load YAML and flatten dict
    params_file = os.path.join(pkg_share, 'config', 'simulation.yaml')
    with open(params_file, 'r') as f:
        yaml_data = yaml.safe_load(f)
    params = flatten_dict(yaml_data.get('ros__parameters', {}))

    # override drone specific params using drone_id
    params['common/drone_id'] = int(drone_id)
    params['common/lid_topic'] = f'bot{drone_id}/lidar_points'
    params['common/imu_topic'] = f'bot{drone_id}/imu'
    params['sub_gt_pose_topic'] = f"/quad_{drone_id}/lidar_slam/odom"

    print(
        f"Launching drone {drone_id} with LIDAR topic {params['common/lid_topic']} "
        f"and IMU topic {params['common/imu_topic']}"
    )

    node = Node(
        package='swarm_lio',
        executable='swarm_lio',
        name=f'laserMapping_bot{drone_id}',
        output=output_mode,
        parameters=[params],
        emulate_tty=True,
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('output_mode', default_value='screen'),
        OpaqueFunction(function=launch_setup),
    ])
