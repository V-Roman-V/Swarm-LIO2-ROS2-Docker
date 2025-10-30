from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id')
    output_mode = LaunchConfiguration('output_mode')

    pkg_share = get_package_share_directory('swarm_lio')
    params_file = os.path.join(pkg_share, 'config/simulation.yaml')
    params = [
        params_file,
        {'drone_id': drone_id},
        {'common/lid_topic': f"/quad{drone_id}_pcl_render_node/sensor_cloud"},
        {'common/imu_topic': f"/quad_{drone_id}/imu"},
        {'sub_gt_pose_topic': f"/quad_{drone_id}/lidar_slam/odom"}
    ]

    return LaunchDescription([
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('output_mode'),
        Node(
            package='swarm_lio',
            executable='swarm_lio',
            name=f"laserMapping_quad{drone_id}",
            output=output_mode,
            parameters=params,
            emulate_tty=True,
        ),
    ])
