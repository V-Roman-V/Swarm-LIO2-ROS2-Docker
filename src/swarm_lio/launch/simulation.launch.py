from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    num_drones = LaunchConfiguration('num_drones')

    pkg_share = get_package_share_directory('swarm_lio')
    launch_dir = os.path.join(pkg_share, 'launch')
    rviz_cfg_dir = os.path.join(pkg_share, 'rviz_cfg')

    ld = LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='5')
    ])

    # Use OpaqueFunction to calculate at launch time
    from launch.actions import OpaqueFunction
    def launch_drones(context, *args, **kwargs):
        actions = []
        n = int(context.launch_configurations['num_drones'])
        for i in range(n):
            output_mode = 'screen' if i == 0 else 'log'
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'single_drone_sim.launch.py')
                    ),
                    launch_arguments={
                        'drone_id': str(i+1),
                        'output_mode': output_mode
                    }.items()
                )
            )
        return actions
    ld.add_action(OpaqueFunction(function=launch_drones))

    rviz_setups = [
        {'name': 'rviz_1', 'config': 'sim01.rviz'},
        {'name': 'rviz_2', 'config': 'sim02.rviz'},
    ]
    for rv in rviz_setups:
        rviz_file = os.path.join(rviz_cfg_dir, rv['config'])
        ld.add_action(GroupAction([
            Node(
                package='rviz2',
                executable='rviz2',
                name=rv['name'],
                arguments=['-d', rviz_file],
                prefix=['nice '],
            )
        ]))

    return ld
