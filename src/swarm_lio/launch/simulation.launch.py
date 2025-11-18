from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def make_rviz_config(template_path: str, prefix: str) -> str:
    with open(template_path, 'r') as f:
        text = f.read()

    text = text.replace('BOT_PREFIX', prefix)

    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.rviz')
    tmp.write(text)
    tmp.close()
    return tmp.name


def generate_launch_description():
    declare_drone_list = DeclareLaunchArgument('drone_list', default_value='1,2,3')

    pkg_share = get_package_share_directory('swarm_lio')
    launch_dir = os.path.join(pkg_share, 'launch')
    rviz_cfg_dir = os.path.join(pkg_share, 'rviz_cfg')
    rviz_template = os.path.join(rviz_cfg_dir, 'ros2_generic.rviz')

    def launch_everything(context, *args, **kwargs):
        actions = []
        drone_ids = [int(x) for x in context.launch_configurations['drone_list'].split(',')]

        # 1) Launch drones
        for drone_id in drone_ids:
            output_mode = 'screen' if drone_id == drone_ids[0] else 'log'
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'single_drone_sim.launch.py')
                    ),
                    launch_arguments={
                        'drone_id': str(drone_id),
                        'output_mode': output_mode
                    }.items()
                )
            )

        # 2) Launch RViz for first drone
        first_id = drone_ids[0]
        bot_prefix = f"/quad{first_id}"

        rviz_config = make_rviz_config(rviz_template, bot_prefix)

        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name=f'rviz_{first_id}',
                arguments=['-d', rviz_config],
                output='screen',
            )
        )

        return actions

    return LaunchDescription([
        declare_drone_list,
        OpaqueFunction(function=launch_everything),
    ])
