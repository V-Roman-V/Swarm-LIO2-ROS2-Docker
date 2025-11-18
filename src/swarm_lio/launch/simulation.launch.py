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
    declare_bot_list = DeclareLaunchArgument(
        "bot_list",
        default_value="1,2,3",
        description="Comma-separated list of bot IDs to simulate",
    )

    declare_rviz_list = DeclareLaunchArgument(
        "rviz_list",
        default_value="",
        description="Comma-separated list of bot IDs to visualize in RViz (defaults to first bot only)",
    )

    pkg_share = get_package_share_directory('swarm_lio')
    launch_dir = os.path.join(pkg_share, 'launch')
    rviz_cfg_dir = os.path.join(pkg_share, 'rviz_cfg')
    rviz_template = os.path.join(rviz_cfg_dir, 'ros2_generic.rviz')

    def launch_everything(context, *args, **kwargs):
        actions = []
        bot_ids = [int(x) for x in context.launch_configurations["bot_list"].split(',')]

        # --- Launch sim for each bot ---
        for bot_id in bot_ids:
            output_mode = 'screen' if bot_id == bot_ids[0] else 'log'
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'single_drone_sim.launch.py')
                    ),
                    launch_arguments={
                        'drone_id': str(bot_id),
                        'output_mode': output_mode
                    }.items()
                )
            )

        # --- Decide which bots get RViz ---
        rviz_list_str = context.launch_configurations.get('rviz_list', '').strip()
        if rviz_list_str:
            rviz_ids = [int(x) for x in rviz_list_str.split(',') if x]
        else:
            rviz_ids = [bot_ids[0]] if bot_ids else []

        # --- Launch one RViz per requested bot ---
        for bot_id in rviz_ids:
            bot_prefix = f'/quad{bot_id}'
            rviz_config = make_rviz_config(rviz_template, bot_prefix)

            actions.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name=f'rviz_{bot_id}',
                    arguments=['-d', rviz_config],
                    output='screen',
                )
            )

        return actions

    return LaunchDescription([
        declare_bot_list,
        declare_rviz_list,
        OpaqueFunction(function=launch_everything),
    ])
