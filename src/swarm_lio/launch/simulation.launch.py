from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def make_rviz_config(template_path: str, prefix: str, bot_id: int) -> str:
    with open(template_path, 'r') as f:
        text = f.read()

    text = text.replace('BOT_PREFIX', prefix)

    # Put a predictable name into /tmp so the RViz title is readable
    cfg_dir = tempfile.gettempdir()
    cfg_path = os.path.join(cfg_dir, f'Bot{bot_id}.rviz')

    with open(cfg_path, 'w') as f:
        f.write(text)

    return cfg_path


def generate_launch_description():
    declare_bot_count = DeclareLaunchArgument(
        "bot_count",
        default_value="4",
        description="Number of bots to simulate (starting at 1). Ignored if bot_list is provided.",
    )

    declare_bot_list = DeclareLaunchArgument(
        "bot_list",
        default_value="",
        description="Comma-separated list of bot IDs to simulate. Overrides bot_count when set.",
    )

    declare_rviz_list = DeclareLaunchArgument(
        "rviz_list",
        default_value="",
        description="Comma-separated list of bot IDs to visualize in RViz (defaults to first bot only)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time from /clock"
    )

    pkg_share = get_package_share_directory('swarm_lio')
    launch_dir = os.path.join(pkg_share, 'launch')
    rviz_cfg_dir = os.path.join(pkg_share, 'rviz_cfg')
    rviz_template = os.path.join(rviz_cfg_dir, 'ros2_generic.rviz')

    def launch_everything(context, *args, **kwargs):
        actions = []
        bot_list_raw = context.launch_configurations.get("bot_list", "").strip()
        bot_count_raw = context.launch_configurations.get("bot_count", "0").strip()

        if bot_list_raw:
            bot_ids = [int(x) for x in bot_list_raw.split(',') if x]
        else:
            try:
                bot_count = int(bot_count_raw) if bot_count_raw else 0
            except ValueError:
                bot_count = 0
            bot_ids = list(range(1, max(bot_count, 0) + 1))

        use_sim_time_str = context.launch_configurations.get('use_sim_time', 'true')
        use_sim_time_bool = str(use_sim_time_str).lower() in ('true', '1', 'yes')

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
                        'output_mode': output_mode,
                        'use_sim_time': use_sim_time_str,
                        'actual_uav_num': str(len(bot_ids)),
                    }.items()
                )
            )
            actions.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'bot{bot_id}_aft_mapped_to_base_link_static_tf',
                    arguments=[ # Identity transform
                        '--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', f'bot{bot_id}/aft_mapped',
                        '--child-frame-id', f'bot{bot_id}/base_link',
                    ],
                    parameters=[{'use_sim_time': use_sim_time_bool}],
                    output='log',
                )
            )


        # --- Decide which bots get RViz ---
        rviz_list_str = context.launch_configurations.get('rviz_list', '').strip()
        if rviz_list_str:
            rviz_ids = [int(x) for x in rviz_list_str.split(',') if x]
        else:
            rviz_ids = [bot_ids[0]] if bot_ids else []

        # --- Publish extrinsics between robots ---
        if bot_ids:
            actions.append(
                Node(
                    package='swarm_lio',
                    executable='robot_extrinsic_publisher',
                    name='robot_extrinsic_publisher',
                    parameters=[{
                        'root_id': int(bot_ids[0]),
                        'robot_ids': bot_ids,
                        'robot_prefix': 'bot',
                        'robot_frame_suffix': 'world',
                        'use_sim_time': use_sim_time_bool,
                    }],
                    output='log',
                )
            )

        # --- Launch one RViz per requested bot ---
        for bot_id in rviz_ids:
            bot_prefix = f'/bot{bot_id}'
            rviz_config = make_rviz_config(rviz_template, bot_prefix, bot_id)

            actions.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name=f'rviz_{bot_id}',
                    arguments=[
                        '-d', rviz_config,
                        '--ros-args',
                        '--log-level', 'error',          # or 'fatal' if you want only crashes
                        '--disable-stdout-logs',         # no logs to terminal
                        # optionally also:
                        # '--disable-rosout-logs',
                        # '--disable-external-lib-logs',
                    ],
                    output='log',
                    emulate_tty=False,
                )

            )

        return actions

    return LaunchDescription([
        declare_bot_count,
        declare_bot_list,
        declare_rviz_list,
        declare_use_sim_time,
        OpaqueFunction(function=launch_everything),
    ])
