from pathlib import Path

from ament_index_python.packages import get_package_share_directory # type: ignore
from launch import LaunchDescription # type: ignore
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import LaunchConfiguration # type: ignore


def generate_launch_description() -> LaunchDescription:
    sim_share = Path(get_package_share_directory('kibot_one_sim'))
    bringup_launch = sim_share / 'launch' / 'kibot_one.launch.py'

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=str(sim_share / 'worlds' / 'kibot_one_obstacles.world.sdf'),
        description='Gazebo 世界文件的绝对路径。',
    )
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='3',
        description='小车的运行模式，默认 FOLLOW。',
    )

    start_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_launch)),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'mode': LaunchConfiguration('mode'),
            'start_follow_controller': 'true',
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        mode_arg,
        start_bringup,
    ])
