from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('kibot_one_sim'))
    default_world = pkg_share / 'worlds' / 'kibot_one.world.sdf'
    models_path = pkg_share / 'models'

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = (
        f'{existing_resource_path}:{models_path}' if existing_resource_path else str(models_path)
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=str(default_world),
        description='Gazebo 世界文件的绝对路径。',
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    )

    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', LaunchConfiguration('world')],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gz_resource_path,
        start_gazebo,
    ])
