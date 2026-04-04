# mypy: disable-error-code="import-untyped"

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory('kibot_one_sim'))
    default_world = pkg_share / 'worlds' / 'kibot_one.world.sdf'
    gazebo_launch = pkg_share / 'launch' / 'gazebo.launch.py'
    bridge_config = pkg_share / 'config' / 'ros_gz_bridge.yaml'

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=str(default_world),
        description='Gazebo 世界文件的绝对路径。'
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )

    start_bridge = RosGzBridge(
        bridge_name='kibot_one_bridge',
        config_file=str(bridge_config)
    )

    return LaunchDescription([
        world_arg,
        start_gazebo,
        start_bridge,
    ])
