from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_share = Path(get_package_share_directory('kibot_one_sim'))
    sim_launch = sim_share / 'launch' / 'sim_with_bridge.launch.py'

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=str(sim_share / 'worlds' / 'kibot_one.world.sdf'),
        description='Gazebo 世界文件的绝对路径。',
    )

    start_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    mode_control = Node(
        package='kibot_one_control',
        executable='mode_control',
        parameters=[{'mode': 3}],
        output='screen',
    )
    watchdog = Node(
        package='kibot_one_control',
        executable='cmd_vel_watchdog',
        output='screen',
    )
    flag_pose_publisher = Node(
        package='kibot_one_control',
        executable='flag_pose_publisher',
        output='screen',
    )
    follow_controller = Node(
        package='kibot_one_control',
        executable='follow_controller',
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        start_sim,
        mode_control,
        watchdog,
        flag_pose_publisher,
        follow_controller,
    ])
