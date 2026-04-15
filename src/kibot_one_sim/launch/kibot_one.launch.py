# mypy: disable-error-code="import-untyped"

from pathlib import Path

from ament_index_python.packages import get_package_share_directory # type: ignore
from launch import LaunchDescription # type: ignore
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import LaunchConfiguration # type: ignore
from launch_ros.actions import Node # type: ignore

def generate_launch_description() -> LaunchDescription:
    sim_pkg_share = Path(get_package_share_directory("kibot_one_sim"))
    sim_with_bridge_launch = sim_pkg_share / "launch" / "sim_with_bridge.launch.py"
    default_world = sim_pkg_share / "worlds" / "kibot_one.world.sdf"

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=str(default_world),
        description="Gazebo 世界的绝对路径"
    )

    stop_time_period_arg = DeclareLaunchArgument(
        name="stop_time_period",
        default_value="0.5",
        description="cmd_vel watchdog 认为应当发生停止的超时时间，单位秒。"
    )

    watch_time_period_arg = DeclareLaunchArgument(
        name="watch_time_period",
        default_value="0.1",
        description="cmd_vel watchdog 超时检查周期，单位秒。"
    )
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="2",
        description="小车的运行模式"
    )
    mode_pub_timer_period_arg = DeclareLaunchArgument(
        name="mode_pub_timer_period",
        default_value="0.01667",
        description="模式控制器发布 STOP, CRUISE 模式下速度的时间间隔，单位秒"
    )

    start_sim = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(str(sim_with_bridge_launch)),
        launch_arguments={
            "world": LaunchConfiguration("world")
        }.items()
    )

    start_cmd_vel_watchdog = Node(
        package="kibot_one_control",
        executable="cmd_vel_watchdog",
        name="cmd_vel_watchdog",
        output="screen",
        parameters=[{
            "stop_time_period": LaunchConfiguration("stop_time_period"),
            "watch_time_period": LaunchConfiguration("watch_time_period")
        }]
    )
    start_mode_control = Node(
        package="kibot_one_control",
        executable="mode_control",
        name="mode_control",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "mode_pub_timer_period": LaunchConfiguration("mode_pub_timer_period")
        }]
    )

    return LaunchDescription([
        world_arg,
        stop_time_period_arg,
        watch_time_period_arg,
        mode_arg,
        mode_pub_timer_period_arg,
        start_sim,
        start_cmd_vel_watchdog,
        start_mode_control,
    ])