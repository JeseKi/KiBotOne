from launch import LaunchDescription  # type: ignore
from launch.actions import ExecuteProcess  # type: ignore


def generate_launch_description() -> LaunchDescription:
    console_command = (
        'if [ -t 0 ]; then '
        'exec ros2 run kibot_one_control control_console; '
        'elif [ -e /proc/$PPID/fd/0 ]; then '
        'exec ros2 run kibot_one_control control_console < /proc/$PPID/fd/0; '
        'else '
        'echo "control_console 需要交互式终端，请改用 ros2 run kibot_one_control control_console"; '
        'exit 1; '
        'fi'
    )
    control_console = ExecuteProcess(
        cmd=['bash', '-lc', console_command],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        control_console,
    ])
