import select
import sys
import termios
import tty
from enum import IntEnum
from typing import cast

import rclpy
from geometry_msgs.msg import Twist  # type: ignore
from kibot_one_interface.msg import ModeState  # type: ignore
from kibot_one_interface.srv import Mode as ModeSrv  # type: ignore
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class Mode(IntEnum):
    STOP = 0
    CRUISE = 1
    MANUAL = 2
    FOLLOW = 3


KEY_UP = 'up'
KEY_DOWN = 'down'
KEY_LEFT = 'left'
KEY_RIGHT = 'right'
KEY_ENTER = 'enter'
KEY_SPACE = 'space'
KEY_Q = 'q'


class ControlConsole(Node):
    def __init__(self) -> None:
        super().__init__(node_name='control_console')

        self.declare_parameter('cmd_vel_raw_topic', 'cmd_vel_raw')
        self.declare_parameter('mode_topic', 'mode')
        self.declare_parameter('mode_service', 'mode_control')
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('poll_timeout', 0.1)
        self.declare_parameter('service_timeout', 3.0)

        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.poll_timeout = self.get_parameter('poll_timeout').get_parameter_value().double_value
        self.service_timeout = self.get_parameter('service_timeout').get_parameter_value().double_value
        self.current_mode: Mode | None = None
        self.status_message = '准备就绪'
        self.menu_items = [
            (Mode.STOP, 'STOP 停止'),
            (Mode.CRUISE, 'CRUISE 巡航'),
            (Mode.MANUAL, 'MANUAL 手操'),
            (Mode.FOLLOW, 'FOLLOW 跟随'),
        ]
        self.selected_index = 0

        mode_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_client = self.create_client(
            srv_type=ModeSrv,
            srv_name=self.get_parameter('mode_service').get_parameter_value().string_value,
        )
        self.cmd_vel_raw_publisher = self.create_publisher(
            msg_type=Twist,
            topic=self.get_parameter('cmd_vel_raw_topic').get_parameter_value().string_value,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=ModeState,
            topic=self.get_parameter('mode_topic').get_parameter_value().string_value,
            callback=self._mode_callback,
            qos_profile=mode_qos,
        )

        if not sys.stdin.isatty():
            raise RuntimeError('control_console 需要交互式终端，请使用 ros2 run 或 control_console.launch.py 启动。')

        self._settings = termios.tcgetattr(sys.stdin)

    def run(self) -> None:
        try:
            if not self._wait_for_mode_service():
                return

            while rclpy.ok():
                self._draw_menu()
                key = self._read_key()
                rclpy.spin_once(self, timeout_sec=0.0)

                if key is None:
                    continue
                if key == KEY_Q:
                    self.publish_stop()
                    break
                if key in ('w', KEY_UP):
                    self.selected_index = (self.selected_index - 1) % len(self.menu_items)
                    continue
                if key in ('s', KEY_DOWN):
                    self.selected_index = (self.selected_index + 1) % len(self.menu_items)
                    continue
                if key == KEY_ENTER:
                    self._activate_selected_mode()
        finally:
            self.publish_stop()
            self._restore_terminal()

    def _mode_callback(self, msg: ModeState) -> None:
        try:
            self.current_mode = Mode(msg.current_mode)
        except ValueError:
            self.current_mode = None

    def _wait_for_mode_service(self) -> bool:
        self._clear_screen()
        print('KiBotOne 控制台')
        print('等待 /mode_control 服务...')
        if self.mode_client.wait_for_service(timeout_sec=self.service_timeout):
            return True

        self.status_message = '未找到 mode_control 服务，请先启动 bringup。'
        self._draw_menu()
        return False

    def _activate_selected_mode(self) -> None:
        mode = self.menu_items[self.selected_index][0]
        if mode == Mode.CRUISE:
            self._start_cruise_mode()
            return
        if mode == Mode.MANUAL:
            if self._request_mode(Mode.MANUAL):
                self._run_manual_mode()
            return

        self._request_mode(mode)

    def _start_cruise_mode(self) -> None:
        self._restore_terminal()
        self._clear_screen()
        print('CRUISE 巡航模式')
        print('输入速度后按回车；直接回车使用 0.0。')
        try:
            linear_x = self._read_float('线速度 linear.x (m/s): ', default=0.0)
            angular_z = self._read_float('角速度 angular.z (rad/s): ', default=0.0)
        except (KeyboardInterrupt, EOFError):
            self.status_message = '已取消巡航设置。'
            return

        cruise_velocity = Twist()
        cruise_velocity.linear.x = linear_x
        cruise_velocity.angular.z = angular_z
        self._request_mode(Mode.CRUISE, cruise_velocity)

    def _run_manual_mode(self) -> None:
        self.publish_stop()
        self.status_message = '已进入 MANUAL 手操；按 q 返回菜单。'

        while rclpy.ok():
            self._draw_manual_screen()
            key = self._read_key()
            rclpy.spin_once(self, timeout_sec=0.0)

            if key is None:
                continue
            if key == KEY_Q:
                self.publish_stop()
                self.status_message = '已退出手操，回到控制台菜单。'
                return
            if key in (KEY_ENTER, KEY_SPACE, 'x'):
                self.publish_stop()
                self.status_message = '已发送停止命令。'
                continue

            command = self._manual_key_to_twist(key)
            if command is not None:
                self.cmd_vel_raw_publisher.publish(command)

    def _manual_key_to_twist(self, key: str) -> Twist | None:
        command = Twist()

        if key in ('w', KEY_UP):
            command.linear.x = self.linear_speed
            self.status_message = '手操：前进'
        elif key in ('s', KEY_DOWN):
            command.linear.x = -self.linear_speed
            self.status_message = '手操：后退'
        elif key in ('a', KEY_LEFT):
            command.angular.z = self.angular_speed
            self.status_message = '手操：原地左转'
        elif key in ('d', KEY_RIGHT):
            command.angular.z = -self.angular_speed
            self.status_message = '手操：原地右转'
        else:
            return None

        return command

    def _request_mode(self, mode: Mode, velocity: Twist | None = None) -> bool:
        request = ModeSrv.Request()
        request.target_mode = mode.value
        request.linear_velocity = velocity if velocity is not None else Twist()

        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)

        if not future.done():
            self.status_message = f'切换到 {mode.name} 超时。'
            return False

        response = future.result()
        if response is None:
            self.status_message = f'切换到 {mode.name} 失败：服务无响应。'
            return False

        if cast(bool, response.success):
            self.status_message = cast(str, response.message)
            return True

        self.status_message = cast(str, response.message)
        return False

    def publish_stop(self) -> None:
        self.cmd_vel_raw_publisher.publish(Twist())

    def _draw_menu(self) -> None:
        self._clear_screen()
        print('KiBotOne 控制台')
        print()
        print(f'当前模式: {self._current_mode_label()}')
        print(f'状态: {self.status_message}')
        print()
        print('W/S 或 ↑/↓ 选择模式，Enter 确认，q 退出')
        print()

        for index, (_, label) in enumerate(self.menu_items):
            prefix = '>' if index == self.selected_index else ' '
            print(f'{prefix} {label}')

    def _draw_manual_screen(self) -> None:
        self._clear_screen()
        print('KiBotOne 控制台 / MANUAL 手操')
        print()
        print(f'当前模式: {self._current_mode_label()}')
        print(f'状态: {self.status_message}')
        print()
        print('W/↑ 前进    S/↓ 后退')
        print('A/← 左转    D/→ 右转')
        print('Enter/Space 停止，q 返回菜单')

    def _current_mode_label(self) -> str:
        if self.current_mode is None:
            return '未知'
        return f'{self.current_mode.value} {self.current_mode.name}'

    def _read_key(self) -> str | None:
        tty.setraw(sys.stdin.fileno())
        try:
            readable, _, _ = select.select([sys.stdin], [], [], self.poll_timeout)
            if not readable:
                return None

            key = sys.stdin.read(1)
            if key == '\x1b':
                return self._read_escape_key()
            if key in ('\r', '\n'):
                return KEY_ENTER
            if key == ' ':
                return KEY_SPACE
            return key.lower()
        finally:
            self._restore_terminal()

    def _read_escape_key(self) -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], 0.01)
        if not readable:
            return None

        sequence = sys.stdin.read(2)
        match sequence:
            case '[A':
                return KEY_UP
            case '[B':
                return KEY_DOWN
            case '[C':
                return KEY_RIGHT
            case '[D':
                return KEY_LEFT
            case _:
                return None

    def _read_float(self, prompt: str, default: float) -> float:
        while True:
            value = input(prompt).strip()
            if value == '':
                return default
            try:
                return float(value)
            except ValueError:
                print('请输入有效数字。')

    def _restore_terminal(self) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)

    @staticmethod
    def _clear_screen() -> None:
        print('\033[2J\033[H', end='')


def main(args=None) -> None:
    control_console: ControlConsole | None = None

    try:
        rclpy.init(args=args)
        control_console = ControlConsole()
        control_console.run()
    except RuntimeError as error:
        print(error)
    except (KeyboardInterrupt, ExternalShutdownException):
        if control_console is not None:
            control_console.publish_stop()
    finally:
        if control_console is not None:
            control_console._restore_terminal()
            control_console.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
