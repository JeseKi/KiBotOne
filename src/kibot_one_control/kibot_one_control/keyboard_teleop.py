import select
import sys
import termios
import tty
from typing import cast
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.srv import GetParameters_Response # type: ignore

from geometry_msgs.msg import Twist # type: ignore

MODE_CONTROL_NAME = "mode_control"

class KeyboardTeleop(Node):

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_raw_topic', 'cmd_vel_raw')
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('poll_timeout', 0.1)
        self.mode_control_param_client = AsyncParameterClient(node=self, remote_node_name=MODE_CONTROL_NAME)
        self.mode_control_ok = self.mode_control_param_client.wait_for_services(timeout_sec=3.0)
        if not self.mode_control_ok:
            self.get_logger().error(f"{MODE_CONTROL_NAME} 节点参数不可用")

        cmd_vel_raw_topic = self.get_parameter(
            'cmd_vel_raw_topic'
        ).get_parameter_value().string_value

        self.linear_speed = self.get_parameter(
            'linear_speed'
        ).get_parameter_value().double_value
        self.angular_speed = self.get_parameter(
            'angular_speed'
        ).get_parameter_value().double_value
        self.poll_timeout = self.get_parameter(
            'poll_timeout'
        ).get_parameter_value().double_value

        self.cmd_vel_raw_publisher = self.create_publisher(Twist, cmd_vel_raw_topic, 10)

        self._settings = termios.tcgetattr(sys.stdin)

    def print_help(self) -> None:
        print(
            '控制器正在运行.\n'
            '使用 WASD 进行移动.\n'
            '  w: 前进\n'
            '  s: 后退\n'
            '  a: 原地左转\n'
            '  d: 原地右转\n'
            'space/x: 停止\n'
            'q: 退出\n'
        )

    def run(self) -> None:
        self.print_help()
        current_mode = 2
        try:
            while rclpy.ok():
                key = self.read_key()
                if key is None:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                if key == 'q':
                    self.publish_stop()
                    break

                if key in ('x', ' '):
                    self.publish_stop()
                    continue

                cmd = self.key_to_twist(key)


                if self.mode_control_ok:
                    future = self.mode_control_param_client.get_parameters(names=["mode"])
                    rclpy.spin_until_future_complete(node=self, future=future, timeout_sec=3.0)

                    if not future.done() or future.result() is None:
                        self.get_logger().error(f"获取 {MODE_CONTROL_NAME} 参数失败")
                    else:
                        response = cast(GetParameters_Response, future.result())
                        if not response.values:
                            self.get_logger().error(f"{MODE_CONTROL_NAME} 未返回 mode 参数")
                        else:
                            current_mode = response.values[0].integer_value # type: ignore
                            
                if cmd is not None and current_mode == 2:
                    self.cmd_vel_raw_publisher.publish(cmd)
        finally:
            self.restore_terminal()

    def read_key(self) -> str | None:
        tty.setraw(sys.stdin.fileno())
        readable, _, _ = select.select([sys.stdin], [], [], self.poll_timeout)
        if not readable:
            self.restore_terminal()
            return None

        key = sys.stdin.read(1)
        self.restore_terminal()
        return key.lower()

    def key_to_twist(self, key: str) -> Twist | None:
        msg = Twist()

        if key == 'w':
            msg.linear.x = self.linear_speed
            print("W - 向前")
        elif key == 's':
            msg.linear.x = -self.linear_speed
            print("S - 向后")
        elif key == 'a':
            msg.angular.z = self.angular_speed
            print("A - 原地左转")
        elif key == 'd':
            msg.angular.z = -self.angular_speed
            print("D - 原地右转")
        else:
            return None

        return msg

    def publish_stop(self) -> None:
        self.cmd_vel_raw_publisher.publish(Twist())

    def restore_terminal(self) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)


def main(args=None) -> None:
    keyboard_teleop: KeyboardTeleop | None = None

    try:
        rclpy.init(args=args)
        keyboard_teleop = KeyboardTeleop()
        keyboard_teleop.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if keyboard_teleop is not None:
            keyboard_teleop.publish_stop()
    finally:
        if keyboard_teleop is not None:
            keyboard_teleop.restore_terminal()
            keyboard_teleop.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
