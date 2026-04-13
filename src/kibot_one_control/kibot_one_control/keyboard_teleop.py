import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist # type: ignore
from kibot_one_interface.msg import ModeState # type: ignore

MANUAL_MODE_VALUE = 2

class KeyboardTeleop(Node):

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_raw_topic', 'cmd_vel_raw')
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('poll_timeout', 0.1)
        self.declare_parameter('mode_topic', 'mode')

        cmd_vel_raw_topic = self.get_parameter('cmd_vel_raw_topic').get_parameter_value().string_value
        mode_topic = self.get_parameter('mode_topic').get_parameter_value().string_value

        self.linear_speed = self.get_parameter(
            'linear_speed'
        ).get_parameter_value().double_value
        self.angular_speed = self.get_parameter(
            'angular_speed'
        ).get_parameter_value().double_value
        self.poll_timeout = self.get_parameter(
            'poll_timeout'
        ).get_parameter_value().double_value
        self.current_mode = MANUAL_MODE_VALUE
        self.mode_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_vel_raw_publisher = self.create_publisher(msg_type=Twist, topic=cmd_vel_raw_topic, qos_profile=10)
        self.mode_sub = self.create_subscription(msg_type=ModeState, topic=mode_topic, callback=self.mode_callback, qos_profile=self.mode_qos)

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

                if cmd is not None and self.current_mode == MANUAL_MODE_VALUE:
                    self.cmd_vel_raw_publisher.publish(cmd)
        finally:
            self.restore_terminal()

    def mode_callback(self, msg: ModeState) -> None:
        self.current_mode = msg.current_mode

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
