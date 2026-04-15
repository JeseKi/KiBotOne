import math
from typing import cast

import rclpy
from geometry_msgs.msg import PoseStamped, Twist  # type: ignore
from kibot_one_interface.msg import ModeState  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


FOLLOW_MODE_VALUE = 3


class FollowController(Node):
    def __init__(self) -> None:
        super().__init__(node_name='follow_controller')

        params = [
            ('mode_topic', 'mode'),
            ('odom_topic', 'odom'),
            ('flag_pose_topic', 'flag_pose'),
            ('cmd_vel_raw_topic', 'cmd_vel_raw'),
            ('control_period', 0.05),
            ('stop_distance', 0.60),
            ('heading_tolerance', 0.20),
            ('linear_gain', 0.60),
            ('angular_gain', 1.80),
            ('max_linear_speed', 0.50),
            ('max_angular_speed', 1.20),
        ]

        self.declare_parameters(namespace='', parameters=params)


        self.robot_x: float | None = None
        self.robot_y: float | None = None
        self.robot_yaw: float | None = None
        self.flag_x: float | None = None
        self.flag_y: float | None = None
        self.current_mode = 2
        self.mode_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_vel_raw_publisher = self.create_publisher(
            msg_type=Twist,
            topic=cast(str, self.get_parameter('cmd_vel_raw_topic').value),
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=ModeState,
            topic=cast(str, self.get_parameter('mode_topic').value),
            callback=self._mode_callback,
            qos_profile=self.mode_qos,
        )
        self.create_subscription(
            msg_type=Odometry,
            topic=cast(str, self.get_parameter('odom_topic').value),
            callback=self._odom_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=PoseStamped,
            topic=cast(str, self.get_parameter('flag_pose_topic').value),
            callback=self._flag_pose_callback,
            qos_profile=10,
        )
        self.create_timer(
            timer_period_sec=cast(float, self.get_parameter('control_period').value),
            callback=self._control_loop,
        )

    def _mode_callback(self, msg: ModeState) -> None:
        self.current_mode = msg.current_mode

    def _odom_callback(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _flag_pose_callback(self, msg: PoseStamped) -> None:
        self.flag_x = msg.pose.position.x
        self.flag_y = msg.pose.position.y

    def _control_loop(self) -> None:
        if self.current_mode != FOLLOW_MODE_VALUE:
            return

        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            self.cmd_vel_raw_publisher.publish(Twist())
            return

        if self.flag_x is None or self.flag_y is None:
            self.cmd_vel_raw_publisher.publish(Twist())
            return

        dx = self.flag_x - self.robot_x
        dy = self.flag_y - self.robot_y
        distance = math.hypot(dx, dy)
        stop_distance = cast(float, self.get_parameter('stop_distance').value)
        if distance <= stop_distance:
            self.cmd_vel_raw_publisher.publish(Twist())
            return

        target_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_yaw - self.robot_yaw)

        linear_speed = min(
            cast(float, self.get_parameter('max_linear_speed').value),
            cast(float, self.get_parameter('linear_gain').value) * max(0.0, distance - stop_distance),
        )
        angular_speed = max(
            -cast(float, self.get_parameter('max_angular_speed').value),
            min(
                cast(float, self.get_parameter('max_angular_speed').value),
                cast(float, self.get_parameter('angular_gain').value) * heading_error,
            ),
        )

        command = Twist()
        command.angular.z = angular_speed
        if abs(heading_error) < cast(float, self.get_parameter('heading_tolerance').value):
            command.linear.x = linear_speed

        self.cmd_vel_raw_publisher.publish(command)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    follow_controller = None
    try:
        rclpy.init(args=args)
        follow_controller = FollowController()
        rclpy.spin(follow_controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if follow_controller is not None:
            follow_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
