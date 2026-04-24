import math
from typing import Any, cast

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist  # type: ignore
from kibot_one_interface.msg import ModeState  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
from sensor_msgs.msg import LaserScan  # type: ignore
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data


FOLLOW_MODE_VALUE = 3


class FollowController(Node):
    def __init__(self) -> None:
        super().__init__(node_name='follow_controller')

        params = [
            ('mode_topic', 'mode'),
            ('odom_topic', 'odom'),
            ('robot_pose_topic', 'robot_pose'),
            ('flag_pose_topic', 'flag_pose'),
            ('scan_topic', 'scan'),
            ('cmd_vel_raw_topic', 'cmd_vel_raw'),
            ('control_period', 0.05),
            ('scan_timeout', 0.30),
            ('stop_distance', 0.60),
            ('heading_tolerance', 0.20),
            ('linear_gain', 0.60),
            ('angular_gain', 1.80),
            ('max_linear_speed', 0.50),
            ('max_angular_speed', 1.20),
            ('turn_only_angle', 0.80),
            ('reverse_turn_only_angle', 2.40),
            ('sector_count', 31),
            ('sector_fov', math.pi * 2),
            ('avoidance_range', 2.50),
            ('robot_body_radius', 0.371),
            ('obstacle_padding', 0.05),
            ('blocked_distance_scale', 1.10),
            ('min_speed_factor', 0.28),
            ('reverse_speed_ratio', 0.45),
            ('reverse_heading_bias', 0.30),
            ('clearance_weight', 0.52),
            ('goal_weight', 0.34),
            ('heading_weight', 0.14),
            ('lidar_offset_x', 0.20),
            ('lidar_offset_y', 0.0),
        ]

        self.declare_parameters(namespace='', parameters=cast(Any, params))


        self.robot_x: float | None = None
        self.robot_y: float | None = None
        self.robot_yaw: float | None = None
        self.has_robot_pose = False
        self.flag_x: float | None = None
        self.flag_y: float | None = None
        self.latest_scan: LaserScan | None = None
        self.last_scan_received_time: Time | None = None
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
            topic=cast(str, self.get_parameter('robot_pose_topic').value),
            callback=self._robot_pose_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=PoseStamped,
            topic=cast(str, self.get_parameter('flag_pose_topic').value),
            callback=self._flag_pose_callback,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=LaserScan,
            topic=cast(str, self.get_parameter('scan_topic').value),
            callback=self._scan_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.create_timer(
            timer_period_sec=cast(float, self.get_parameter('control_period').value),
            callback=self._control_loop,
        )

    def _mode_callback(self, msg: ModeState) -> None:
        self.current_mode = msg.current_mode

    def _odom_callback(self, msg: Odometry) -> None:
        if self.has_robot_pose:
            return

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

    def _robot_pose_callback(self, msg: PoseStamped) -> None:
        self.has_robot_pose = True
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.robot_yaw = self._yaw_from_quaternion(msg.pose.orientation)

    def _flag_pose_callback(self, msg: PoseStamped) -> None:
        self.flag_x = msg.pose.position.x
        self.flag_y = msg.pose.position.y

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_received_time = self.get_clock().now()

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

        if not self._has_fresh_scan():
            self.cmd_vel_raw_publisher.publish(Twist())
            return

        selected_heading_error, selected_clearance, scan_range_limit, should_reverse = self._select_heading(
            goal_heading_error=heading_error,
            goal_distance=distance,
        )

        linear_speed = min(
            cast(float, self.get_parameter('max_linear_speed').value),
            cast(float, self.get_parameter('linear_gain').value) * max(0.0, distance - stop_distance),
        )
        angular_speed = max(
            -cast(float, self.get_parameter('max_angular_speed').value),
            min(
                cast(float, self.get_parameter('max_angular_speed').value),
                cast(float, self.get_parameter('angular_gain').value) * selected_heading_error,
            ),
        )

        command = Twist()
        command.angular.z = angular_speed

        turn_only_angle = cast(float, self.get_parameter('turn_only_angle').value)
        reverse_turn_only_angle = cast(float, self.get_parameter('reverse_turn_only_angle').value)
        linear_direction = -1.0 if should_reverse else 1.0
        active_turn_only_angle = reverse_turn_only_angle if should_reverse else turn_only_angle

        if abs(selected_heading_error) < active_turn_only_angle:
            heading_tolerance = cast(float, self.get_parameter('heading_tolerance').value)
            motion_heading_error = selected_heading_error
            if should_reverse:
                motion_heading_error = self._normalize_angle(selected_heading_error - math.pi)

            heading_speed_factor = 1.0
            if abs(motion_heading_error) >= heading_tolerance:
                heading_speed_factor = max(0.0, math.cos(motion_heading_error))

            min_speed_factor = cast(float, self.get_parameter('min_speed_factor').value)
            clearance_speed_factor = self._clamp(selected_clearance / scan_range_limit, min_speed_factor, 1.0)
            commanded_speed = linear_speed * heading_speed_factor * clearance_speed_factor
            if should_reverse:
                commanded_speed *= cast(float, self.get_parameter('reverse_speed_ratio').value)

            command.linear.x = linear_direction * commanded_speed

        self.cmd_vel_raw_publisher.publish(command)

    def _has_fresh_scan(self) -> bool:
        if self.latest_scan is None or self.last_scan_received_time is None:
            return False

        scan_timeout = cast(float, self.get_parameter('scan_timeout').value)
        scan_age = (self.get_clock().now() - self.last_scan_received_time).nanoseconds / 1e9
        return scan_age <= scan_timeout

    def _select_heading(self, goal_heading_error: float, goal_distance: float) -> tuple[float, float, float, bool]:
        safe_radius = cast(float, self.get_parameter('robot_body_radius').value) + cast(
            float, self.get_parameter('obstacle_padding').value
        )
        scan_range_limit = self._scan_range_limit()
        goal_clearance = self._direction_clearance(goal_heading_error, safe_radius, scan_range_limit)
        direct_path_free = goal_clearance >= min(goal_distance, scan_range_limit)
        base_heading = goal_heading_error if direct_path_free else 0.0

        best_heading = goal_heading_error
        best_clearance = goal_clearance
        best_score = -math.inf

        sector_count = max(3, cast(int, self.get_parameter('sector_count').value))
        sector_fov = cast(float, self.get_parameter('sector_fov').value)
        blocked_distance = safe_radius * cast(float, self.get_parameter('blocked_distance_scale').value)
        clearance_weight = cast(float, self.get_parameter('clearance_weight').value)
        goal_weight = cast(float, self.get_parameter('goal_weight').value)
        heading_weight = cast(float, self.get_parameter('heading_weight').value)
        reverse_heading_bias = cast(float, self.get_parameter('reverse_heading_bias').value)
        best_should_reverse = False

        for index in range(sector_count):
            ratio = index / (sector_count - 1)
            sector_heading = self._normalize_angle(base_heading - sector_fov / 2.0 + ratio * sector_fov)
            sector_clearance = self._direction_clearance(sector_heading, safe_radius, scan_range_limit)
            clearance_ratio = sector_clearance / scan_range_limit
            goal_align = (math.cos(self._normalize_angle(sector_heading - goal_heading_error)) + 1.0) / 2.0
            heading_align = reverse_heading_bias + (1.0 - reverse_heading_bias) * ((math.cos(sector_heading) + 1.0) / 2.0)
            should_reverse = abs(sector_heading) > (math.pi / 2.0)

            if sector_clearance < blocked_distance:
                score = -9999.0 + clearance_ratio
            else:
                score = (
                    clearance_ratio * clearance_weight
                    + goal_align * goal_weight
                    + heading_align * heading_weight
                )

            if score > best_score:
                best_score = score
                best_heading = sector_heading
                best_clearance = sector_clearance
                best_should_reverse = should_reverse

        return best_heading, best_clearance, scan_range_limit, best_should_reverse

    def _scan_range_limit(self) -> float:
        if self.latest_scan is None:
            return max(0.1, cast(float, self.get_parameter('avoidance_range').value))

        return max(
            0.1,
            min(
                cast(float, self.get_parameter('avoidance_range').value),
                self.latest_scan.range_max,
            ),
        )

    def _direction_clearance(self, heading: float, safe_radius: float, scan_range_limit: float) -> float:
        if self.latest_scan is None:
            return 0.0

        direction_x = math.cos(heading)
        direction_y = math.sin(heading)
        lidar_offset_x = cast(float, self.get_parameter('lidar_offset_x').value)
        lidar_offset_y = cast(float, self.get_parameter('lidar_offset_y').value)
        robot_body_radius = cast(float, self.get_parameter('robot_body_radius').value)
        min_clearance = scan_range_limit

        for index, measured_range in enumerate(self.latest_scan.ranges):
            if not math.isfinite(measured_range) or measured_range < self.latest_scan.range_min:
                continue

            # No-return beams and obstacles beyond the local planning horizon should stay "clear".
            if measured_range >= self.latest_scan.range_max or measured_range > scan_range_limit:
                continue

            beam_angle = self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            point_x = lidar_offset_x + measured_range * math.cos(beam_angle)
            point_y = lidar_offset_y + measured_range * math.sin(beam_angle)

            # Ignore returns from the robot's own body footprint.
            if math.hypot(point_x, point_y) <= robot_body_radius + 0.02:
                continue

            projection = point_x * direction_x + point_y * direction_y
            if projection <= 0.0 or projection > scan_range_limit:
                continue

            perpendicular = abs(point_x * direction_y - point_y * direction_x)
            if perpendicular >= safe_radius:
                continue

            offset = math.sqrt(max(0.0, safe_radius * safe_radius - perpendicular * perpendicular))
            entry_distance = projection - offset
            if entry_distance <= 0.0:
                return 0.0

            if entry_distance < min_clearance:
                min_clearance = entry_distance

        return min_clearance

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)


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
