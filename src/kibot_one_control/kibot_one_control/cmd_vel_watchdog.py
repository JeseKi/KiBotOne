from typing import Any, cast

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist # type: ignore

class CMDVelWatchDog(Node):

    def __init__(self) -> None:
        super().__init__(node_name='cmd_vel_watchdog')
        
        self.last_topic_msg_timestamp = self.get_clock().now()
        self.last_command_was_zero = True

        params = [
            ("stop_time_period", 0.5), # seconds
            ("watch_time_period", 0.1), # seconds
        ]
        self.declare_parameters(namespace="", parameters=cast(Any, params))

        self.cmd_vel_raw_subscription = self.create_subscription(
            msg_type=Twist,
            topic='cmd_vel_raw',
            callback=self.cmd_vel_raw_listener_callback,
            qos_profile=10
        )
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10
        )

        self.watch_timer = self.create_timer(
            timer_period_sec=cast(
                float, self.get_parameter(name="watch_time_period").get_parameter_value().double_value
            ),
            callback=self.watch_timer_callback
        )

    def cmd_vel_raw_listener_callback(self, msg: Twist) -> None:
        self.last_topic_msg_timestamp = self.get_clock().now()
        self.cmd_vel_publisher.publish(msg)
        if msg.linear.x != 0 or \
            msg.linear.y != 0 or \
            msg.linear.z != 0 or \
            msg.angular.x != 0 or \
            msg.angular.y != 0 or \
            msg.angular.z != 0:
            self.last_command_was_zero = False
        else:
            self.last_command_was_zero = True

    def watch_timer_callback(self) -> None:
        future = self.last_topic_msg_timestamp + Duration(
            seconds=cast(
                float, self.get_parameter(name="stop_time_period").get_parameter_value().double_value
            )
        )
        if not self.last_command_was_zero and self.get_clock().now() > future:
            msg = Twist()

            self.cmd_vel_publisher.publish(msg=msg)
            
            self.last_command_was_zero = True

def main(args=None) -> None:
    cmd_vel_watchdog = None
    try:
        rclpy.init(args=args)
        cmd_vel_watchdog = CMDVelWatchDog()
        rclpy.spin(cmd_vel_watchdog)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if cmd_vel_watchdog is not None:
            cmd_vel_watchdog.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()