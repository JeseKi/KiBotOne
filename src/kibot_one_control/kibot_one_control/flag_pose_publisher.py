from typing import Any, cast

import rclpy
from geometry_msgs.msg import PoseStamped  # type: ignore
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FlagPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='flag_pose_publisher')

        params = [
            ('flag_pose_topic', 'flag_pose'),
            ('frame_id', 'odom'),
            ('publish_period', 0.1),
            ('flag_x', 2.5),
            ('flag_y', 1.0),
            ('flag_z', 0.0),
        ]
        self.declare_parameters(namespace='', parameters=cast(Any, params))

        topic_name = cast(str, self.get_parameter(name='flag_pose_topic').value)
        publish_period = cast(float, self.get_parameter(name='publish_period').value)

        self.flag_pose_publisher = self.create_publisher(msg_type=PoseStamped, topic=topic_name, qos_profile=10)
        self.flag_pose_msg = PoseStamped()
        self.flag_pose_msg.header.frame_id = cast(str, self.get_parameter(name='frame_id').value)
        self.flag_pose_msg.pose.position.x = cast(float, self.get_parameter(name='flag_x').value)
        self.flag_pose_msg.pose.position.y = cast(float, self.get_parameter(name='flag_y').value)
        self.flag_pose_msg.pose.position.z = cast(float, self.get_parameter(name='flag_z').value)
        self.flag_pose_msg.pose.orientation.w = 1.0

        self.create_timer(timer_period_sec=publish_period, callback=self._publish_flag_pose)

    def _publish_flag_pose(self) -> None:
        self.flag_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.flag_pose_publisher.publish(msg=self.flag_pose_msg)


def main(args=None) -> None:
    flag_pose_publisher = None
    try:
        rclpy.init(args=args)
        flag_pose_publisher = FlagPosePublisher()
        rclpy.spin(node=flag_pose_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if flag_pose_publisher is not None:
            flag_pose_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
