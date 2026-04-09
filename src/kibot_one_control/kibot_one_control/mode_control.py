from typing import Any, cast
from enum import IntEnum

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist # type: ignore

from kibot_one_interface.srv import Mode as ModeSrv # type: ignore

class Mode(IntEnum):
    STOP = 0
    CRUISE = 1
    MANUAL = 2
    FOLLOW = 3

class ModeControl(Node):
    def __init__(self) -> None:
        super().__init__(node_name="mode_control")

        self.linear_velocity = Twist()

        params = [
            ("mode", Mode.MANUAL),
            ("mode_pub_timer_period", 0.01667)
        ]
        self.declare_parameters(namespace="", parameters=cast(Any, params))
        
        self.srv = self.create_service(srv_type=ModeSrv, srv_name="mode_control", callback=self._change_mode)
        
        self.mode_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel_raw",
            qos_profile=10
        )
        self.pub_timer = self.create_timer(
            timer_period_sec=cast(
                float, self.get_parameter("mode_pub_timer_period").value
            ),
            callback=self._pub_timer_callback
        )

    def _pub_timer_callback(self) -> None:
        current_mode = self._get_current_mode()
        match current_mode:
            case Mode.STOP:
                self.linear_velocity = Twist()
            case Mode.CRUISE:
                pass
            case Mode.MANUAL:
                self.linear_velocity = Twist()
            case Mode.FOLLOW:
                pass
        
        if current_mode not in [Mode.MANUAL, Mode.FOLLOW]:
            self.mode_vel_publisher.publish(self.linear_velocity)

    def _change_mode(self, request: ModeSrv.Request, response: ModeSrv.Response) -> ModeSrv.Response:
        message = ""
        
        try:
            current_mode = self._get_current_mode()
            new_mode = Mode(value=request.target_mode)
            new_mode_param = Parameter(
                name="mode",
                type_=Parameter.Type.INTEGER,
                value=new_mode.value
            )
            all_new_parameters = [new_mode_param]
            self.set_parameters(all_new_parameters)
            if new_mode == Mode.CRUISE:
                self.linear_velocity = request.linear_velocity
            
            response.success = True
            response.message = f"成功改变运行模式 {current_mode.name} -> {new_mode.name}"

            return response

        except ValueError:
            message += f"不支持的运行模式枚举值: {request.target_mode}\n"

            for member in Mode:
                message += f"支持的运行模式：{member.value} -> {member.name}\n"

            self.get_logger().error(message=message)

            response.success = False
            response.message = message

            return response
        
        except Exception as e:
            message = f"改变运行模式时出现错误 {e}"

            self.get_logger().error(message=message)

            response.success = False
            response.message = message

            return response
        
    def _get_current_mode(self) -> Mode:
        return Mode(self.get_parameter("mode").value)
        
def main(args = None) -> None:
    mode_control = None
    try:
        rclpy.init(args=args)
        mode_control = ModeControl()
        rclpy.spin(mode_control)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if mode_control is not None:
            mode_control.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()