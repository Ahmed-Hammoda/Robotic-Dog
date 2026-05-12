import rclpy
from rclpy.node import Node
from robodog_msgs.msg import MotorCommand
from std_msgs.msg import String
from std_srvs.srv import SetBool


class Stm32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('stm32_bridge_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('feedback_input_topic', '/robodog/stm32/raw_feedback')
        self.motors_enabled = True

        self.feedback_input_topic = str(self.get_parameter('feedback_input_topic').value)

        self.feedback_pub = self.create_publisher(String, '/robodog/stm32/feedback', 10)
        self.create_subscription(MotorCommand, '/robodog/motor/command', self.command_callback, 10)
        self.create_subscription(String, self.feedback_input_topic, self.feedback_callback, 10)
        self.create_service(SetBool, '/robodog/stm32/set_motors_enabled', self.set_enable_callback)

    def set_enable_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.motors_enabled = request.data
        response.success = True
        response.message = f'motors_enabled={self.motors_enabled}'
        return response

    def command_callback(self, msg: MotorCommand) -> None:
        if not self.motors_enabled:
            self.get_logger().warn('ignored motor command (motors disabled)')
            return

        # Placeholder for serial/UART packet transmission to STM32.
        self.get_logger().debug(f'sent mode={msg.mode} speed={msg.speed_scale:.2f}')

    def feedback_callback(self, msg: String) -> None:
        # STM32 is the source of truth; republish received feedback as-is.
        self.feedback_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
