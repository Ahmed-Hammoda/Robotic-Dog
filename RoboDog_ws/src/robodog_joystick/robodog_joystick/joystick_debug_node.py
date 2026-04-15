import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickDebugNode(Node):
    def __init__(self) -> None:
        super().__init__('joystick_debug_node')
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('stop_button', -1)

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg: Joy) -> None:
        linear = msg.axes[self.linear_axis] if len(msg.axes) > self.linear_axis else 0.0
        angular = msg.axes[self.angular_axis] if len(msg.axes) > self.angular_axis else 0.0
        deadman = self._read_button(msg, self.deadman_button)
        stop = self._read_button(msg, self.stop_button)

        self.get_logger().info(
            f'axes: linear={linear:.2f}, angular={angular:.2f} | '
            f'buttons: deadman={deadman}, stop={stop} | '
            f'raw_axes={list(msg.axes)} raw_buttons={list(msg.buttons)}'
        )

    def _read_button(self, msg: Joy, button_index: int) -> bool:
        return button_index >= 0 and len(msg.buttons) > button_index and msg.buttons[button_index] == 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()