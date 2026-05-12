import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickNode(Node):
    def __init__(self) -> None:
        super().__init__('joystick_node')
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('linear_axis_sign', 1.0)
        self.declare_parameter('angular_axis_sign', 1.0)
        self.declare_parameter('linear_scale', 0.6)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('button_linear_scale', 0.6)
        self.declare_parameter('button_angular_scale', 1.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('full_scale_threshold', 0.9)
        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('stop_button', -1)
        self.declare_parameter('forward_button', -1)
        self.declare_parameter('backward_button', -1)
        self.declare_parameter('left_button', -1)
        self.declare_parameter('right_button', -1)
        self.declare_parameter('stand_button', -1)
        self.declare_parameter('sit_button', -1)
        self.declare_parameter('publish_zero_when_inactive', True)

        self.linear_axis = int(self.get_parameter('linear_axis').value)
        self.angular_axis = int(self.get_parameter('angular_axis').value)
        self.linear_axis_sign = float(self.get_parameter('linear_axis_sign').value)
        self.angular_axis_sign = float(self.get_parameter('angular_axis_sign').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.button_linear_scale = float(self.get_parameter('button_linear_scale').value)
        self.button_angular_scale = float(self.get_parameter('button_angular_scale').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.full_scale_threshold = float(self.get_parameter('full_scale_threshold').value)
        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)
        self.forward_button = int(self.get_parameter('forward_button').value)
        self.backward_button = int(self.get_parameter('backward_button').value)
        self.left_button = int(self.get_parameter('left_button').value)
        self.right_button = int(self.get_parameter('right_button').value)
        self.stand_button = int(self.get_parameter('stand_button').value)
        self.sit_button = int(self.get_parameter('sit_button').value)
        self.publish_zero_when_inactive = bool(self.get_parameter('publish_zero_when_inactive').value)

        self.twist_pub = self.create_publisher(Twist, '/robodog/joystick/cmd_vel', 10)
        self.raw_pub = self.create_publisher(Joy, '/robodog/joystick/raw', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0

        # Some controllers never hit exactly +/-1.0; snap near-max values.
        if value >= self.full_scale_threshold:
            return 1.0
        if value <= -self.full_scale_threshold:
            return -1.0

        return value

    def joy_callback(self, msg: Joy) -> None:
        self.raw_pub.publish(msg)

        deadman_pressed = self._read_button(msg, self.deadman_button)
        stop_pressed = self._read_button(msg, self.stop_button)
        forward_pressed = self._read_button(msg, self.forward_button)
        backward_pressed = self._read_button(msg, self.backward_button)
        left_pressed = self._read_button(msg, self.left_button)
        right_pressed = self._read_button(msg, self.right_button)
        stand_pressed = self._read_button(msg, self.stand_button)
        sit_pressed = self._read_button(msg, self.sit_button)

        twist = Twist()
        direction = 'idle'

        if stand_pressed:
            direction = 'stand'
            self.twist_pub.publish(twist)
            return

        if sit_pressed:
            self.twist_pub.publish(twist)
            return

        if stop_pressed:
            self.twist_pub.publish(twist)
            return

        if self.deadman_button >= 0 and not deadman_pressed:
            if self.publish_zero_when_inactive:
                self.twist_pub.publish(twist)
            return

        if forward_pressed:
            twist.linear.x = self.button_linear_scale
            self.twist_pub.publish(twist)
            return

        if backward_pressed:
            twist.linear.x = -self.button_linear_scale
            self.twist_pub.publish(twist)
            return

        if left_pressed:
            twist.angular.z = self.button_angular_scale
            self.twist_pub.publish(twist)
            return

        if right_pressed:
            twist.angular.z = -self.button_angular_scale
            self.twist_pub.publish(twist)
            return

        if len(msg.axes) > self.linear_axis:
            value = msg.axes[self.linear_axis] * self.linear_axis_sign
            twist.linear.x = self._apply_deadzone(value) * self.linear_scale
        if len(msg.axes) > self.angular_axis:
            value = msg.axes[self.angular_axis] * self.angular_axis_sign
            twist.angular.z = self._apply_deadzone(value) * self.angular_scale
        self.twist_pub.publish(twist)

    def _read_button(self, msg: Joy, button_index: int) -> bool:
        return button_index >= 0 and len(msg.buttons) > button_index and msg.buttons[button_index] == 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
