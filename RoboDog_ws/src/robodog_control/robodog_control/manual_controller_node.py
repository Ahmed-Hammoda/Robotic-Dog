from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from robodog_msgs.msg import MotorCommand, RobotStatus
from sensor_msgs.msg import Joy


class ManualControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('manual_controller_node')
        self.latest_cmd = Twist()
        self.latest_joy = Joy()
        self.sit_latched = False

        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('stop_button', -1)
        self.declare_parameter('forward_button', -1)
        self.declare_parameter('backward_button', -1)
        self.declare_parameter('left_button', -1)
        self.declare_parameter('right_button', -1)
        self.declare_parameter('stand_button', -1)
        self.declare_parameter('sit_button', -1)
        self.declare_parameter('axis_deadzone', 0.1)
        self.declare_parameter('control_period', 1.5)

        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.stop_button = int(self.get_parameter('stop_button').value)
        self.forward_button = int(self.get_parameter('forward_button').value)
        self.backward_button = int(self.get_parameter('backward_button').value)
        self.left_button = int(self.get_parameter('left_button').value)
        self.right_button = int(self.get_parameter('right_button').value)
        self.stand_button = int(self.get_parameter('stand_button').value)
        self.sit_button = int(self.get_parameter('sit_button').value)
        self.axis_deadzone = float(self.get_parameter('axis_deadzone').value)
        self.control_period = float(self.get_parameter('control_period').value)

        self.motor_pub = self.create_publisher(MotorCommand, '/robodog/motor/command', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/robodog/status/robot', 10)

        self.create_subscription(Twist, '/robodog/joystick/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Joy, '/robodog/joystick/raw', self.joy_callback, 10)
        self.create_timer(self.control_period, self.control_loop)

    def cmd_callback(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def joy_callback(self, msg: Joy) -> None:
        self.latest_joy = msg

# this is not a real controller, just a simple mapping from cmd_vel to motor commands for testing purposes
    def _build_targets(self, linear_x: float, angular_z: float) -> List[float]:
        targets = [0.0] * 12
        forward = max(min(linear_x, 1.0), -1.0)
        rotate = max(min(angular_z, 1.0), -1.0)

        for i in range(12):
            leg_side = -1.0 if (i % 3 == 0 or i % 3 == 1) else 1.0
            targets[i] = 0.4 * forward + 0.25 * rotate * leg_side
        return targets

    def _mode(self, linear_x: float, angular_z: float) -> str:
        if self._pressed(self.stand_button):
            self.sit_latched = False
            return 'stand_up'
        if self._pressed(self.sit_button):
            self.sit_latched = True
            return 'sit_down'

        if self.sit_latched:
            # Keep robot seated until an explicit stand command is given.
            return 'sit_down'

        if self._pressed(self.stop_button):
            return 'stop'
        if self._pressed(self.forward_button):
            return 'move_forward'
        if self._pressed(self.backward_button):
            return 'move_backward'
        if self._pressed(self.left_button):
            return 'rotate_angle_left'
        if self._pressed(self.right_button):
            return 'rotate_angle_right'

        if abs(linear_x) < self.axis_deadzone and abs(angular_z) < self.axis_deadzone:
            return 'stand'

        if abs(linear_x) >= abs(angular_z):
            return 'move_forward' if linear_x > 0.0 else 'move_backward'

        return 'rotate_angle_left' if angular_z > 0.0 else 'rotate_angle_right'

    def _pressed(self, button_index: int) -> bool:
        return button_index >= 0 and len(self.latest_joy.buttons) > button_index and self.latest_joy.buttons[button_index] == 1

    def control_loop(self) -> None:
        linear_x = self.latest_cmd.linear.x
        angular_z = self.latest_cmd.angular.z

        cmd = MotorCommand()
        cmd.target_positions = self._build_targets(linear_x, angular_z)
        cmd.speed_scale = min(1.0, max(0.1, abs(linear_x) + abs(angular_z)))
        cmd.mode = self._mode(linear_x, angular_z)
        self.motor_pub.publish(cmd)

        status = RobotStatus()
        status.motors_enabled = True
        status.control_mode = cmd.mode
        status.stm32_state = 'commanding'
        status.note = f'lin={linear_x:.2f}, ang={angular_z:.2f}'
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
