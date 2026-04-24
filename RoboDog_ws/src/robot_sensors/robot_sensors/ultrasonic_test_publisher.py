"""Publish fake ultrasonic distance data for testing occupancy mapping without hardware."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class UltrasonicTestPublisher(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_test_publisher')

        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('left_topic', '/ultrasonic1')
        self.declare_parameter('right_topic', '/ultrasonic2')

        frequency = float(self.get_parameter('publish_frequency').value)
        left_topic = str(self.get_parameter('left_topic').value)
        right_topic = str(self.get_parameter('right_topic').value)

        self.pub_left = self.create_publisher(Float32, left_topic, 10)
        self.pub_right = self.create_publisher(Float32, right_topic, 10)

        self.time_offset = 0.0
        self.create_timer(1.0 / frequency, self._publish_data)

        self.get_logger().info(f'ultrasonic_test_publisher started (freq={frequency} Hz)')

    def _publish_data(self) -> None:
        """Publish test data: simulated obstacles at different angles."""
        self.time_offset += 0.1

        # Simulate a wall 1.0m ahead
        dist_left = 1.0 + 0.1 * math.sin(self.time_offset)  # meters → centimeters
        dist_right = 1.0 + 0.1 * math.cos(self.time_offset)

        # Publish as Float32 (centimeters) to match ultrasonic_node output format
        msg_left = Float32()
        msg_left.data = float(dist_left * 100.0)
        self.pub_left.publish(msg_left)

        msg_right = Float32()
        msg_right.data = float(dist_right * 100.0)
        self.pub_right.publish(msg_right)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
