from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Range
from std_msgs.msg import Float32


class UltrasonicRangeBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_range_bridge_node')

        self.declare_parameter('radiation_type', Range.ULTRASOUND)
        self.declare_parameter('field_of_view', 0.52)  # ~30 deg
        self.declare_parameter('min_range_m', 0.02)
        self.declare_parameter('max_range_m', 4.0)

        self.declare_parameter('input_topic_1', '/ultrasonic1')
        self.declare_parameter('output_topic_1', '/ultrasonic1/range')
        self.declare_parameter('frame_id_1', 'ultrasonic1_link')

        self.declare_parameter('input_topic_2', '/ultrasonic2')
        self.declare_parameter('output_topic_2', '/ultrasonic2/range')
        self.declare_parameter('frame_id_2', 'ultrasonic2_link')

        self.radiation_type = int(self.get_parameter('radiation_type').value)
        self.field_of_view = float(self.get_parameter('field_of_view').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)

        self._bridge_pubs: Dict[str, Publisher] = {}

        self._register_bridge(
            str(self.get_parameter('input_topic_1').value),
            str(self.get_parameter('output_topic_1').value),
            str(self.get_parameter('frame_id_1').value),
        )
        self._register_bridge(
            str(self.get_parameter('input_topic_2').value),
            str(self.get_parameter('output_topic_2').value),
            str(self.get_parameter('frame_id_2').value),
        )

        self.get_logger().info('ultrasonic_range_bridge_node started')

    def _register_bridge(self, input_topic: str, output_topic: str, frame_id: str) -> None:
        pub = self.create_publisher(Range, output_topic, 10)
        self._bridge_pubs[input_topic] = pub

        def _callback(msg: Float32) -> None:
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = frame_id
            range_msg.radiation_type = self.radiation_type
            range_msg.field_of_view = self.field_of_view
            range_msg.min_range = self.min_range_m
            range_msg.max_range = self.max_range_m

            value_m = float(msg.data) / 100.0  # source is in centimeters
            if value_m < self.min_range_m or value_m > self.max_range_m:
                range_msg.range = float('inf')
            else:
                range_msg.range = value_m

            pub.publish(range_msg)

        self.create_subscription(Float32, input_topic, _callback, 10)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicRangeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()