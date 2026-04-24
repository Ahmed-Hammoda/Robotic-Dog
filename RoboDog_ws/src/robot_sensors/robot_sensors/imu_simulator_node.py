"""
Fallback IMU simulator when hardware is unavailable.
Publishes zeroed quaternion and gyro data so mapper can run.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


class ImuSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_simulator_node')
        
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('topic', '/imu/data')
        
        frequency = float(self.get_parameter('publish_frequency').value)
        topic = str(self.get_parameter('topic').value)
        
        self.pub = self.create_publisher(Imu, topic, 10)
        self.create_timer(1.0 / frequency, self._publish_imu)
        
        self.get_logger().warn('IMU Simulator: Publishing zero quaternion (hardware unavailable)')

    def _publish_imu(self) -> None:
        """Publish dummy IMU data: identity quaternion, zero angular velocity."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Zero-rotated quaternion [x, y, z, w] = [0, 0, 0, 1]
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.orientation_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        
        # Zero angular velocity
        msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Zero linear acceleration
        msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=-9.81)
        msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
