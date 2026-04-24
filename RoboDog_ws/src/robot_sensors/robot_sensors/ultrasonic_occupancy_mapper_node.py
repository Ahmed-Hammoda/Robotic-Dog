"""
2D occupancy map from ultrasonic sensors with IMU-based rotation.
Properly transforms sensor beams from base_link to map frame using robot orientation.
"""
import math
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32


class UltrasonicOccupancyMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_occupancy_mapper_node')

        # Map parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('map_topic', '/ultrasonic/local_map')
        self.declare_parameter('resolution', 0.05)  # 5 cm per cell
        self.declare_parameter('width', 200)  # 10m × 10m
        self.declare_parameter('height', 200)
        self.declare_parameter('publish_frequency', 5.0)

        # Sensor topics
        self.declare_parameter('sensor_1_topic', '/ultrasonic1/range')
        self.declare_parameter('sensor_2_topic', '/ultrasonic2/range')
        self.declare_parameter('imu_topic', '/imu/data')

        # Sensor positions in base_link frame
        self.declare_parameter('sensor_1_x', 0.12)
        self.declare_parameter('sensor_1_y', 0.06)
        self.declare_parameter('sensor_1_yaw_rad', 0.30)

        self.declare_parameter('sensor_2_x', 0.12)
        self.declare_parameter('sensor_2_y', -0.06)
        self.declare_parameter('sensor_2_yaw_rad', -0.30)

        # Confidence/occupancy parameters
        self.declare_parameter('occupied_increment', 18)
        self.declare_parameter('free_decrement', 3)
        self.declare_parameter('min_confidence', -60)
        self.declare_parameter('max_confidence', 100)
        self.declare_parameter('occupied_threshold', 20)
        self.declare_parameter('free_threshold', -20)
        self.declare_parameter('endpoint_radius_cells', 1)

        # Load parameters
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)

        self.occupied_increment = int(self.get_parameter('occupied_increment').value)
        self.free_decrement = int(self.get_parameter('free_decrement').value)
        self.min_confidence = int(self.get_parameter('min_confidence').value)
        self.max_confidence = int(self.get_parameter('max_confidence').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.endpoint_radius_cells = int(self.get_parameter('endpoint_radius_cells').value)

        # Publishers/subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 5)

        self.sensor_cfg = {
            's1': {
                'topic': str(self.get_parameter('sensor_1_topic').value),
                'base_x': float(self.get_parameter('sensor_1_x').value),
                'base_y': float(self.get_parameter('sensor_1_y').value),
                'base_yaw': float(self.get_parameter('sensor_1_yaw_rad').value),
            },
            's2': {
                'topic': str(self.get_parameter('sensor_2_topic').value),
                'base_x': float(self.get_parameter('sensor_2_x').value),
                'base_y': float(self.get_parameter('sensor_2_y').value),
                'base_yaw': float(self.get_parameter('sensor_2_yaw_rad').value),
            },
        }

        # State
        self.last_ranges: Dict[str, Optional[Range]] = {'s1': None, 's2': None}
        self.robot_yaw = 0.0  # yaw angle in map frame (from IMU)
        self.confidence_grid = [0] * (self.width * self.height)
        self.publish_count = 0
        self.robot_x = 0.0  # Robot position in map (center)
        self.robot_y = 0.0

        # Subscriptions
        self.create_subscription(Range, self.sensor_cfg['s1']['topic'], self._make_range_cb('s1'), 10)
        self.create_subscription(Range, self.sensor_cfg['s2']['topic'], self._make_range_cb('s2'), 10)
        self.create_subscription(Imu, str(self.get_parameter('imu_topic').value), self._imu_callback, 10)

        if self.publish_frequency <= 0.0:
            self.publish_frequency = 5.0
        self.create_timer(1.0 / self.publish_frequency, self._publish_grid)

        self.get_logger().info(f'{self.get_name()} started')

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """Convert quaternion to yaw angle (radians)."""
        # q = [x, y, z, w]
        x, y, z, w = q.x, q.y, q.z, q.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def _imu_callback(self, msg: Imu) -> None:
        """Update robot orientation from IMU."""
        self.robot_yaw = self._quaternion_to_yaw(msg.orientation)

    def _make_range_cb(self, key: str):
        def _callback(msg: Range) -> None:
            self.last_ranges[key] = msg
        return _callback

    def _world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """Convert world (map) coordinates to grid indices."""
        # Robot is at center of grid
        gx = int(round((x / self.resolution) + (self.width / 2)))
        gy = int(round((y / self.resolution) + (self.height / 2)))
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return None
        return gx, gy

    def _set_cell(self, data, x: int, y: int, value: int) -> None:
        idx = y * self.width + x
        if 0 <= idx < len(data):
            data[idx] = value

    def _update_confidence_cell(self, x: int, y: int, delta: int) -> None:
        idx = y * self.width + x
        if 0 <= idx < len(self.confidence_grid):
            current = self.confidence_grid[idx]
            updated = max(self.min_confidence, min(self.max_confidence, current + delta))
            self.confidence_grid[idx] = updated

    def _raytrace(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """Bresenham line algorithm to mark cells along ray."""
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0

        while True:
            self._update_confidence_cell(x, y, -self.free_decrement)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _mark_occupied_endpoint(self, x: int, y: int) -> None:
        """Mark endpoint as occupied with blurred blob."""
        radius = max(0, self.endpoint_radius_cells)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                px = x + dx
                py = y + dy
                if px < 0 or px >= self.width or py < 0 or py >= self.height:
                    continue
                if dx * dx + dy * dy <= radius * radius:
                    self._update_confidence_cell(px, py, self.occupied_increment)

    def _confidence_to_occupancy(self) -> list:
        """Convert confidence scores to occupancy values."""
        grid_data: list = [-1] * len(self.confidence_grid)
        for i, score in enumerate(self.confidence_grid):
            if score >= self.occupied_threshold:
                grid_data[i] = 100
            elif score <= self.free_threshold:
                grid_data[i] = 0
        return grid_data

    def _transform_sensor_to_map(self, sensor_x_base: float, sensor_y_base: float, 
                                  sensor_yaw_base: float, range_m: float) -> Tuple[float, float]:
        """
        Transform a sensor reading from base_link frame to map frame.
        
        Steps:
        1. Get sensor position in base_link
        2. Rotate sensor direction by robot yaw
        3. Add to robot center position
        """
        # Sensor position in base_link (robot frame)
        # Rotate to map frame based on robot orientation
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        
        # Sensor position rotated to map frame
        sensor_x_map = (sensor_x_base * cos_yaw - sensor_y_base * sin_yaw) + self.robot_x
        sensor_y_map = (sensor_x_base * sin_yaw + sensor_y_base * cos_yaw) + self.robot_y
        
        # Sensor direction (base_link frame) rotated to map frame
        sensor_yaw_map = sensor_yaw_base + self.robot_yaw
        
        # Ray endpoint in map frame
        ray_end_x = sensor_x_map + range_m * math.cos(sensor_yaw_map)
        ray_end_y = sensor_y_map + range_m * math.sin(sensor_yaw_map)
        
        return (sensor_x_map, sensor_y_map, ray_end_x, ray_end_y)

    def _publish_grid(self) -> None:
        """Update occupancy grid and publish."""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        # Origin at bottom-left (grid coordinates adjusted so center is robot)
        grid.info.origin.position.x = -0.5 * self.width * self.resolution
        grid.info.origin.position.y = -0.5 * self.height * self.resolution
        grid.info.origin.orientation.w = 1.0

        # Process sensor readings
        for key in ('s1', 's2'):
            msg = self.last_ranges[key]
            if msg is None or not math.isfinite(float(msg.range)):
                continue

            # Get sensor configuration
            cfg = self.sensor_cfg[key]
            range_m = float(msg.range)

            # Transform from base_link to map frame
            sensor_x_map, sensor_y_map, ray_end_x, ray_end_y = self._transform_sensor_to_map(
                cfg['base_x'], cfg['base_y'], cfg['base_yaw'], range_m
            )

            # Convert to grid coordinates
            start = self._world_to_grid(sensor_x_map, sensor_y_map)
            if start is None:
                continue

            end = self._world_to_grid(ray_end_x, ray_end_y)
            if end is None:
                continue

            # Raytrace from sensor to endpoint
            self._raytrace(start[0], start[1], end[0], end[1])
            self._mark_occupied_endpoint(end[0], end[1])

        # Convert confidence to occupancy and publish
        data = self._confidence_to_occupancy()
        grid.data = data

        known_cells = sum(1 for val in data if val != -1)
        self.publish_count += 1
        if known_cells > 0 and self.publish_count % 5 == 0:
            self.get_logger().info(f'Published grid: {known_cells} cells, robot_yaw={math.degrees(self.robot_yaw):.1f}°')

        self.map_pub.publish(grid)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicOccupancyMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
