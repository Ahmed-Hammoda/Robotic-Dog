import importlib
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


GRAVITY_M_S2 = 9.80665


@dataclass
class KalmanFilter1D:
    """Simple scalar Kalman filter for noisy sensor channels."""

    process_variance: float
    measurement_variance: float
    estimate: Optional[float] = None
    error_covariance: float = 1.0

    def update(self, measurement: float) -> float:
        if self.estimate is None:
            self.estimate = measurement
            return measurement

        self.error_covariance += self.process_variance
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error_covariance = (1.0 - kalman_gain) * self.error_covariance
        return self.estimate


class MPU9250Node(Node):
    def __init__(self) -> None:
        super().__init__('mpu9250_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('parent_frame_id', 'map')
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('mpu_address', 0x68)
        self.declare_parameter('auto_detect_mpu_address', True)
        self.declare_parameter('fall_threshold_deg', 30.0)
        self.declare_parameter('held_z_target_m_s2', GRAVITY_M_S2)
        self.declare_parameter('held_z_tolerance_m_s2', 1.0)
        self.declare_parameter('held_gyro_threshold_rad_s', 0.5)
        self.declare_parameter('kalman_process_variance_accel', 0.02)
        self.declare_parameter('kalman_measurement_variance_accel', 0.5)
        self.declare_parameter('kalman_process_variance_gyro', 0.01)
        self.declare_parameter('kalman_measurement_variance_gyro', 0.2)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.parent_frame_id = str(self.get_parameter('parent_frame_id').value)
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.mpu_address_param = int(self.get_parameter('mpu_address').value)
        self.auto_detect_mpu_address = bool(self.get_parameter('auto_detect_mpu_address').value)
        self.fall_threshold_deg = float(self.get_parameter('fall_threshold_deg').value)
        self.held_z_target_m_s2 = float(self.get_parameter('held_z_target_m_s2').value)
        self.held_z_tolerance_m_s2 = float(self.get_parameter('held_z_tolerance_m_s2').value)
        self.held_gyro_threshold_rad_s = float(self.get_parameter('held_gyro_threshold_rad_s').value)

        if self.publish_frequency <= 0.0:
            self.get_logger().warn('publish_frequency must be > 0. Falling back to 20.0 Hz')
            self.publish_frequency = 20.0

        accel_q = float(self.get_parameter('kalman_process_variance_accel').value)
        accel_r = float(self.get_parameter('kalman_measurement_variance_accel').value)
        gyro_q = float(self.get_parameter('kalman_process_variance_gyro').value)
        gyro_r = float(self.get_parameter('kalman_measurement_variance_gyro').value)

        self.bus_id = 1
        self.bus = None
        self.mpu_addr: Optional[int] = None

        self.accel_filters: Dict[str, KalmanFilter1D] = {
            'x': KalmanFilter1D(accel_q, accel_r),
            'y': KalmanFilter1D(accel_q, accel_r),
            'z': KalmanFilter1D(accel_q, accel_r),
        }
        self.gyro_filters: Dict[str, KalmanFilter1D] = {
            'x': KalmanFilter1D(gyro_q, gyro_r),
            'y': KalmanFilter1D(gyro_q, gyro_r),
            'z': KalmanFilter1D(gyro_q, gyro_r),
        }

        self.held_counter = 0
        self.held_debounce_threshold = 10

        self.az_offset = 0.0
        self.calibration_done = False

        self.imu_raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.imu_filtered_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.pitch_pub = self.create_publisher(Float32, '/imu/pitch_deg', 10)
        self.roll_pub = self.create_publisher(Float32, '/imu/roll_deg', 10)
        self.fall_pub = self.create_publisher(Bool, '/imu/fall_detected', 10)
        self.held_pub = self.create_publisher(Bool, '/imu/held_detected', 10)
        self.status_pub = self.create_publisher(String, '/imu/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/imu/visualization', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self._open_bus()
        if self.bus is None:
            return

        selected_addr = self._select_mpu_address()
        if selected_addr is None:
            self.get_logger().error(
                f'MPU-9250 not found on I2C bus {self.bus_id} at 0x68/0x69 '
                f'(requested 0x{self.mpu_address_param:02X}).'
            )
            return

        self.mpu_addr = selected_addr
        if not self._init_mpu9250():
            self.get_logger().error('Failed to initialize MPU-9250')
            return

        self.get_logger().info(
            f'Ready: frame_id={self.frame_id}, parent_frame_id={self.parent_frame_id}, '
            f'publish_frequency={self.publish_frequency:.2f} Hz, mpu_address=0x{self.mpu_addr:02X}, '
            f'fall_threshold_deg={self.fall_threshold_deg:.1f}, held_z_target={self.held_z_target_m_s2:.2f} m/s^2'
        )
        
        self.get_logger().info('Calibrating Z-axis acceleration (keep robot flat and still for 2 seconds)...')
        self._calibrate_z_axis()
        self.get_logger().info(f'Z-axis offset calibrated: {self.az_offset:.3f} m/s^2')
        
        self.create_timer(1.0 / self.publish_frequency, self.timer_callback)

    def _open_bus(self) -> None:
        try:
            smbus2 = importlib.import_module('smbus2')
            self.bus = smbus2.SMBus(self.bus_id)
        except Exception as exc:
            self.bus = None
            self.get_logger().error(f'Could not open I2C bus {self.bus_id}: {exc}')

    def _probe(self, addr: int) -> bool:
        if self.bus is None:
            return False
        try:
            self.bus.read_byte(addr)
            return True
        except Exception:
            return False

    def _select_mpu_address(self) -> Optional[int]:
        if self.bus is None:
            return None

        configured = self.mpu_address_param & 0x7F
        if self.auto_detect_mpu_address:
            candidates = [configured] + [addr for addr in (0x68, 0x69) if addr != configured]
        else:
            candidates = [configured]

        for addr in candidates:
            try:
                who_am_i = self.bus.read_byte_data(addr, 0x75)
                if who_am_i in (0x71, 0x73):
                    self.get_logger().info(f'Found MPU device at 0x{addr:02X} with WHO_AM_I=0x{who_am_i:02X}')
                    return addr
                self.get_logger().warn(
                    f'I2C device at 0x{addr:02X} returned unexpected WHO_AM_I=0x{who_am_i:02X}; using it anyway.'
                )
                return addr
            except Exception:
                continue

        for addr in candidates:
            if self._probe(addr):
                self.get_logger().warn(f'Found I2C device at 0x{addr:02X} by probe only.')
                return addr

        return None

    def _init_mpu9250(self) -> bool:
        if self.bus is None or self.mpu_addr is None:
            return False

        try:
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x01)
            time.sleep(0.05)
            self.bus.write_byte_data(self.mpu_addr, 0x1A, 0x03)
            self.bus.write_byte_data(self.mpu_addr, 0x1B, 0x00)
            self.bus.write_byte_data(self.mpu_addr, 0x1C, 0x00)
            self.bus.write_byte_data(self.mpu_addr, 0x1D, 0x03)
            return True
        except Exception as exc:
            self.get_logger().error(f'MPU-9250 init failed: {exc}')
            return False

    @staticmethod
    def _to_int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    def _read_imu(self) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        if self.bus is None or self.mpu_addr is None:
            return None

        try:
            data = self.bus.read_i2c_block_data(self.mpu_addr, 0x3B, 14)
        except Exception as exc:
            self.get_logger().warn(f'MPU-9250 read failed: {exc}')
            return None

        ax_raw = self._to_int16(data[0], data[1])
        ay_raw = self._to_int16(data[2], data[3])
        az_raw = self._to_int16(data[4], data[5])
        temp_raw = self._to_int16(data[6], data[7])
        gx_raw = self._to_int16(data[8], data[9])
        gy_raw = self._to_int16(data[10], data[11])
        gz_raw = self._to_int16(data[12], data[13])

        ax = (ax_raw / 16384.0) * GRAVITY_M_S2
        ay = (ay_raw / 16384.0) * GRAVITY_M_S2
        az = (az_raw / 16384.0) * GRAVITY_M_S2
        gx = (gx_raw / 131.0) * (math.pi / 180.0)
        gy = (gy_raw / 131.0) * (math.pi / 180.0)
        gz = (gz_raw / 131.0) * (math.pi / 180.0)
        temp_c = (temp_raw / 333.87) + 21.0
        return ax, ay, az, gx, gy, gz, temp_c

    def _calibrate_z_axis(self, samples: int = 40) -> None:
        """
        Calibrate Z-axis by averaging readings while robot is flat and stationary.
        At 20 Hz, 40 samples ≈ 2 seconds.
        """
        if self.bus is None or self.mpu_addr is None:
            return

        sum_az = 0.0
        count = 0
        start = time.time()
        
        while count < samples:
            imu_data = self._read_imu()
            if imu_data is None:
                time.sleep(0.01)
                continue
            
            _ax, _ay, az, _gx, _gy, _gz, _temp = imu_data
            sum_az += az
            count += 1
            
            elapsed = time.time() - start
            if count % 10 == 0:
                self.get_logger().info(f'Calibration: {count}/{samples} samples ({elapsed:.1f}s)')
            
            time.sleep(0.001)
        
        self.az_offset = sum_az / samples
        self.calibration_done = True

    def _kalman_filter(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> Tuple[float, float, float, float, float, float]:
        return (
            self.accel_filters['x'].update(ax),
            self.accel_filters['y'].update(ay),
            self.accel_filters['z'].update(az),
            self.gyro_filters['x'].update(gx),
            self.gyro_filters['y'].update(gy),
            self.gyro_filters['z'].update(gz),
        )

    @staticmethod
    def _roll_pitch_from_accel(ax: float, ay: float, az: float) -> Tuple[float, float]:
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return roll, pitch

    @staticmethod
    def _quaternion_from_roll_pitch(roll: float, pitch: float) -> Tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        qx = sr * cp
        qy = cr * sp
        qz = -sr * sp
        qw = cr * cp
        return qx, qy, qz, qw

    def _publish_marker_array(self, stamp, pitch_deg: float, roll_deg: float, fall_detected: bool, held_detected: bool) -> None:
        marker_array = MarkerArray()

        cube = Marker()
        cube.header.stamp = stamp
        cube.header.frame_id = self.frame_id
        cube.ns = 'mpu9250_status'
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = 0.0
        cube.pose.position.y = 0.0
        cube.pose.position.z = 0.0
        cube.pose.orientation.w = 1.0
        cube.scale.x = 0.22
        cube.scale.y = 0.14
        cube.scale.z = 0.08
        if fall_detected:
            cube.color.r = 1.0
            cube.color.g = 0.1
            cube.color.b = 0.1
        elif held_detected:
            cube.color.r = 0.1
            cube.color.g = 0.4
            cube.color.b = 1.0
        else:
            cube.color.r = 0.1
            cube.color.g = 0.9
            cube.color.b = 0.1
        cube.color.a = 0.95
        cube.lifetime.sec = 0
        cube.lifetime.nanosec = int(0.5 * 1e9)

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = self.frame_id
        text.ns = 'mpu9250_status'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = 0.18
        text.pose.orientation.w = 1.0
        text.scale.z = 0.06
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f'Pitch: {pitch_deg:.1f} deg\n'
            f'Roll: {roll_deg:.1f} deg\n'
            f'Fall: {"YES" if fall_detected else "NO"}\n'
            f'Held: {"YES" if held_detected else "NO"}'
        )
        text.lifetime.sec = 0
        text.lifetime.nanosec = int(0.5 * 1e9)

        marker_array.markers.append(cube)
        marker_array.markers.append(text)
        self.marker_pub.publish(marker_array)

    def timer_callback(self) -> None:
        imu_data = self._read_imu()
        if imu_data is None:
            return

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_c = imu_data
        ax, ay, az, gx, gy, gz = self._kalman_filter(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)
        roll_rad, pitch_rad = self._roll_pitch_from_accel(ax, ay, az)

        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        fall_detected = abs(roll_deg) > self.fall_threshold_deg or abs(pitch_deg) > self.fall_threshold_deg
        gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        
        held_condition = (
            abs(abs(az) - self.held_z_target_m_s2) <= self.held_z_tolerance_m_s2 and
            gyro_mag <= self.held_gyro_threshold_rad_s
        )
        
        if held_condition:
            self.held_counter += 1
        else:
            self.held_counter = 0
        
        held_detected = self.held_counter >= self.held_debounce_threshold

        stamp = self.get_clock().now().to_msg()

        raw_msg = Imu()
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id
        raw_msg.orientation_covariance[0] = -1.0
        raw_msg.angular_velocity.x = gx_raw
        raw_msg.angular_velocity.y = gy_raw
        raw_msg.angular_velocity.z = gz_raw
        raw_msg.linear_acceleration.x = ax_raw
        raw_msg.linear_acceleration.y = ay_raw
        raw_msg.linear_acceleration.z = az_raw
        self.imu_raw_pub.publish(raw_msg)

        qx, qy, qz, qw = self._quaternion_from_roll_pitch(roll_rad, pitch_rad)

        filtered_msg = Imu()
        filtered_msg.header.stamp = stamp
        filtered_msg.header.frame_id = self.frame_id
        filtered_msg.orientation.x = qx
        filtered_msg.orientation.y = qy
        filtered_msg.orientation.z = qz
        filtered_msg.orientation.w = qw
        filtered_msg.orientation_covariance[0] = 0.05
        filtered_msg.orientation_covariance[4] = 0.05
        filtered_msg.orientation_covariance[8] = 0.05
        filtered_msg.angular_velocity.x = gx
        filtered_msg.angular_velocity.y = gy
        filtered_msg.angular_velocity.z = gz
        filtered_msg.angular_velocity_covariance[0] = 0.02
        filtered_msg.angular_velocity_covariance[4] = 0.02
        filtered_msg.angular_velocity_covariance[8] = 0.02
        filtered_msg.linear_acceleration.x = ax
        filtered_msg.linear_acceleration.y = ay
        filtered_msg.linear_acceleration.z = az
        filtered_msg.linear_acceleration_covariance[0] = 0.2
        filtered_msg.linear_acceleration_covariance[4] = 0.2
        filtered_msg.linear_acceleration_covariance[8] = 0.2
        self.imu_filtered_pub.publish(filtered_msg)

        pitch_msg = Float32()
        pitch_msg.data = float(pitch_deg)
        self.pitch_pub.publish(pitch_msg)

        roll_msg = Float32()
        roll_msg.data = float(roll_deg)
        self.roll_pub.publish(roll_msg)

        fall_msg = Bool()
        fall_msg.data = bool(fall_detected)
        self.fall_pub.publish(fall_msg)

        held_msg = Bool()
        held_msg.data = bool(held_detected)
        self.held_pub.publish(held_msg)

        status_msg = String()
        status_msg.data = (
            f'pitch={pitch_deg:.1f} roll={roll_deg:.1f} '
            f'fall={"true" if fall_detected else "false"} '
            f'held={"true" if held_detected else "false"} '
            f'az={az:.2f} gyro_mag={gyro_mag:.2f} temp={temp_c:.2f}C'
        )
        self.status_pub.publish(status_msg)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.parent_frame_id
            transform.child_frame_id = self.frame_id
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

        self._publish_marker_array(stamp, pitch_deg, roll_deg, fall_detected, held_detected)

    def destroy_node(self) -> bool:
        try:
            if self.bus is not None:
                self.bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPU9250Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
