import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, MagneticField
from smbus2 import SMBus
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


class MadgwickFilter:
    """
    Lightweight Madgwick 9-axis sensor fusion filter.
    Produces quaternion orientation from accelerometer, gyroscope, and magnetometer.
    """

    def __init__(self, sample_rate: float = 20.0, beta: float = 0.1):
        """
        Args:
            sample_rate: Update frequency in Hz (1/Ts where Ts is sample period)
            beta: Algorithm gain (higher = faster convergence, less gyro integration drift)
        """
        self.sample_rate = sample_rate
        self.beta = beta
        # Quaternion: [x, y, z, w]
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

    def update_imu(
        self,
        gx: float, gy: float, gz: float,
        ax: float, ay: float, az: float,
    ) -> None:
        """
        Update filter with gyro (rad/s) and accel (m/s^2) data.
        Gyroscope-only integration with accelerometer drift correction.
        """
        # Normalize accelerometer
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 1e-6:
            return
        ax /= a_norm
        ay /= a_norm
        az /= a_norm

        # Compute error vector (accelerometer feedback)
        # Direction of accel in body frame from current quaternion estimate
        ax_est = 2.0 * (self.qx * self.qz - self.qw * self.qy)
        ay_est = 2.0 * (self.qw * self.qx + self.qy * self.qz)
        az_est = 1.0 - 2.0 * (self.qx * self.qx + self.qy * self.qy)

        # Cross product error (accel_measured × accel_estimated)
        ex = ay * az_est - az * ay_est
        ey = az * ax_est - ax * az_est
        ez = ax * ay_est - ay * ax_est

        # Apply proportional feedback
        gx += self.beta * ex
        gy += self.beta * ey
        gz += self.beta * ez

        # Integrate quaternion rate using feedback-corrected gyro values.
        gx_q = 0.5 * (-self.qy * gx - self.qz * gy - self.qw * gz)
        gy_q = 0.5 * (self.qx * gx + self.qz * gz - self.qw * gy)
        gz_q = 0.5 * (self.qx * gy - self.qy * gz + self.qw * gx)
        gw_q = 0.5 * (self.qx * gx + self.qy * gy + self.qz * gz)

        # Integrate quaternion rate
        dt = 1.0 / self.sample_rate
        self.qx += gx_q * dt
        self.qy += gy_q * dt
        self.qz += gz_q * dt
        self.qw += gw_q * dt

        # Normalize quaternion
        q_norm = math.sqrt(
            self.qx * self.qx + self.qy * self.qy +
            self.qz * self.qz + self.qw * self.qw
        )
        if q_norm > 1e-6:
            self.qx /= q_norm
            self.qy /= q_norm
            self.qz /= q_norm
            self.qw /= q_norm

    def update_marg(
        self,
        gx: float, gy: float, gz: float,
        ax: float, ay: float, az: float,
        mx: float, my: float, mz: float,
    ) -> None:
        """
        Update filter with 9-axis data (MARG: Magnetic, Angular Rate, Gravity).
        Full Madgwick with magnetometer correction for yaw drift.
        """
        # Normalize accelerometer
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 1e-6:
            return
        ax /= a_norm
        ay /= a_norm
        az /= a_norm

        # Normalize magnetometer
        m_norm = math.sqrt(mx * mx + my * my + mz * mz)
        if m_norm < 1e-6:
            return
        mx /= m_norm
        my /= m_norm
        mz /= m_norm

        # Compute reference direction of gravity in body frame
        # (inverse quaternion rotation of [0, 0, 1])
        gx_body = 2.0 * (self.qx * self.qz - self.qw * self.qy)
        gy_body = 2.0 * (self.qw * self.qx + self.qy * self.qz)
        gz_body = 1.0 - 2.0 * (self.qx * self.qx + self.qy * self.qy)

        # Error is sum of cross products between estimated direction and measured direction
        ex = ay * gz_body - az * gy_body
        ey = az * gx_body - ax * gz_body
        ez = ax * gy_body - ay * gx_body

        # Compute reference direction of magnetic field in body frame
        hx = 2.0 * mx * (0.5 - self.qy * self.qy - self.qz * self.qz) \
            + 2.0 * my * (self.qx * self.qy - self.qw * self.qz) \
            + 2.0 * mz * (self.qx * self.qz + self.qw * self.qy)
        hy = 2.0 * mx * (self.qx * self.qy + self.qw * self.qz) \
            + 2.0 * my * (0.5 - self.qx * self.qx - self.qz * self.qz) \
            + 2.0 * mz * (self.qy * self.qz - self.qw * self.qx)
        hz = 2.0 * mx * (self.qx * self.qz - self.qw * self.qy) \
            + 2.0 * my * (self.qy * self.qz + self.qw * self.qx) \
            + 2.0 * mz * (0.5 - self.qx * self.qx - self.qy * self.qy)

        # Magnetometer reference (North, no inclination)
        bx = math.sqrt(hx * hx + hy * hy)
        bz = hz

        # Error for magnetometer
        mx_ref = 2.0 * bx * (0.5 - self.qy * self.qy - self.qz * self.qz) + \
                 2.0 * bz * (self.qx * self.qz - self.qw * self.qy)
        my_ref = 2.0 * bx * (self.qx * self.qy - self.qw * self.qz) + \
                 2.0 * bz * (0.5 - self.qx * self.qx - self.qz * self.qz)
        mz_ref = 2.0 * bz * self.qx * self.qz + 2.0 * bx * self.qy

        ex += my * mz_ref - mz * my_ref
        ey += mz * mx_ref - mx * mz_ref
        ez += mx * my_ref - my * mx_ref

        # Apply proportional feedback
        gx += self.beta * ex
        gy += self.beta * ey
        gz += self.beta * ez

        # Integrate quaternion rate
        dt = 1.0 / self.sample_rate
        gx_q = 0.5 * (-self.qy * gx - self.qz * gy - self.qw * gz)
        gy_q = 0.5 * (self.qx * gx + self.qz * gz - self.qw * gy)
        gz_q = 0.5 * (self.qx * gy - self.qy * gz + self.qw * gx)
        gw_q = 0.5 * (self.qx * gx + self.qy * gy + self.qz * gz)

        self.qx += gx_q * dt
        self.qy += gy_q * dt
        self.qz += gz_q * dt
        self.qw += gw_q * dt

        # Normalize quaternion
        q_norm = math.sqrt(
            self.qx * self.qx + self.qy * self.qy +
            self.qz * self.qz + self.qw * self.qw
        )
        if q_norm > 1e-6:
            self.qx /= q_norm
            self.qy /= q_norm
            self.qz /= q_norm
            self.qw /= q_norm

    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """Return quaternion as (x, y, z, w)"""
        return (self.qx, self.qy, self.qz, self.qw)


class MPU9250Node(Node):
    def __init__(self) -> None:
        super().__init__('mpu9250_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('publish_tf_on_map_frame', False)
        self.declare_parameter('mpu_address', 0x68)
        self.declare_parameter('auto_detect_mpu_address', True)
        self.declare_parameter('use_magnetometer', True)
        self.declare_parameter('use_magnetometer_when_stationary', True)
        self.declare_parameter('zero_gyro_when_stationary', True)
        self.declare_parameter('stationary_gyro_threshold_rad_s', 0.01)
        self.declare_parameter('stationary_accel_tolerance_m_s2', 0.2)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.publish_tf_on_map_frame = bool(self.get_parameter('publish_tf_on_map_frame').value)
        self.mpu_address_param = int(self.get_parameter('mpu_address').value)
        self.auto_detect_mpu_address = bool(self.get_parameter('auto_detect_mpu_address').value)
        self.use_magnetometer = bool(self.get_parameter('use_magnetometer').value)
        self.use_magnetometer_when_stationary = bool(
            self.get_parameter('use_magnetometer_when_stationary').value
        )
        self.zero_gyro_when_stationary = bool(self.get_parameter('zero_gyro_when_stationary').value)
        self.stationary_gyro_threshold_rad_s = float(
            self.get_parameter('stationary_gyro_threshold_rad_s').value
        )
        self.stationary_accel_tolerance_m_s2 = float(
            self.get_parameter('stationary_accel_tolerance_m_s2').value
        )
        if self.publish_frequency <= 0.0:
            self.get_logger().warn('publish_frequency must be > 0. Falling back to 20.0 Hz')
            self.publish_frequency = 20.0

        self.bus_id = 1
        self.mpu_addr = 0x68
        self.mag_addr = 0x0C
        self.bus: Optional[SMBus] = None
        self.mag_present = False

        # Magnetometer sensitivity adjustment (ASA) from fuse ROM
        self.mag_asa_x = 1.0
        self.mag_asa_y = 1.0
        self.mag_asa_z = 1.0

        # Gyroscope bias (calibrated at startup)
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        # Sensor fusion filter
        self.filter = MadgwickFilter(sample_rate=self.publish_frequency, beta=0.1)

        # Publishers
        self.imu_raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.imu_filtered_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.temp_pub = self.create_publisher(Float32, '/imu/temperature', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.publish_tf_on_map_frame:
            self._publish_identity_tf_now()

        self._open_bus()
        if self.bus is None:
            return

        selected_addr = self._select_mpu_address()
        if selected_addr is None:
            self.get_logger().error(
                f'MPU-9250/6500 not found on I2C bus {self.bus_id} at 0x68/0x69 '
                f'(requested 0x{self.mpu_address_param:02X}).'
            )
            return
        self.mpu_addr = selected_addr

        if not self._init_mpu9250():
            self.get_logger().error('Failed to initialize MPU-9250')
            return

        self.mag_present = self._init_magnetometer()
        if self.mag_present:
            self.get_logger().info('MPU-9250 + AK8963 ready on I2C')
            self._read_magnetometer_calibration()
        else:
            self.get_logger().warn('MPU-9250 ready, magnetometer not detected')

        # Perform gyro bias calibration
        self.get_logger().info('Starting gyro bias calibration (3 seconds, keep robot stationary)...')
        self._calibrate_gyro_bias()
        self.get_logger().info(
            f'Gyro bias: gx={self.gyro_bias_x:.6f}, '
            f'gy={self.gyro_bias_y:.6f}, gz={self.gyro_bias_z:.6f} rad/s'
        )

        self.get_logger().info(
            f'frame_id={self.frame_id}, publish_frequency={self.publish_frequency:.2f}, '
            f'publish_tf_on_map_frame={self.publish_tf_on_map_frame}, '
            f'mpu_address=0x{self.mpu_addr:02X}, auto_detect_mpu_address={self.auto_detect_mpu_address}, '
            f'use_magnetometer={self.use_magnetometer}, '
            f'use_magnetometer_when_stationary={self.use_magnetometer_when_stationary}, '
            f'zero_gyro_when_stationary={self.zero_gyro_when_stationary}'
        )
        self.create_timer(1.0 / self.publish_frequency, self.timer_callback)

    def _publish_identity_tf_now(self) -> None:
        """Publish one startup transform so RViz can resolve imu frame immediately."""
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

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
                    self.get_logger().info(
                        f'Found MPU device at 0x{addr:02X} with WHO_AM_I=0x{who_am_i:02X}'
                    )
                    return addr

                # Some clones return unexpected IDs but still expose MPU registers.
                self.get_logger().warn(
                    f'I2C device at 0x{addr:02X} returned unexpected WHO_AM_I=0x{who_am_i:02X}; '
                    'trying as MPU-compatible device.'
                )
                return addr
            except Exception:
                continue

        # Fallback probe to catch devices that do not support WHO_AM_I reads reliably.
        for addr in candidates:
            if self._probe(addr):
                self.get_logger().warn(
                    f'Found I2C device at 0x{addr:02X} by probe only (WHO_AM_I read failed).'
                )
                return addr

        return None

    def _open_bus(self) -> None:
        try:
            self.bus = SMBus(self.bus_id)
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

    def _init_mpu9250(self) -> bool:
        if self.bus is None:
            return False
        try:
            # Wake device and use PLL with X axis gyroscope reference.
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0x01)
            time.sleep(0.05)
            # DLPF enabled (mode 3: 44 Hz BW, 4.9 ms delay).
            self.bus.write_byte_data(self.mpu_addr, 0x1A, 0x03)
            # Gyro full scale +/-250 dps (131 LSB/dps).
            self.bus.write_byte_data(self.mpu_addr, 0x1B, 0x00)
            # Accel full scale +/-2g (16384 LSB/g).
            self.bus.write_byte_data(self.mpu_addr, 0x1C, 0x00)
            # Enable I2C bypass to talk to AK8963 directly.
            self.bus.write_byte_data(self.mpu_addr, 0x37, 0x02)
            return True
        except Exception as exc:
            self.get_logger().error(f'MPU-9250 init failed: {exc}')
            return False

    def _init_magnetometer(self) -> bool:
        """Initialize AK8963 magnetometer in continuous mode."""
        if self.bus is None:
            return False
        if not self._probe(self.mag_addr):
            return False

        try:
            # Power down first.
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x00)
            time.sleep(0.01)
            # Fuse ROM access mode to read calibration data.
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x0F)
            time.sleep(0.01)
            return True
        except Exception as exc:
            self.get_logger().warn(f'AK8963 init failed: {exc}')
            return False

    def _read_magnetometer_calibration(self) -> None:
        """
        Read AK8963 sensitivity adjustment values (ASA) from fuse ROM.
        These are factory calibration values that MUST be applied to magnetometer readings.
        """
        if self.bus is None or not self.mag_present:
            return

        try:
            # Read ASA values from fuse ROM
            data = self.bus.read_i2c_block_data(self.mag_addr, 0x10, 3)
            # ASA to sensitivity multiplier: (ASA + 128) / 256
            self.mag_asa_x = (data[0] + 128.0) / 256.0
            self.mag_asa_y = (data[1] + 128.0) / 256.0
            self.mag_asa_z = (data[2] + 128.0) / 256.0

            self.get_logger().debug(
                f'Magnetometer ASA calibration: '
                f'x={self.mag_asa_x:.4f}, y={self.mag_asa_y:.4f}, z={self.mag_asa_z:.4f}'
            )

            # Exit fuse ROM access mode, start continuous measurement
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x00)
            time.sleep(0.01)
            # Continuous measurement mode 2, 16-bit output (100 Hz).
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x16)
            time.sleep(0.01)
        except Exception as exc:
            self.get_logger().warn(f'Failed to read magnetometer calibration: {exc}')

    def _calibrate_gyro_bias(self, samples: int = 300) -> None:
        """
        Estimate gyroscope bias by averaging samples while robot is stationary.
        At 20 Hz timer, 300 samples ≈ 15 seconds. Use ~300 for stability.
        """
        if self.bus is None:
            return

        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0
        count = 0

        start = time.time()
        while count < samples:
            imu_data = self._read_imu()
            if imu_data is None:
                time.sleep(0.001)
                continue

            _ax, _ay, _az, gx, gy, gz, _temp = imu_data
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            count += 1

            elapsed = time.time() - start
            if count % 50 == 0:
                self.get_logger().info(f'Calibration: {count}/{samples} samples ({elapsed:.1f}s)')

            time.sleep(0.001)

        self.gyro_bias_x = sum_gx / samples
        self.gyro_bias_y = sum_gy / samples
        self.gyro_bias_z = sum_gz / samples

    @staticmethod
    def _to_int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    def _read_imu(self) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        """
        Read raw IMU data from MPU-9250.
        Returns: (ax, ay, az, gx, gy, gz, temp_c)
        - Accel in m/s^2
        - Gyro in rad/s
        - Temp in Celsius
        """
        if self.bus is None:
            return None
        try:
            data = self.bus.read_i2c_block_data(self.mpu_addr, 0x3B, 14)
        except Exception as exc:
            self.get_logger().warn(f'MPU-9250 read failed: {exc}')
            return None

        # Parse raw values
        ax_raw = self._to_int16(data[0], data[1])
        ay_raw = self._to_int16(data[2], data[3])
        az_raw = self._to_int16(data[4], data[5])
        temp_raw = self._to_int16(data[6], data[7])
        gx_raw = self._to_int16(data[8], data[9])
        gy_raw = self._to_int16(data[10], data[11])
        gz_raw = self._to_int16(data[12], data[13])

        # Scale to proper units
        # Accelerometer: ±2g range, 16384 LSB/g
        ax = (ax_raw / 16384.0) * 9.80665
        ay = (ay_raw / 16384.0) * 9.80665
        az = (az_raw / 16384.0) * 9.80665

        # Gyroscope: ±250 dps range, 131 LSB/dps, convert to rad/s
        gx = (gx_raw / 131.0) * (math.pi / 180.0)
        gy = (gy_raw / 131.0) * (math.pi / 180.0)
        gz = (gz_raw / 131.0) * (math.pi / 180.0)

        # Temperature: (raw / 333.87) + 21.0 °C
        temp_c = (temp_raw / 333.87) + 21.0

        return ax, ay, az, gx, gy, gz, temp_c

    def _read_mag(self) -> Optional[Tuple[float, float, float]]:
        """
        Read magnetometer data from AK8963.
        Returns: (mx, my, mz) in Tesla (T)
        Applies factory sensitivity adjustment (ASA) for calibration.
        """
        if self.bus is None or not self.mag_present:
            return None

        try:
            # Read status register ST1
            st1 = self.bus.read_byte_data(self.mag_addr, 0x02)
            # Check data ready bit (bit 0)
            if (st1 & 0x01) == 0:
                return None

            # Read 7 bytes: HXL, HXH, HYL, HYH, HZL, HZH, ST2
            data = self.bus.read_i2c_block_data(self.mag_addr, 0x03, 7)

            # Check overflow bit in ST2 (bit 3)
            st2 = data[6]
            if (st2 & 0x08) != 0:
                self.get_logger().warn('Magnetometer overflow detected')
                return None

        except Exception as exc:
            self.get_logger().warn(f'AK8963 read failed: {exc}')
            return None

        # Parse values (little-endian byte order for AK8963)
        mx_raw = self._to_int16(data[1], data[0])
        my_raw = self._to_int16(data[3], data[2])
        mz_raw = self._to_int16(data[5], data[4])

        # AK8963 16-bit mode sensitivity: 0.15 µT/LSB
        # Apply ASA calibration: value * (ASA + 128) / 256
        mx = mx_raw * 0.15e-6 * self.mag_asa_x
        my = my_raw * 0.15e-6 * self.mag_asa_y
        mz = mz_raw * 0.15e-6 * self.mag_asa_z

        return mx, my, mz

    def timer_callback(self) -> None:
        """
        Main sensor update loop at 20 Hz.
        Publishes both raw and filtered IMU data.
        """
        imu_data = self._read_imu()
        if imu_data is None:
            return

        ax, ay, az, gx_raw, gy_raw, gz_raw, temp_c = imu_data
        stamp = self.get_clock().now().to_msg()

        # Publish raw IMU (no orientation)
        imu_raw_msg = Imu()
        imu_raw_msg.header.stamp = stamp
        imu_raw_msg.header.frame_id = self.frame_id
        # -1 means orientation not available (raw data only)
        imu_raw_msg.orientation_covariance[0] = -1.0
        imu_raw_msg.angular_velocity.x = gx_raw
        imu_raw_msg.angular_velocity.y = gy_raw
        imu_raw_msg.angular_velocity.z = gz_raw
        imu_raw_msg.angular_velocity_covariance[0] = 0.0001  # (rad/s)^2, ~0.005 rad/s σ
        imu_raw_msg.angular_velocity_covariance[4] = 0.0001
        imu_raw_msg.angular_velocity_covariance[8] = 0.0001
        imu_raw_msg.linear_acceleration.x = ax
        imu_raw_msg.linear_acceleration.y = ay
        imu_raw_msg.linear_acceleration.z = az
        imu_raw_msg.linear_acceleration_covariance[0] = 0.001  # (m/s^2)^2, ~0.032 m/s^2 σ
        imu_raw_msg.linear_acceleration_covariance[4] = 0.001
        imu_raw_msg.linear_acceleration_covariance[8] = 0.001
        self.imu_raw_pub.publish(imu_raw_msg)

        # Remove gyro bias
        gx = gx_raw - self.gyro_bias_x
        gy = gy_raw - self.gyro_bias_y
        gz = gz_raw - self.gyro_bias_z

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        is_stationary = (
            abs(gx) < self.stationary_gyro_threshold_rad_s and
            abs(gy) < self.stationary_gyro_threshold_rad_s and
            abs(gz) < self.stationary_gyro_threshold_rad_s and
            abs(accel_norm - 9.80665) < self.stationary_accel_tolerance_m_s2
        )

        if self.zero_gyro_when_stationary and is_stationary:
            # Clamp residual gyro bias while stationary to reduce slow orientation drift.
            gx = 0.0
            gy = 0.0
            gz = 0.0

        # Update sensor fusion filter with 9-axis data
        mag_data = self._read_mag()
        use_mag_now = (
            self.use_magnetometer and
            mag_data is not None and
            (self.use_magnetometer_when_stationary or not is_stationary)
        )
        if use_mag_now:
            mx, my, mz = mag_data
            self.filter.update_marg(gx, gy, gz, ax, ay, az, mx, my, mz)
        else:
            # Fall back to 6-axis if magnetometer is unavailable/disabled.
            self.filter.update_imu(gx, gy, gz, ax, ay, az)

        # Publish filtered IMU with orientation
        qx, qy, qz, qw = self.filter.get_quaternion()

        imu_filtered_msg = Imu()
        imu_filtered_msg.header.stamp = stamp
        imu_filtered_msg.header.frame_id = self.frame_id
        # Filtered orientation from Madgwick
        imu_filtered_msg.orientation.x = qx
        imu_filtered_msg.orientation.y = qy
        imu_filtered_msg.orientation.z = qz
        imu_filtered_msg.orientation.w = qw
        # Orientation covariance (from 9-axis fusion)
        # Estimated uncertainty: ~0.001 radians squared
        imu_filtered_msg.orientation_covariance[0] = 0.001
        imu_filtered_msg.orientation_covariance[4] = 0.001
        imu_filtered_msg.orientation_covariance[8] = 0.001
        # Angular velocity from gyroscope (after bias removal)
        imu_filtered_msg.angular_velocity.x = gx
        imu_filtered_msg.angular_velocity.y = gy
        imu_filtered_msg.angular_velocity.z = gz
        imu_filtered_msg.angular_velocity_covariance[0] = 0.0001
        imu_filtered_msg.angular_velocity_covariance[4] = 0.0001
        imu_filtered_msg.angular_velocity_covariance[8] = 0.0001
        # Linear acceleration (filtered through fusion)
        imu_filtered_msg.linear_acceleration.x = ax
        imu_filtered_msg.linear_acceleration.y = ay
        imu_filtered_msg.linear_acceleration.z = az
        imu_filtered_msg.linear_acceleration_covariance[0] = 0.001
        imu_filtered_msg.linear_acceleration_covariance[4] = 0.001
        imu_filtered_msg.linear_acceleration_covariance[8] = 0.001
        self.imu_filtered_pub.publish(imu_filtered_msg)

        if self.publish_tf_on_map_frame:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map'
            t.child_frame_id = self.frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # Publish temperature
        temp_msg = Float32()
        temp_msg.data = float(temp_c)
        self.temp_pub.publish(temp_msg)

        # Publish raw magnetometer
        if mag_data is not None:
            mx, my, mz = mag_data
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz
            mag_msg.magnetic_field_covariance[0] = 1e-12  # (T)^2, very small
            mag_msg.magnetic_field_covariance[4] = 1e-12
            mag_msg.magnetic_field_covariance[8] = 1e-12
            self.mag_pub.publish(mag_msg)

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
