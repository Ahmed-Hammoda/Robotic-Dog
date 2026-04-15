import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from smbus2 import SMBus
from std_msgs.msg import Float32


class MPU9250Node(Node):
    def __init__(self) -> None:
        super().__init__('mpu9250_node')

        self.bus_id = 1
        self.mpu_addr = 0x68
        self.mag_addr = 0x0C
        self.bus: Optional[SMBus] = None
        self.mag_present = False

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.temp_pub = self.create_publisher(Float32, '/imu/temperature', 10)

        self._open_bus()
        if self.bus is None:
            return

        if not self._probe(self.mpu_addr):
            self.get_logger().error('MPU-9250 not found at 0x68')
            return

        if not self._init_mpu9250():
            self.get_logger().error('Failed to initialize MPU-9250')
            return

        self.mag_present = self._init_magnetometer()
        if self.mag_present:
            self.get_logger().info('MPU-9250 + AK8963 ready on I2C')
        else:
            self.get_logger().warn('MPU-9250 ready, magnetometer not detected')

        self.create_timer(0.05, self.timer_callback)

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
            # DLPF enabled, lower sensor noise.
            self.bus.write_byte_data(self.mpu_addr, 0x1A, 0x03)
            # Gyro full scale +/-250 dps.
            self.bus.write_byte_data(self.mpu_addr, 0x1B, 0x00)
            # Accel full scale +/-2g.
            self.bus.write_byte_data(self.mpu_addr, 0x1C, 0x00)
            # Enable I2C bypass to talk to AK8963 directly.
            self.bus.write_byte_data(self.mpu_addr, 0x37, 0x02)
            return True
        except Exception as exc:
            self.get_logger().error(f'MPU-9250 init failed: {exc}')
            return False

    def _init_magnetometer(self) -> bool:
        if self.bus is None:
            return False
        if not self._probe(self.mag_addr):
            return False

        try:
            # Power down first.
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x00)
            time.sleep(0.01)
            # Continuous measurement mode 2, 16-bit output.
            self.bus.write_byte_data(self.mag_addr, 0x0A, 0x16)
            time.sleep(0.01)
            return True
        except Exception as exc:
            self.get_logger().warn(f'AK8963 init failed: {exc}')
            return False

    @staticmethod
    def _to_int16(msb: int, lsb: int) -> int:
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    def _read_imu(self) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        if self.bus is None:
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

        # Scale factors from MPU-9250 datasheet default ranges.
        ax = (ax_raw / 16384.0) * 9.80665
        ay = (ay_raw / 16384.0) * 9.80665
        az = (az_raw / 16384.0) * 9.80665
        gx = math.radians(gx_raw / 131.0)
        gy = math.radians(gy_raw / 131.0)
        gz = math.radians(gz_raw / 131.0)
        temp_c = (temp_raw / 333.87) + 21.0

        return ax, ay, az, gx, gy, gz, temp_c

    def _read_mag(self) -> Optional[Tuple[float, float, float]]:
        if self.bus is None or not self.mag_present:
            return None

        try:
            st1 = self.bus.read_byte_data(self.mag_addr, 0x02)
            if (st1 & 0x01) == 0:
                return None

            data = self.bus.read_i2c_block_data(self.mag_addr, 0x03, 7)
        except Exception as exc:
            self.get_logger().warn(f'AK8963 read failed: {exc}')
            return None

        # Little-endian byte order for AK8963 axis output.
        mx_raw = self._to_int16(data[1], data[0])
        my_raw = self._to_int16(data[3], data[2])
        mz_raw = self._to_int16(data[5], data[4])

        # 16-bit mode sensitivity: 0.15 uT/LSB.
        mx = mx_raw * 0.15e-6
        my = my_raw * 0.15e-6
        mz = mz_raw * 0.15e-6
        return mx, my, mz

    def timer_callback(self) -> None:
        imu_data = self._read_imu()
        if imu_data is None:
            return

        ax, ay, az, gx, gy, gz, temp_c = imu_data
        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        self.imu_pub.publish(imu_msg)

        temp_msg = Float32()
        temp_msg.data = float(temp_c)
        self.temp_pub.publish(temp_msg)

        mag_data = self._read_mag()
        if mag_data is not None:
            mx, my, mz = mag_data
            mag_msg = MagneticField()
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = 'imu_link'
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz
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
