import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from smbus2 import SMBus, i2c_msg
from std_msgs.msg import Float32


class AHT10Node(Node):
    def __init__(self) -> None:
        super().__init__('aht10_node')

        self.bus_id = 1
        self.bus: Optional[SMBus] = None
        self.addresses: List[int] = [0x38, 0x39]
        self.active_addresses: List[int] = []
        self.last_read_valid: Dict[int, bool] = {}

        # Keep legacy topics for 0x38 compatibility and expose per-address topics.
        self.temp_publishers = {
            0x38: self.create_publisher(Float32, '/temperature', 10),
            0x39: self.create_publisher(Float32, '/temperature2', 10),
        }
        self.hum_publishers = {
            0x38: self.create_publisher(Float32, '/humidity', 10),
            0x39: self.create_publisher(Float32, '/humidity2', 10),
        }

        self._setup_i2c()
        if self.active_addresses:
            self.create_timer(1.0, self.timer_callback)
            detected = ', '.join([f'0x{addr:02X}' for addr in self.active_addresses])
            self.get_logger().info(f'AHT10 ready at: {detected}')
        else:
            self.get_logger().error('No AHT10 found at 0x38/0x39')

    def _setup_i2c(self) -> None:
        try:
            self.bus = SMBus(self.bus_id)
        except Exception as exc:
            self.get_logger().error(f'Could not open I2C bus {self.bus_id}: {exc}')
            self.bus = None
            return

        for addr in self.addresses:
            try:
                if not self._probe_address(addr):
                    continue
                if self._init_aht10(addr):
                    self.active_addresses.append(addr)
                    self.last_read_valid[addr] = True
                else:
                    self.get_logger().error(f'AHT10 found at 0x{addr:02X}, but init failed')
            except Exception as exc:
                self.get_logger().error(f'I2C setup failed for 0x{addr:02X}: {exc}')

    def _probe_address(self, addr: int) -> bool:
        if self.bus is None:
            return False
        try:
            self.bus.read_byte(addr)
            return True
        except Exception:
            return False

    def _init_aht10(self, addr: int) -> bool:
        if self.bus is None:
            return False

        # Some boards expose AHT10-compatible parts that prefer slightly different
        # startup command sequences. Try safe options before giving up.
        init_sequences = ([0xE1, 0x08, 0x00], [0xBE, 0x08, 0x00])

        for seq in init_sequences:
            for _ in range(3):
                try:
                    self._write_bytes(addr, seq)
                    time.sleep(0.02)
                    return True
                except Exception:
                    time.sleep(0.03)

        # Final attempt: soft reset then one more init.
        try:
            self._write_bytes(addr, [0xBA])
            time.sleep(0.03)
            self._write_bytes(addr, [0xE1, 0x08, 0x00])
            time.sleep(0.02)
            return True
        except Exception as exc:
            self.get_logger().warn(f'AHT10 init failed at 0x{addr:02X}: {exc}')
            return False

    def _read_aht10(self, addr: int) -> Tuple[float, float]:
        if self.bus is None:
            return float('nan'), float('nan')

        try:
            self._write_bytes(addr, [0xAC, 0x33, 0x00])
            time.sleep(0.08)

            data = self._read_bytes(addr, 6)
            if len(data) < 6:
                return float('nan'), float('nan')

            # If busy bit is still set, wait briefly and retry once.
            if data[0] & 0x80:
                time.sleep(0.02)
                data = self._read_bytes(addr, 6)
        except Exception as exc:
            self.get_logger().warn(f'AHT10 read failed at 0x{addr:02X}: {exc}')
            return float('nan'), float('nan')

        if data[0] & 0x80:
            return float('nan'), float('nan')

        raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        raw_temperature = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5])
        humidity = (raw_humidity * 100.0) / 1048576.0
        temperature = (raw_temperature * 200.0) / 1048576.0 - 50.0
        return temperature, humidity

    def timer_callback(self) -> None:
        if not self.active_addresses:
            return

        for addr in self.active_addresses:
            temperature, humidity = self._read_aht10(addr)

            if temperature != temperature or humidity != humidity:
                if self.last_read_valid.get(addr, True):
                    self.get_logger().warn(f'AHT10 0x{addr:02X} read returned invalid data')
                self.last_read_valid[addr] = False
                continue

            self.last_read_valid[addr] = True

            self._publish(self.temp_publishers[addr], temperature)
            self._publish(self.hum_publishers[addr], humidity)

            print(f'aht10(0x{addr:02X}) temperature={temperature:.2f} C | humidity={humidity:.2f} %')

    def _publish(self, publisher, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def _write_bytes(self, addr: int, values) -> None:
        if self.bus is None:
            raise RuntimeError('I2C bus not available')
        msg = i2c_msg.write(addr, list(values))
        self.bus.i2c_rdwr(msg)

    def _read_bytes(self, addr: int, count: int):
        if self.bus is None:
            raise RuntimeError('I2C bus not available')
        msg = i2c_msg.read(addr, count)
        self.bus.i2c_rdwr(msg)
        return list(msg)

    def destroy_node(self) -> bool:
        try:
            if self.bus is not None:
                self.bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AHT10Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()