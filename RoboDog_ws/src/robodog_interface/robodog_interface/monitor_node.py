import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Joy, MagneticField
from robodog_msgs.msg import MotorCommand, RobotStatus
from std_msgs.msg import Float32, String


class MonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('monitor_node')
        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0
        self.mode_deadzone = 0.12
        self.start_time = self.get_clock().now()
        self.declare_parameter('status_period', 1.5)
        self.declare_parameter('stream_url', 'http://localhost:8080/stream?topic=/robodog/camera/detections/image')
        self.status_period = float(self.get_parameter('status_period').value)
        self.stream_url = str(self.get_parameter('stream_url').value)

        self.camera_frame_count = 0
        self.last_camera_frame_count = 0
        self.last_fps_time = self.get_clock().now()
        self.camera_fps = 0.0

        self.latest = {
            'twist': 'n/a',
            'joy_raw': 'n/a',
            'joy_axes': [],
            'joy_buttons': [],
            'robot_move': 'n/a',
            'robot_mode': 'n/a',
            'stm32_state': '',
            'motors_enabled': True,
            'imu_accel_x': 0.0,
            'imu_accel_y': 0.0,
            'imu_accel_z': 0.0,
            'imu_gyro_x': 0.0,
            'imu_gyro_y': 0.0,
            'imu_gyro_z': 0.0,
            'imu_mag_x': 0.0,
            'imu_mag_y': 0.0,
            'imu_mag_z': 0.0,
            'temperature': 'n/a',
            'humidity': 'n/a',
            'temperature2': 'n/a',
            'humidity2': 'n/a',
            'ultrasonic1': 'n/a',
            'ultrasonic2': 'n/a',
            'imu_accel': 'n/a',
            'imu_gyro': 'n/a',
            'imu_mag': 'n/a',
            'imu_temp': 'n/a',
            'temperature_value': None,
            'humidity_value': None,
            'temperature2_value': None,
            'humidity2_value': None,
            'ultrasonic1_value': None,
            'ultrasonic2_value': None,
            'imu_temp_value': None,
            'detections': 'n/a',
        }

        self.status_pub = self.create_publisher(String, '/robodog/interface/status', 10)
        self.status_json_pub = self.create_publisher(String, '/robodog/interface/status_json', 10)
        self.ui_cmd_vel_pub = self.create_publisher(Twist, '/robodog/joystick/cmd_vel', 10)

        self.create_subscription(Twist, '/robodog/joystick/cmd_vel', self.on_twist, 10)
        self.create_subscription(Twist, '/robodog/interface/cmd_vel', self.on_interface_cmd_vel, 10)
        self.create_subscription(Joy, '/robodog/joystick/raw', self.on_joy_raw, 10)
        self.create_subscription(MotorCommand, '/robodog/motor/command', self.on_motor, 10)
        self.create_subscription(RobotStatus, '/robodog/status/robot', self.on_robot_status, 10)
        self.create_subscription(Float32, '/temperature', self.on_temperature, 10)
        self.create_subscription(Float32, '/humidity', self.on_humidity, 10)
        self.create_subscription(Float32, '/temperature2', self.on_temperature2, 10)
        self.create_subscription(Float32, '/humidity2', self.on_humidity2, 10)
        self.create_subscription(Float32, '/ultrasonic1', self.on_ultrasonic1, 10)
        self.create_subscription(Float32, '/ultrasonic2', self.on_ultrasonic2, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.on_mag, 10)
        self.create_subscription(Float32, '/imu/temperature', self.on_imu_temp, 10)
        self.create_subscription(Image, '/robodog/camera/detections/image', self.on_detection_image, 10)                                                                                                                                                                                                                         
        self.create_subscription(String, '/robodog/camera/detections/text', self.on_detections_text, 10)

        self.create_timer(self.status_period, self.publish_status)

    def on_twist(self, msg: Twist) -> None:
        self.latest_linear_x = msg.linear.x
        self.latest_angular_z = msg.angular.z
        self.latest['twist'] = f'lin={msg.linear.x:.2f} ang={msg.angular.z:.2f}'

    def on_interface_cmd_vel(self, msg: Twist) -> None:
        self.ui_cmd_vel_pub.publish(msg)

    def on_joy_raw(self, msg: Joy) -> None:
        self.latest['joy_axes'] = [float(x) for x in msg.axes]
        self.latest['joy_buttons'] = [int(x) for x in msg.buttons]
        axes_preview = ','.join([f'{x:.2f}' for x in msg.axes[:4]]) if msg.axes else ''
        buttons_preview = ','.join([str(int(x)) for x in msg.buttons[:8]]) if msg.buttons else ''
        self.latest['joy_raw'] = f'axes[{axes_preview}] buttons[{buttons_preview}]'

    def _resolve_generic_mode(self, mode: str) -> str:
        if not mode or not mode.strip():
            return 'stand'

        m = mode.strip().lower()

        if m in ('move_forward', 'forward'):
            return 'move_forward'
        if m in ('move_backward', 'backward'):
            return 'move_backward'
        if m in ('rotate_angle_left', 'turn_left', 'left'):
            return 'rotate_angle_left'
        if m in ('rotate_angle_right', 'turn_right', 'right'):
            return 'rotate_angle_right'

        if m == 'walk':
            if abs(self.latest_linear_x) < self.mode_deadzone:
                return 'stand'
            return 'move_forward' if self.latest_linear_x > 0.0 else 'move_backward'

        if m == 'turn':
            if abs(self.latest_angular_z) < self.mode_deadzone:
                return 'stand'
            return 'rotate_angle_left' if self.latest_angular_z > 0.0 else 'rotate_angle_right'

        return mode

    def on_motor(self, msg: MotorCommand) -> None:
        mode = self._resolve_generic_mode(msg.mode)
        self.latest['robot_move'] = mode

    def on_robot_status(self, msg: RobotStatus) -> None:
        mode = self._resolve_generic_mode(msg.control_mode)
        self.latest['robot_mode'] = mode
        self.latest['stm32_state'] = msg.stm32_state
        self.latest['motors_enabled'] = bool(msg.motors_enabled)

    def on_temperature(self, msg: Float32) -> None:
        self.latest['temperature_value'] = float(msg.data)
        self.latest['temperature'] = f'{msg.data:.2f}C'

    def on_humidity(self, msg: Float32) -> None:
        self.latest['humidity_value'] = float(msg.data)
        self.latest['humidity'] = f'{msg.data:.2f}%'

    def on_temperature2(self, msg: Float32) -> None:
        self.latest['temperature2_value'] = float(msg.data)
        self.latest['temperature2'] = f'{msg.data:.2f}C'

    def on_humidity2(self, msg: Float32) -> None:
        self.latest['humidity2_value'] = float(msg.data)
        self.latest['humidity2'] = f'{msg.data:.2f}%'

    def on_ultrasonic1(self, msg: Float32) -> None:
        self.latest['ultrasonic1_value'] = float(msg.data)
        self.latest['ultrasonic1'] = f'{msg.data:.2f}cm'

    def on_ultrasonic2(self, msg: Float32) -> None:
        self.latest['ultrasonic2_value'] = float(msg.data)
        self.latest['ultrasonic2'] = f'{msg.data:.2f}cm'

    def on_imu(self, msg: Imu) -> None:
        self.latest['imu_accel_x'] = float(msg.linear_acceleration.x)
        self.latest['imu_accel_y'] = float(msg.linear_acceleration.y)
        self.latest['imu_accel_z'] = float(msg.linear_acceleration.z)
        self.latest['imu_gyro_x'] = float(msg.angular_velocity.x)
        self.latest['imu_gyro_y'] = float(msg.angular_velocity.y)
        self.latest['imu_gyro_z'] = float(msg.angular_velocity.z)
        self.latest['imu_accel'] = (
            f'x={msg.linear_acceleration.x:.2f} y={msg.linear_acceleration.y:.2f} '
            f'z={msg.linear_acceleration.z:.2f}m/s2'
        )
        self.latest['imu_gyro'] = (
            f'x={msg.angular_velocity.x:.2f} y={msg.angular_velocity.y:.2f} '
            f'z={msg.angular_velocity.z:.2f}rad/s'
        )

    def on_mag(self, msg: MagneticField) -> None:
        self.latest['imu_mag_x'] = float(msg.magnetic_field.x)
        self.latest['imu_mag_y'] = float(msg.magnetic_field.y)
        self.latest['imu_mag_z'] = float(msg.magnetic_field.z)
        self.latest['imu_mag'] = (
            f'x={msg.magnetic_field.x:.5f} y={msg.magnetic_field.y:.5f} '
            f'z={msg.magnetic_field.z:.5f}T'
        )

    def on_imu_temp(self, msg: Float32) -> None:
        self.latest['imu_temp_value'] = float(msg.data)
        self.latest['imu_temp'] = f'{msg.data:.2f}C'

    def on_detection_image(self, _msg: Image) -> None:
        self.camera_frame_count += 1

    def on_detections_text(self, msg: String) -> None:
        self.latest['detections'] = msg.data if msg.data else 'none'

    def _safe_number(self, value, default=None):
        if value is None:
            return default
        try:
            f = float(value)
        except (TypeError, ValueError):
            return default
        if math.isfinite(f):
            return f
        return default

    def _build_status_json(self) -> str:
        payload = {
            'joy_cmd': {
                'linear_x': self._safe_number(self.latest_linear_x, 0.0),
                'angular_z': self._safe_number(self.latest_angular_z, 0.0),
            },
            'joy_raw': {
                'axes': self.latest['joy_axes'] if self.latest['joy_axes'] else [],
                'buttons': self.latest['joy_buttons'] if self.latest['joy_buttons'] else [],
            },
            'sensors': {
                'temp1_c': self._safe_number(self.latest['temperature_value'], None),
                'hum1_pct': self._safe_number(self.latest['humidity_value'], None),
                'temp2_c': self._safe_number(self.latest['temperature2_value'], None),
                'hum2_pct': self._safe_number(self.latest['humidity2_value'], None),
                'ultrasonic1_cm': self._safe_number(self.latest['ultrasonic1_value'], None),
                'ultrasonic2_cm': self._safe_number(self.latest['ultrasonic2_value'], None),
            },
            'imu': {
                'accel': {
                    'x': self._safe_number(self.latest.get('imu_accel_x', 0.0), 0.0),
                    'y': self._safe_number(self.latest.get('imu_accel_y', 0.0), 0.0),
                    'z': self._safe_number(self.latest.get('imu_accel_z', 0.0), 0.0),
                },
                'gyro': {
                    'x': self._safe_number(self.latest.get('imu_gyro_x', 0.0), 0.0),
                    'y': self._safe_number(self.latest.get('imu_gyro_y', 0.0), 0.0),
                    'z': self._safe_number(self.latest.get('imu_gyro_z', 0.0), 0.0),
                },
                'mag': {
                    'x': self._safe_number(self.latest.get('imu_mag_x', 0.0), 0.0),
                    'y': self._safe_number(self.latest.get('imu_mag_y', 0.0), 0.0),
                    'z': self._safe_number(self.latest.get('imu_mag_z', 0.0), 0.0),
                },
                'temp_c': self._safe_number(self.latest['imu_temp_value'], None),
            },
            'robot': {
                'mode': str(self.latest['robot_mode']) if self.latest['robot_mode'] != 'n/a' else 'stand',
                'move': str(self.latest['robot_move']) if self.latest['robot_move'] != 'n/a' else 'stand',
            },
            'camera': {
                'fps': self._safe_number(self.camera_fps, 0.0),
                'detections': str(self.latest['detections']) if self.latest['detections'] != 'n/a' else 'none',
                'stream_url': str(self.stream_url),
            },
            'system': {
                'bridge': 'rosbridge',
                'battery_pct': None,
                'uptime_s': int((self.get_clock().now() - self.start_time).nanoseconds / 1e9),
            },
        }

        return json.dumps(payload, separators=(',', ':'))

    def _build_status_text(self) -> str:
        return (
            f"joy_cmd[{self.latest['twist']}] | "
            f"joy_raw[{self.latest['joy_raw']}] | "
            f"sensors[temp1={self.latest['temperature']} hum1={self.latest['humidity']} "
            f"temp2={self.latest['temperature2']} hum2={self.latest['humidity2']} "
            f"u1={self.latest['ultrasonic1']} u2={self.latest['ultrasonic2']}] | "
            f"imu[accel={self.latest['imu_accel']} gyro={self.latest['imu_gyro']} "
            f"mag={self.latest['imu_mag']} temp={self.latest['imu_temp']}] | "
            f"robot_mode[{self.latest['robot_mode']}] | "
            f"robot_move[{self.latest['robot_move']}] | "
            f"camera[fps={self.camera_fps:.1f} detections={self.latest['detections']}] | "
            f"stream[{self.stream_url}]"
        )

    def _update_camera_fps(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_fps_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        frames = self.camera_frame_count - self.last_camera_frame_count
        self.camera_fps = frames / dt
        self.last_camera_frame_count = self.camera_frame_count
        self.last_fps_time = now

    def publish_status(self) -> None:
        self._update_camera_fps()

        report = String()
        report.data = self._build_status_text()
        self.status_pub.publish(report)

        report_json = String()
        report_json.data = self._build_status_json()
        self.status_json_pub.publish(report_json)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
