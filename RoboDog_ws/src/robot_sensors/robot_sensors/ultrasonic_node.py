import time

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class UltrasonicNode(Node):
    def __init__(self) -> None:
        super().__init__('ultrasonic_node')

        self.trig1 = 17
        self.echo1 = 27
        self.trig2 = 22
        self.echo2 = 23
        self.gpio_ok = False

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            for pin in (self.trig1, self.trig2):
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            for pin in (self.echo1, self.echo2):
                GPIO.setup(pin, GPIO.IN)

            self.gpio_ok = True
        except Exception as exc:
            self.get_logger().error(f'GPIO init failed for ultrasonic sensors: {exc}')
            self.get_logger().warn('Ultrasonic node will stay alive and publish NaN values')

        time.sleep(0.05)

        self.pub1 = self.create_publisher(Float32, '/ultrasonic1', 10)
        self.pub2 = self.create_publisher(Float32, '/ultrasonic2', 10)
        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('ultrasonic_node started')

    def _read_distance_cm(self, trig_pin: int, echo_pin: int) -> float:
        if not self.gpio_ok:
            return float('nan')

        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_pin, GPIO.LOW)

        timeout_s = 0.03
        start_wait = time.perf_counter()
        while GPIO.input(echo_pin) == 0:
            if time.perf_counter() - start_wait > timeout_s:
                return float('nan')

        pulse_start = time.perf_counter()
        while GPIO.input(echo_pin) == 1:
            if time.perf_counter() - pulse_start > timeout_s:
                return float('nan')

        pulse_duration = time.perf_counter() - pulse_start
        return (pulse_duration * 34300.0) / 2.0

    def timer_callback(self) -> None:
        d1 = self._read_distance_cm(self.trig1, self.echo1)
        d2 = self._read_distance_cm(self.trig2, self.echo2)

        msg1 = Float32()
        msg1.data = float(d1)
        msg2 = Float32()
        msg2.data = float(d2)
        self.pub1.publish(msg1)
        self.pub2.publish(msg2)

        print(f'ultrasonic1={d1:.2f} cm | ultrasonic2={d2:.2f} cm')

    def destroy_node(self) -> bool:
        if self.gpio_ok:
            GPIO.cleanup()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()