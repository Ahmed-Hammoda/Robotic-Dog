from launch import LaunchDescription

from robodog_bringup.joystick_launch_common import build_joystick_stack


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(build_joystick_stack('laptop_usb_joystick.yaml'))
