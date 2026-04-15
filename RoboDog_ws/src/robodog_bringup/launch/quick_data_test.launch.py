from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory('robodog_bringup'))
    vision_share = Path(get_package_share_directory('robodog_vision'))

    joystick_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / 'launch' / 'rpi_bluetooth_joystick.launch.py'))
    )

    sensors = [
        Node(package='robot_sensors', executable='aht10_node', name='aht10_node'),
        Node(package='robot_sensors', executable='mpu9250_node', name='mpu9250_node'),
        Node(package='robot_sensors', executable='ultrasonic_node', name='ultrasonic_node'),
    ]

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vision_share / 'launch' / 'camera_detection.launch.py')),
        launch_arguments={
            'fps': '25',
            'capture_backend': 'rpicam_vid',
            'target_size': '320',
            'process_every_n': '2',
            'show_window': 'false',
        }.items(),
    )

    return LaunchDescription([
        joystick_stack,
        *sensors,
        vision,
    ])
