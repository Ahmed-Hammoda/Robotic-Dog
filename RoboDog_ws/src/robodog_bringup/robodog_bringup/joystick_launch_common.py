from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _config_path(filename: str) -> str:
    bringup_share = Path(get_package_share_directory('robodog_bringup'))
    return str(bringup_share / 'config' / filename)


def build_joystick_stack(config_file: str):
    return [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[_config_path(config_file)],
        ),
        Node(
            package='robodog_joystick',
            executable='joystick_node',
            name='joystick_node',
            parameters=[_config_path(config_file)],
        ),
        Node(
            package='robodog_control',
            executable='manual_controller_node',
            name='manual_controller_node',
            parameters=[_config_path(config_file)],
        ),
        Node(package='robodog_stm32_bridge', executable='stm32_bridge_node', name='stm32_bridge_node'),
        Node(package='robodog_interface', executable='monitor_node', name='monitor_node'),
    ]