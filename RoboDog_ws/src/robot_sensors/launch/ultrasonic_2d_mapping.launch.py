from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_ultrasonic1_tf',
            arguments=['0.12', '0.06', '0.10', '0.0', '0.0', '0.30', 'base_link', 'ultrasonic1_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_ultrasonic2_tf',
            arguments=['0.12', '-0.06', '0.10', '0.0', '0.0', '-0.30', 'base_link', 'ultrasonic2_link'],
        ),
        Node(
            package='robot_sensors',
            executable='ultrasonic_node',
            name='ultrasonic_node',
        ),
        Node(
            package='robot_sensors',
            executable='ultrasonic_range_bridge_node',
            name='ultrasonic_range_bridge_node',
        ),
        Node(
            package='robot_sensors',
            executable='imu_simulator_node',
            name='imu_simulator_node',
            parameters=[{
                'publish_frequency': 20.0,
                'topic': '/imu/data',
            }],
        ),
        Node(
            package='robot_sensors',
            executable='ultrasonic_occupancy_mapper_node',
            name='ultrasonic_occupancy_mapper_node',
            parameters=[{
                'frame_id': 'map',
                'resolution': 0.05,
                'width': 200,
                'height': 200,
                'publish_frequency': 5.0,
                'imu_topic': '/imu/data',
            }],
        ),
    ])
