from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    vision_share = Path(get_package_share_directory('robodog_vision'))
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(vision_share / 'launch' / 'camera_detection.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_vision')),
        launch_arguments={
            'fps': '25',
            'capture_backend': 'rpicam_vid',
            'target_size': '320',
            'process_every_n': '2',
            'show_window': 'false',
        }.items(),
    )

    visual_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visual_slam')),
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_rgb': True,
            'subscribe_depth': False,
            'subscribe_scan': False,
            'subscribe_imu': True,
            'wait_imu_to_init': True,
            'approx_sync': True,
            'queue_size': 20,
            'Reg/Force3DoF': 'true',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
        }],
        remappings=[
            ('rgb/image', '/robodog/camera/image_raw'),
            ('imu', '/imu/data'),
        ],
    )

    sensor_tf_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            condition=IfCondition(LaunchConfiguration('enable_vision')),
            arguments=['0.14', '0.00', '0.18', '0.0', '0.0', '0.0', 'base_link', 'camera'],
        ),
    ]

    sensors = [
        Node(package='robot_sensors', executable='aht10_node', name='aht10_node'),
        Node(
            package='robot_sensors',
            executable='mpu9250_node',
            name='mpu9250_node',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_frequency': 20.0,
                'publish_tf_on_map_frame': True,
                'use_magnetometer_when_stationary': True,
                'zero_gyro_when_stationary': True,
                'stationary_gyro_threshold_rad_s': 0.015,
                'stationary_accel_tolerance_m_s2': 0.25,
            }],
        ),
    ]

    interface_nodes = [
        Node(package='robodog_control', executable='manual_controller_node', name='manual_controller_node'),
        Node(package='robodog_interface', executable='monitor_node', name='monitor_node'),
        Node(package='robodog_stm32_bridge', executable='stm32_bridge_node', name='stm32_bridge_node'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('enable_vision', default_value='false'),
        DeclareLaunchArgument('enable_visual_slam', default_value='false'),
        *sensors,
        *sensor_tf_nodes,
        *interface_nodes,
        vision,
        visual_slam,
    ])
