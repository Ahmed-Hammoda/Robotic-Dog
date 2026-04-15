from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bringup_share = Path(get_package_share_directory('robodog_bringup'))

    rosbridge_port = int(LaunchConfiguration('rosbridge_port').perform(context))

    cleanup = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'pkill -f rosbridge_websocket || true; '
            'pkill -f camera_detection_node || true; '
            'pkill -f ncnn_detection_node || true; '
            'pkill -f rpicam-vid || true; '
            'pkill -f rpicam-still || true; '
            'pkill -f libcamera-vid || true; '
            'pkill -f libcamera-still || true; '
            'rm -f /tmp/robodog_camera/stream.mjpeg || true; '
            'rm -rf /dev/shm/fastdds* || true; '
            'sleep 1',
        ],
        output='screen',
    )

    quick_data_test = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / 'launch' / 'quick_data_test.launch.py'))
    )

    actions = [cleanup, quick_data_test]

    try:
        get_package_share_directory('rosbridge_server')
        rosbridge = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': rosbridge_port, 'address': '0.0.0.0'}],
        )
        actions.append(rosbridge)
        print(f"[web_interface.launch] WebSocket server running on port {rosbridge_port}. Connect and subscribe to /robodog/camera/detections/image for video stream.")
    except PackageNotFoundError:
        print("[web_interface.launch] ERROR: rosbridge_server not found. Install it: sudo apt install ros-kilted-rosbridge-server")

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        OpaqueFunction(function=_launch_setup),
    ])
