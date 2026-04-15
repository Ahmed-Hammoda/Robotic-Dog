from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for camera and NCNN detection nodes."""
    
    # Launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',
        description='Image width in pixels'
    )
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='720',
        description='Image height in pixels'
    )
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='20',
        description='Frames per second'
    )
    backend_arg = DeclareLaunchArgument(
        'capture_backend',
        default_value='rpicam_vid',
        description='Camera backend: rpicam_vid or rpicam_still'
    )
    detection_arg = DeclareLaunchArgument(
        'enable_detection',
        default_value='false',
        description='Enable object detection'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='Show OpenCV preview window from detection node'
    )
    prob_arg = DeclareLaunchArgument(
        'prob_threshold',
        default_value='0.25',
        description='Detection confidence threshold'
    )
    nms_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS IoU threshold'
    )
    target_size_arg = DeclareLaunchArgument(
        'target_size',
        default_value='416',
        description='NCNN detector input size'
    )
    process_every_arg = DeclareLaunchArgument(
        'process_every_n',
        default_value='1',
        description='Run NCNN every N frames (higher N gives higher display FPS)'
    )
    model_param_arg = DeclareLaunchArgument(
        'model_param_path',
        default_value='/home/robodog/RoboDog_ws/src/robodog_vision/models/yolov8n.ncnn.param',
        description='Path to NCNN .param file'
    )
    model_bin_arg = DeclareLaunchArgument(
        'model_bin_path',
        default_value='/home/robodog/RoboDog_ws/src/robodog_vision/models/yolov8n.ncnn.bin',
        description='Path to NCNN .bin file'
    )
    
    # Camera detection node
    camera_node = Node(
        package='robodog_vision',
        executable='camera_detection_node',
        name='camera_detection',
        output='screen',
        parameters=[
            {
                'image_width': LaunchConfiguration('width'),
                'image_height': LaunchConfiguration('height'),
                'frame_rate': LaunchConfiguration('fps'),
                'capture_backend': LaunchConfiguration('capture_backend'),
                'enable_detection': LaunchConfiguration('enable_detection'),
            }
        ],
    )

    ncnn_node = Node(
        package='robodog_vision',
        executable='ncnn_detection_node',
        name='ncnn_detection',
        output='screen',
        parameters=[
            {
                'input_topic': '/robodog/camera/image_raw',
                'output_topic': '/robodog/camera/detections/image',
                'detections_topic': '/robodog/camera/detections/text',
                'show_window': LaunchConfiguration('show_window'),
                'target_size': LaunchConfiguration('target_size'),
                'prob_threshold': LaunchConfiguration('prob_threshold'),
                'nms_threshold': LaunchConfiguration('nms_threshold'),
                'process_every_n': LaunchConfiguration('process_every_n'),
                'model_param_path': LaunchConfiguration('model_param_path'),
                'model_bin_path': LaunchConfiguration('model_bin_path'),
            }
        ],
    )
    
    ld = LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        backend_arg,
        detection_arg,
        show_window_arg,
        prob_arg,
        nms_arg,
        target_size_arg,
        process_every_arg,
        model_param_arg,
        model_bin_arg,
        camera_node,
        ncnn_node,
    ])
    
    return ld
