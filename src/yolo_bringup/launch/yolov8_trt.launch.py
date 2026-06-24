from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    engine_path = LaunchConfiguration("engine_path")
    input_image_topic = LaunchConfiguration("input_image_topic")
    namespace = LaunchConfiguration("namespace")
    use_debug = LaunchConfiguration("use_debug")
    image_reliability = LaunchConfiguration("image_reliability")
    yolo_encoding = LaunchConfiguration("yolo_encoding")
    enable = LaunchConfiguration("enable")
    threshold = LaunchConfiguration("threshold")
    iou = LaunchConfiguration("iou")
    imgsz_height = LaunchConfiguration("imgsz_height")
    imgsz_width = LaunchConfiguration("imgsz_width")
    max_det = LaunchConfiguration("max_det")
    num_labels = LaunchConfiguration("num_labels")
    class_names = LaunchConfiguration("class_names")
    publish_dbg_image = LaunchConfiguration("publish_dbg_image")
    debug_stream_ip = LaunchConfiguration("debug_stream_ip")
    debug_stream_port = LaunchConfiguration("debug_stream_port")
    debug_stream_width = LaunchConfiguration("debug_stream_width")
    debug_stream_height = LaunchConfiguration("debug_stream_height")
    debug_stream_framerate = LaunchConfiguration("debug_stream_framerate")
    debug_stream_bitrate = LaunchConfiguration("debug_stream_bitrate")
    debug_use_hw_encoder = LaunchConfiguration("debug_use_hw_encoder")
    debug_draw_fps = LaunchConfiguration("debug_draw_fps")

    yolo_node = Node(
        package="yolo_jetson",
        executable="yolo_node",
        name="yolo_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "engine_path": engine_path,
                "yolo_encoding": yolo_encoding,
                "enable": ParameterValue(enable, value_type=bool),
                "threshold": ParameterValue(threshold, value_type=float),
                "iou": ParameterValue(iou, value_type=float),
                "imgsz_height": ParameterValue(imgsz_height, value_type=int),
                "imgsz_width": ParameterValue(imgsz_width, value_type=int),
                "max_det": ParameterValue(max_det, value_type=int),
                "num_labels": ParameterValue(num_labels, value_type=int),
                "class_names": class_names,
                "image_reliability": ParameterValue(
                    image_reliability, value_type=int
                ),
            }
        ],
        remappings=[("image_raw", input_image_topic)],
    )

    debug_node = Node(
        package="yolo_debug",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        parameters=[
            {
                "image_reliability": ParameterValue(
                    image_reliability, value_type=int
                ),
                "publish_dbg_image": ParameterValue(
                    publish_dbg_image, value_type=bool
                ),
                "enable_udp_stream": True,
                "stream_host": debug_stream_ip,
                "stream_port": ParameterValue(debug_stream_port, value_type=int),
                "stream_width": ParameterValue(debug_stream_width, value_type=int),
                "stream_height": ParameterValue(debug_stream_height, value_type=int),
                "stream_framerate": ParameterValue(
                    debug_stream_framerate, value_type=int
                ),
                "stream_bitrate": ParameterValue(debug_stream_bitrate, value_type=int),
                "use_hw_encoder": ParameterValue(
                    debug_use_hw_encoder, value_type=bool
                ),
                "draw_fps": ParameterValue(debug_draw_fps, value_type=bool),
            }
        ],
        remappings=[("image_raw", input_image_topic)],
        condition=IfCondition(use_debug),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "engine_path",
                default_value="",
                description="Path to the TensorRT .engine file",
            ),
            DeclareLaunchArgument(
                "input_image_topic",
                default_value="/camera/rgb/image_raw",
                description="Input image topic",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="yolo",
                description="Namespace for YOLO nodes",
            ),
            DeclareLaunchArgument(
                "use_debug",
                default_value="False",
                description="Whether to run UDP debug streaming",
            ),
            DeclareLaunchArgument(
                "publish_dbg_image",
                default_value="False",
                description="Whether debug_node also publishes /yolo/dbg_image",
            ),
            DeclareLaunchArgument(
                "debug_stream_ip",
                default_value="127.0.0.1",
                description="UDP debug stream receiver IP address",
            ),
            DeclareLaunchArgument(
                "debug_stream_port",
                default_value="5000",
                description="UDP debug stream receiver port",
            ),
            DeclareLaunchArgument(
                "debug_stream_width",
                default_value="960",
                description="UDP debug stream output width",
            ),
            DeclareLaunchArgument(
                "debug_stream_height",
                default_value="540",
                description="UDP debug stream output height",
            ),
            DeclareLaunchArgument(
                "debug_stream_framerate",
                default_value="30",
                description="UDP debug stream framerate",
            ),
            DeclareLaunchArgument(
                "debug_stream_bitrate",
                default_value="4000000",
                description="UDP debug stream H.264 bitrate in bits per second",
            ),
            DeclareLaunchArgument(
                "debug_use_hw_encoder",
                default_value="True",
                description="Whether to try Jetson hardware H.264 encoder first",
            ),
            DeclareLaunchArgument(
                "debug_draw_fps",
                default_value="True",
                description="Whether to draw callback FPS on the debug stream",
            ),
            DeclareLaunchArgument(
                "image_reliability",
                default_value="2",
                choices=["0", "1", "2"],
                description="Input image QoS reliability: 0=default, 1=reliable, 2=best effort",
            ),
            DeclareLaunchArgument(
                "yolo_encoding",
                default_value="bgr8",
                description="cv_bridge encoding for input images",
            ),
            DeclareLaunchArgument(
                "enable",
                default_value="True",
                description="Whether YOLO starts enabled",
            ),
            DeclareLaunchArgument(
                "threshold",
                default_value="0.5",
                description="Detection confidence threshold",
            ),
            DeclareLaunchArgument(
                "iou",
                default_value="0.5",
                description="NMS IoU threshold",
            ),
            DeclareLaunchArgument(
                "imgsz_height",
                default_value="640",
                description="Inference input height",
            ),
            DeclareLaunchArgument(
                "imgsz_width",
                default_value="640",
                description="Inference input width",
            ),
            DeclareLaunchArgument(
                "max_det",
                default_value="100",
                description="Maximum detections per frame",
            ),
            DeclareLaunchArgument(
                "num_labels",
                default_value="80",
                description="Number of model classes",
            ),
            DeclareLaunchArgument(
                "class_names",
                default_value="[]",
                description="Custom class names list",
            ),
            yolo_node,
            debug_node,
        ]
    )
