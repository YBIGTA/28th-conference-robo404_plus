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
                "image_reliability": ParameterValue(
                    image_reliability, value_type=int
                ),
            }
        ],
        remappings=[("image_raw", input_image_topic)],
    )

    debug_node = Node(
        package="yolo_ros",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        parameters=[
            {
                "image_reliability": ParameterValue(
                    image_reliability, value_type=int
                )
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
                description="Whether to run the debug visualization node",
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
            yolo_node,
            debug_node,
        ]
    )
