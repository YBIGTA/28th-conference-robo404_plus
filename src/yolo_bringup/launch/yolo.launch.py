import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file = os.path.join(
        get_package_share_directory("yolo_bringup"),
        "launch",
        "yolov8_trt.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("engine_path", default_value=""),
            DeclareLaunchArgument(
                "input_image_topic", default_value="/camera/rgb/image_raw"
            ),
            DeclareLaunchArgument("namespace", default_value="yolo"),
            DeclareLaunchArgument("use_debug", default_value="False"),
            DeclareLaunchArgument("image_reliability", default_value="2"),
            DeclareLaunchArgument("yolo_encoding", default_value="bgr8"),
            DeclareLaunchArgument("enable", default_value="True"),
            DeclareLaunchArgument("threshold", default_value="0.5"),
            DeclareLaunchArgument("iou", default_value="0.5"),
            DeclareLaunchArgument("imgsz_height", default_value="640"),
            DeclareLaunchArgument("imgsz_width", default_value="640"),
            DeclareLaunchArgument("max_det", default_value="100"),
            DeclareLaunchArgument("num_labels", default_value="80"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "engine_path": LaunchConfiguration("engine_path", default=""),
                    "input_image_topic": LaunchConfiguration(
                        "input_image_topic", default="/camera/rgb/image_raw"
                    ),
                    "namespace": LaunchConfiguration("namespace", default="yolo"),
                    "use_debug": LaunchConfiguration("use_debug", default="False"),
                    "image_reliability": LaunchConfiguration(
                        "image_reliability", default="2"
                    ),
                    "yolo_encoding": LaunchConfiguration("yolo_encoding", default="bgr8"),
                    "enable": LaunchConfiguration("enable", default="True"),
                    "threshold": LaunchConfiguration("threshold", default="0.5"),
                    "iou": LaunchConfiguration("iou", default="0.5"),
                    "imgsz_height": LaunchConfiguration("imgsz_height", default="640"),
                    "imgsz_width": LaunchConfiguration("imgsz_width", default="640"),
                    "max_det": LaunchConfiguration("max_det", default="100"),
                    "num_labels": LaunchConfiguration("num_labels", default="80"),
                }.items(),
            )
        ]
    )
