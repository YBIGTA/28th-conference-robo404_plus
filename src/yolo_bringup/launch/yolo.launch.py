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
                "input_image_topic", default_value="/camera/image_raw"
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
            DeclareLaunchArgument("publish_dbg_image", default_value="False"),
            DeclareLaunchArgument("debug_stream_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("debug_stream_port", default_value="5000"),
            DeclareLaunchArgument("debug_stream_width", default_value="960"),
            DeclareLaunchArgument("debug_stream_height", default_value="540"),
            DeclareLaunchArgument("debug_stream_framerate", default_value="30"),
            DeclareLaunchArgument("debug_stream_bitrate", default_value="4000000"),
            DeclareLaunchArgument("debug_use_hw_encoder", default_value="True"),
            DeclareLaunchArgument("debug_draw_fps", default_value="True"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "engine_path": LaunchConfiguration("engine_path", default=""),
                    "input_image_topic": LaunchConfiguration(
                        "input_image_topic", default="/camera/image_raw"
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
                    "publish_dbg_image": LaunchConfiguration(
                        "publish_dbg_image", default="False"
                    ),
                    "debug_stream_ip": LaunchConfiguration(
                        "debug_stream_ip", default="127.0.0.1"
                    ),
                    "debug_stream_port": LaunchConfiguration(
                        "debug_stream_port", default="5000"
                    ),
                    "debug_stream_width": LaunchConfiguration(
                        "debug_stream_width", default="960"
                    ),
                    "debug_stream_height": LaunchConfiguration(
                        "debug_stream_height", default="540"
                    ),
                    "debug_stream_framerate": LaunchConfiguration(
                        "debug_stream_framerate", default="30"
                    ),
                    "debug_stream_bitrate": LaunchConfiguration(
                        "debug_stream_bitrate", default="4000000"
                    ),
                    "debug_use_hw_encoder": LaunchConfiguration(
                        "debug_use_hw_encoder", default="True"
                    ),
                    "debug_draw_fps": LaunchConfiguration(
                        "debug_draw_fps", default="True"
                    ),
                }.items(),
            )
        ]
    )
