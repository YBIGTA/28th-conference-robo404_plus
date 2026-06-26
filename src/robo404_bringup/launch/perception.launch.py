import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package_name, launch_file_name):
    return os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file_name,
    )


def generate_launch_description():
    csi_launch = _launch_file("csi_camera", "single_csi.launch.py")
    yolo_launch = _launch_file("yolo_bringup", "yolov8_trt.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("engine_path", default_value=""),
            DeclareLaunchArgument("camera_sensor_id", default_value="0"),
            DeclareLaunchArgument("capture_width", default_value="1280"),
            DeclareLaunchArgument("capture_height", default_value="720"),
            DeclareLaunchArgument("output_width", default_value="960"),
            DeclareLaunchArgument("output_height", default_value="540"),
            DeclareLaunchArgument("framerate", default_value="30"),
            DeclareLaunchArgument("camera_flip_method", default_value="0"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera"),
            DeclareLaunchArgument(
                "input_image_topic", default_value="/camera/image_raw"
            ),
            DeclareLaunchArgument("namespace", default_value="yolo"),
            DeclareLaunchArgument("use_debug", default_value="False"),
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
                PythonLaunchDescriptionSource(csi_launch),
                launch_arguments={
                    "sensor_id": LaunchConfiguration("camera_sensor_id"),
                    "image_topic": LaunchConfiguration("input_image_topic"),
                    "capture_width": LaunchConfiguration("capture_width"),
                    "capture_height": LaunchConfiguration("capture_height"),
                    "output_width": LaunchConfiguration("output_width"),
                    "output_height": LaunchConfiguration("output_height"),
                    "framerate": LaunchConfiguration("framerate"),
                    "flip_method": LaunchConfiguration("camera_flip_method"),
                    "frame_id": LaunchConfiguration("camera_frame_id"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yolo_launch),
                launch_arguments={
                    "engine_path": LaunchConfiguration("engine_path"),
                    "input_image_topic": LaunchConfiguration("input_image_topic"),
                    "namespace": LaunchConfiguration("namespace"),
                    "use_debug": LaunchConfiguration("use_debug"),
                    "publish_dbg_image": LaunchConfiguration("publish_dbg_image"),
                    "debug_stream_ip": LaunchConfiguration("debug_stream_ip"),
                    "debug_stream_port": LaunchConfiguration("debug_stream_port"),
                    "debug_stream_width": LaunchConfiguration("debug_stream_width"),
                    "debug_stream_height": LaunchConfiguration("debug_stream_height"),
                    "debug_stream_framerate": LaunchConfiguration(
                        "debug_stream_framerate"
                    ),
                    "debug_stream_bitrate": LaunchConfiguration(
                        "debug_stream_bitrate"
                    ),
                    "debug_use_hw_encoder": LaunchConfiguration(
                        "debug_use_hw_encoder"
                    ),
                    "debug_draw_fps": LaunchConfiguration("debug_draw_fps"),
                }.items(),
            ),
            Node(
                package="traffic_light",
                executable="traffic_light_node",
                name="traffic_light_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("input_image_topic"),
                    }
                ],
            ),
        ]
    )
