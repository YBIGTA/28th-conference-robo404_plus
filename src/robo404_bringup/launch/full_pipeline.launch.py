import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _own_launch_file(launch_file_name):
    return os.path.join(
        get_package_share_directory("robo404_bringup"),
        "launch",
        launch_file_name,
    )


def generate_launch_description():
    perception_launch = _own_launch_file("perception.launch.py")
    drive_launch = _own_launch_file("drive.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("engine_path", default_value=""),
            DeclareLaunchArgument("bottom_sensor_id", default_value="0"),
            DeclareLaunchArgument("top_sensor_id", default_value="1"),
            DeclareLaunchArgument("capture_width", default_value="1280"),
            DeclareLaunchArgument("capture_height", default_value="720"),
            DeclareLaunchArgument("output_width", default_value="960"),
            DeclareLaunchArgument("output_height", default_value="540"),
            DeclareLaunchArgument("framerate", default_value="30"),
            DeclareLaunchArgument("bottom_flip_method", default_value="0"),
            DeclareLaunchArgument("top_flip_method", default_value="0"),
            DeclareLaunchArgument("bottom_frame_id", default_value="bottom_camera"),
            DeclareLaunchArgument("top_frame_id", default_value="top_camera"),
            DeclareLaunchArgument(
                "input_image_topic", default_value="/camera/rgb/image_raw"
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
            DeclareLaunchArgument(
                "publish_follower_debug_image", default_value="False"
            ),
            DeclareLaunchArgument(
                "publish_follower_mask_image", default_value="False"
            ),
            DeclareLaunchArgument("enable_follower_debug_stream", default_value="False"),
            DeclareLaunchArgument(
                "follower_debug_stream_ip", default_value="127.0.0.1"
            ),
            DeclareLaunchArgument("follower_debug_stream_port", default_value="5001"),
            DeclareLaunchArgument("follower_debug_stream_width", default_value="960"),
            DeclareLaunchArgument("follower_debug_stream_height", default_value="540"),
            DeclareLaunchArgument(
                "follower_debug_stream_framerate", default_value="30"
            ),
            DeclareLaunchArgument(
                "follower_debug_stream_bitrate", default_value="4000000"
            ),
            DeclareLaunchArgument(
                "follower_debug_use_hw_encoder", default_value="True"
            ),
            DeclareLaunchArgument("follower_debug_draw_fps", default_value="True"),
            DeclareLaunchArgument("show_follower_debug_window", default_value="False"),
            DeclareLaunchArgument("use_debug_monitor", default_value="False"),
            DeclareLaunchArgument("debug_monitor_publish_rate_hz", default_value="2.0"),
            DeclareLaunchArgument(
                "debug_monitor_path_state_timeout_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "debug_monitor_cmd_vel_line_timeout_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "debug_monitor_yolo_detections_timeout_sec", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "debug_monitor_traffic_light_state_timeout_sec", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "debug_monitor_decision_state_timeout_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "debug_monitor_cmd_vel_timeout_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "debug_monitor_zero_twist_epsilon", default_value="0.001"
            ),
            DeclareLaunchArgument(
                "debug_monitor_cmd_compare_epsilon", default_value="0.001"
            ),
            DeclareLaunchArgument("num_labels", default_value="80"),
            DeclareLaunchArgument("class_names", default_value="[]"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                launch_arguments={
                    "engine_path": LaunchConfiguration("engine_path"),
                    "bottom_sensor_id": LaunchConfiguration("bottom_sensor_id"),
                    "top_sensor_id": LaunchConfiguration("top_sensor_id"),
                    "capture_width": LaunchConfiguration("capture_width"),
                    "capture_height": LaunchConfiguration("capture_height"),
                    "output_width": LaunchConfiguration("output_width"),
                    "output_height": LaunchConfiguration("output_height"),
                    "framerate": LaunchConfiguration("framerate"),
                    "bottom_flip_method": LaunchConfiguration("bottom_flip_method"),
                    "top_flip_method": LaunchConfiguration("top_flip_method"),
                    "bottom_frame_id": LaunchConfiguration("bottom_frame_id"),
                    "top_frame_id": LaunchConfiguration("top_frame_id"),
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
                    "num_labels": LaunchConfiguration("num_labels"),
                    "class_names": LaunchConfiguration("class_names"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(drive_launch),
                launch_arguments={
                    "publish_follower_debug_image": LaunchConfiguration(
                        "publish_follower_debug_image"
                    ),
                    "publish_follower_mask_image": LaunchConfiguration(
                        "publish_follower_mask_image"
                    ),
                    "enable_follower_debug_stream": LaunchConfiguration(
                        "enable_follower_debug_stream"
                    ),
                    "follower_debug_stream_ip": LaunchConfiguration(
                        "follower_debug_stream_ip"
                    ),
                    "follower_debug_stream_port": LaunchConfiguration(
                        "follower_debug_stream_port"
                    ),
                    "follower_debug_stream_width": LaunchConfiguration(
                        "follower_debug_stream_width"
                    ),
                    "follower_debug_stream_height": LaunchConfiguration(
                        "follower_debug_stream_height"
                    ),
                    "follower_debug_stream_framerate": LaunchConfiguration(
                        "follower_debug_stream_framerate"
                    ),
                    "follower_debug_stream_bitrate": LaunchConfiguration(
                        "follower_debug_stream_bitrate"
                    ),
                    "follower_debug_use_hw_encoder": LaunchConfiguration(
                        "follower_debug_use_hw_encoder"
                    ),
                    "follower_debug_draw_fps": LaunchConfiguration(
                        "follower_debug_draw_fps"
                    ),
                    "show_follower_debug_window": LaunchConfiguration(
                        "show_follower_debug_window"
                    ),
                    "use_debug_monitor": LaunchConfiguration("use_debug_monitor"),
                    "debug_monitor_publish_rate_hz": LaunchConfiguration(
                        "debug_monitor_publish_rate_hz"
                    ),
                    "debug_monitor_path_state_timeout_sec": LaunchConfiguration(
                        "debug_monitor_path_state_timeout_sec"
                    ),
                    "debug_monitor_cmd_vel_line_timeout_sec": LaunchConfiguration(
                        "debug_monitor_cmd_vel_line_timeout_sec"
                    ),
                    "debug_monitor_yolo_detections_timeout_sec": LaunchConfiguration(
                        "debug_monitor_yolo_detections_timeout_sec"
                    ),
                    "debug_monitor_traffic_light_state_timeout_sec": LaunchConfiguration(
                        "debug_monitor_traffic_light_state_timeout_sec"
                    ),
                    "debug_monitor_decision_state_timeout_sec": LaunchConfiguration(
                        "debug_monitor_decision_state_timeout_sec"
                    ),
                    "debug_monitor_cmd_vel_timeout_sec": LaunchConfiguration(
                        "debug_monitor_cmd_vel_timeout_sec"
                    ),
                    "debug_monitor_zero_twist_epsilon": LaunchConfiguration(
                        "debug_monitor_zero_twist_epsilon"
                    ),
                    "debug_monitor_cmd_compare_epsilon": LaunchConfiguration(
                        "debug_monitor_cmd_compare_epsilon"
                    ),
                }.items(),
            ),
        ]
    )
