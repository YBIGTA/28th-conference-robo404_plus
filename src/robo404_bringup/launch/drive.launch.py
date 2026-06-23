from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_follower_debug_image = LaunchConfiguration("publish_follower_debug_image")
    publish_follower_mask_image = LaunchConfiguration("publish_follower_mask_image")
    show_follower_debug_window = LaunchConfiguration("show_follower_debug_window")
    enable_follower_debug_stream = LaunchConfiguration("enable_follower_debug_stream")
    follower_debug_stream_ip = LaunchConfiguration("follower_debug_stream_ip")
    follower_debug_stream_port = LaunchConfiguration("follower_debug_stream_port")
    follower_debug_stream_width = LaunchConfiguration("follower_debug_stream_width")
    follower_debug_stream_height = LaunchConfiguration("follower_debug_stream_height")
    follower_debug_stream_framerate = LaunchConfiguration(
        "follower_debug_stream_framerate"
    )
    follower_debug_stream_bitrate = LaunchConfiguration(
        "follower_debug_stream_bitrate"
    )
    follower_debug_use_hw_encoder = LaunchConfiguration(
        "follower_debug_use_hw_encoder"
    )
    follower_debug_draw_fps = LaunchConfiguration("follower_debug_draw_fps")

    return LaunchDescription(
        [
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
            Node(
                package="follower",
                executable="follower_node",
                name="follower_node",
                output="screen",
                parameters=[
                    {
                        "publish_debug_image": ParameterValue(
                            publish_follower_debug_image, value_type=bool
                        ),
                        "publish_mask_image": ParameterValue(
                            publish_follower_mask_image, value_type=bool
                        ),
                        "show_debug_window": ParameterValue(
                            show_follower_debug_window, value_type=bool
                        ),
                        "enable_udp_stream": ParameterValue(
                            enable_follower_debug_stream, value_type=bool
                        ),
                        "stream_host": follower_debug_stream_ip,
                        "stream_port": ParameterValue(
                            follower_debug_stream_port, value_type=int
                        ),
                        "stream_width": ParameterValue(
                            follower_debug_stream_width, value_type=int
                        ),
                        "stream_height": ParameterValue(
                            follower_debug_stream_height, value_type=int
                        ),
                        "stream_framerate": ParameterValue(
                            follower_debug_stream_framerate, value_type=int
                        ),
                        "stream_bitrate": ParameterValue(
                            follower_debug_stream_bitrate, value_type=int
                        ),
                        "use_hw_encoder": ParameterValue(
                            follower_debug_use_hw_encoder, value_type=bool
                        ),
                        "draw_fps": ParameterValue(
                            follower_debug_draw_fps, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="decision",
                executable="decision_node",
                name="decision_node",
                output="screen",
            ),
        ]
    )
