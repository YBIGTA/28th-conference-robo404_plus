from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _camera_parameters(sensor_id, flip_method, frame_id):
    return {
        "sensor_id": ParameterValue(sensor_id, value_type=int),
        "capture_width": ParameterValue(
            LaunchConfiguration("capture_width"), value_type=int
        ),
        "capture_height": ParameterValue(
            LaunchConfiguration("capture_height"), value_type=int
        ),
        "output_width": ParameterValue(
            LaunchConfiguration("output_width"), value_type=int
        ),
        "output_height": ParameterValue(
            LaunchConfiguration("output_height"), value_type=int
        ),
        "framerate": ParameterValue(LaunchConfiguration("framerate"), value_type=int),
        "flip_method": ParameterValue(flip_method, value_type=int),
        "frame_id": ParameterValue(frame_id, value_type=str),
    }


def generate_launch_description():
    bottom_sensor_id = LaunchConfiguration("bottom_sensor_id")
    top_sensor_id = LaunchConfiguration("top_sensor_id")
    bottom_flip_method = LaunchConfiguration("bottom_flip_method")
    top_flip_method = LaunchConfiguration("top_flip_method")
    bottom_frame_id = LaunchConfiguration("bottom_frame_id")
    top_frame_id = LaunchConfiguration("top_frame_id")

    return LaunchDescription(
        [
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
            Node(
                package="csi_camera",
                executable="csi_camera_node",
                name="bottom_camera",
                output="screen",
                parameters=[
                    _camera_parameters(
                        bottom_sensor_id,
                        bottom_flip_method,
                        bottom_frame_id,
                    )
                ],
                remappings=[("image_raw", "/camera/image_raw")],
            ),
            Node(
                package="csi_camera",
                executable="csi_camera_node",
                name="top_camera",
                output="screen",
                parameters=[
                    _camera_parameters(
                        top_sensor_id,
                        top_flip_method,
                        top_frame_id,
                    )
                ],
                remappings=[("image_raw", "/camera/rgb/image_raw")],
            ),
        ]
    )
