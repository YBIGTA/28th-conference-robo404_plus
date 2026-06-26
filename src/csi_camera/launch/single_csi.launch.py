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
    sensor_id = LaunchConfiguration("sensor_id")
    flip_method = LaunchConfiguration("flip_method")
    frame_id = LaunchConfiguration("frame_id")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("sensor_id", default_value="0"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("capture_width", default_value="1280"),
            DeclareLaunchArgument("capture_height", default_value="720"),
            DeclareLaunchArgument("output_width", default_value="960"),
            DeclareLaunchArgument("output_height", default_value="540"),
            DeclareLaunchArgument("framerate", default_value="30"),
            DeclareLaunchArgument("flip_method", default_value="0"),
            DeclareLaunchArgument("frame_id", default_value="camera"),
            Node(
                package="csi_camera",
                executable="csi_camera_node",
                name="camera",
                output="screen",
                parameters=[
                    _camera_parameters(
                        sensor_id,
                        flip_method,
                        frame_id,
                    )
                ],
                remappings=[("image_raw", image_topic)],
            ),
        ]
    )
