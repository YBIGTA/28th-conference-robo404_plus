from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_serial_device = LaunchConfiguration("agent_serial_device")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agent_serial_device",
                default_value="/dev/ttyACM0",
            ),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", agent_serial_device],
            ),
        ]
    )
