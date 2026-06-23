from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="follower",
                executable="follower_node",
                name="follower_node",
                output="screen",
            ),
            Node(
                package="decision",
                executable="decision_node",
                name="decision_node",
                output="screen",
            ),
        ]
    )
