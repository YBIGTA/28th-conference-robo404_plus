#!/usr/bin/env python3
"""Run the auto-labelling recorder against an already-running Gazebo sim.

Bring up the sim + a classical autopilot separately, e.g.:
    ros2 launch robo_description gazebo.launch.py
    ros2 run follower follower_node          # the expert driving /cmd_vel
then:
    ros2 launch yolo_dataset record.launch.py output_dir:=runs/track1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("yolo_dataset")
    default_cfg = os.path.join(pkg_share, "config", "objects.yaml")

    output_dir = LaunchConfiguration("output_dir")
    objects_config = LaunchConfiguration("objects_config")
    record_rate = LaunchConfiguration("record_rate")

    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="robo404_dataset"),
        DeclareLaunchArgument("objects_config", default_value=default_cfg),
        DeclareLaunchArgument("record_rate", default_value="5.0"),
        Node(
            package="yolo_dataset",
            executable="recorder_node",
            output="screen",
            parameters=[{
                "output_dir": output_dir,
                "objects_config": objects_config,
                "record_rate": record_rate,
            }],
        ),
    ])
