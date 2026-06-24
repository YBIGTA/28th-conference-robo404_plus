#!/usr/bin/env python3
"""View the robo404 model in RViz2 (no Gazebo) with joint sliders."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robo_description")
    xacro_file = os.path.join(pkg_share, "urdf", "robo404.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "config", "robo404.rviz")

    gui = LaunchConfiguration("gui")
    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Use joint_state_publisher_gui sliders"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
