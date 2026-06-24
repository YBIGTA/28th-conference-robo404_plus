#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _start_pose():
    import sys
    pkg_share = get_package_share_directory("robo_description")
    sys.path.insert(0, os.path.join(pkg_share, "..", "..", "lib", "robo_description"))
    try:
        from track_path import start_pose
        x, y, yaw = start_pose()
        return f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}"
    except Exception:
        return "-0.6", "0.4", "0.0"

def generate_launch_description():
    pkg_share = get_package_share_directory("robo_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(pkg_share, "urdf", "robo404.urdf.xacro")
    default_world = os.path.join(pkg_share, "worlds", "traffic_light_track.world")
    start_x, start_y, start_yaw = _start_pose()

    # Prepend our models dir to GAZEBO_MODEL_PATH
    models_dir = os.path.join(pkg_share, "models")
    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    if models_dir not in gazebo_model_path:
        os.environ["GAZEBO_MODEL_PATH"] = models_dir + ":" + gazebo_model_path

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        ),
        "use_sim_time": use_sim_time,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robo404",
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-Y", yaw,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="false"),  # Default headless for training speed
        DeclareLaunchArgument("x_pose", default_value="-0.6"),
        DeclareLaunchArgument("y_pose", default_value="0.4"),
        DeclareLaunchArgument("z_pose", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value=start_yaw),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
    ])
