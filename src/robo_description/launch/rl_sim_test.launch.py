#!/usr/bin/env python3
"""
Full robo404+ pipeline in Gazebo, but driven by the RL (SAC) line follower
instead of the classic PID follower_node.

Brings up:
  1. Gazebo server + client with the traffic_light_track world
  2. robot_state_publisher (URDF) + spawn_entity
  3. rl_follower_node   (ONNX SAC policy -> /cmd_vel_line, /path_state)
  4. sim_yolo_node      (top camera -> /yolo/detections)
  5. traffic_light_node (-> /traffic_light_state)
  6. decision_node      (gates /cmd_vel_line on the traffic light -> /cmd_vel)

Because decision_node consumes /cmd_vel_line + /path_state + /traffic_light_state
exactly like it does for the classic follower, the RL policy is a drop-in
replacement and gets the SAME red-light / stop behaviour for free.

Usage:
  ros2 launch robo_description rl_sim_test.launch.py
  # then:
  ros2 service call /start_follower std_srvs/srv/Empty   # enable RL policy output
  ros2 service call /start_driving  std_srvs/srv/Empty   # enable decision gating

Tip: max_linear_speed defaults to 0.20 here for clear sim visualisation;
the real-robot default in the node is 0.05.
"""

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
    rviz_config = os.path.join(pkg_share, "rviz", "sim_view.rviz")
    start_x, start_y, start_yaw = _start_pose()
    default_yolo_model = os.path.join(
        pkg_share, "models_yolo", "best_traffic_nano_yolo.pt"
    )
    default_rl_model = os.path.join(
        pkg_share, "..", "..", "..", "..", "models", "sac_robo404.onnx"
    )
    # Prefer the in-source models/ path if resolvable, else the share fallback.
    src_rl_model = "/home/jiucai/28th-conference-robo404_plus/models/sac_robo404.onnx"
    if os.path.exists(src_rl_model):
        default_rl_model = src_rl_model

    models_dir = os.path.join(pkg_share, "models")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    rl_model_path = LaunchConfiguration("rl_model_path")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    auto_demo = LaunchConfiguration("auto_demo")
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

    gazebo_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    if models_dir not in gazebo_model_path:
        os.environ["GAZEBO_MODEL_PATH"] = models_dir + ":" + gazebo_model_path

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

    # ---- RL line follower (drop-in for follower_node) ----
    rl_follower_node = Node(
        package="follower",
        executable="rl_follower_node",
        name="rl_follower_node",
        output="screen",
        parameters=[
            {
                "model_path": rl_model_path,
                # Gazebo camera publishes RELIABLE; sim needs this.
                "image_reliability": "reliable",
                # Faster than the 0.05 real-robot cap, for visualisation.
                "max_linear_speed": ParameterValue(
                    max_linear_speed, value_type=float
                ),
                "start_enabled": False,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    sim_yolo_node = Node(
        package="robo_description",
        executable="sim_yolo_node.py",
        name="sim_yolo_node",
        output="screen",
        parameters=[
            {
                "model_path": yolo_model_path,
                "image_topic": "/camera/rgb/image_raw",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    traffic_light_node = Node(
        package="traffic_light",
        executable="traffic_light_node",
        name="traffic_light_node",
        output="screen",
        parameters=[
            {
                "simulation_mode": True,
                # Gazebo top camera publishes RELIABLE; without this the node
                # receives no frames and never publishes /traffic_light_state.
                "image_reliability": "reliable",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    demo_orchestrator = Node(
        package="robo_description",
        executable="demo_orchestrator.py",
        name="demo_orchestrator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            # Wait this long after the car stops at a red before turning it
            # green, so the stop is clearly visible in the demo.
            "min_stop_sec": 3.0,
        }],
        condition=IfCondition(auto_demo),
    )

    decision_node = Node(
        package="decision",
        executable="decision_node",
        name="decision_node",
        output="screen",
        parameters=[
            {
                "start_enabled": False,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("yolo_model_path", default_value=default_yolo_model),
        DeclareLaunchArgument("rl_model_path", default_value=default_rl_model),
        DeclareLaunchArgument("max_linear_speed", default_value="0.35"),
        DeclareLaunchArgument("auto_demo", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value=start_x),
        DeclareLaunchArgument("y_pose", default_value=start_y),
        DeclareLaunchArgument("z_pose", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value=start_yaw),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        rl_follower_node,
        sim_yolo_node,
        traffic_light_node,
        decision_node,
        rviz_node,
        demo_orchestrator,
    ])
