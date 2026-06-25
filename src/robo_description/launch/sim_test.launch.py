#!/usr/bin/env python3
"""
Launch the full robo404+ software pipeline in Gazebo simulation.

Brings up:
  1. Gazebo server + client with the traffic_light_track world
  2. robot_state_publisher (URDF)
  3. spawn_entity (place robot on the track)
  4. follower_node (line tracking from bottom camera)
  5. traffic_light_node (colour classification from top camera)
  6. decision_node (arbitrates cmd_vel)

NOTE: yolo_jetson is NOT launched (requires CUDA/TensorRT on Jetson).
      The traffic_light_node will work because it subscribes to
      /yolo/detections (which won't arrive), but also directly reads
      /camera/rgb/image_raw for HSV fallback if a detection bbox is
      provided manually.

Usage:
  ros2 launch robo_description sim_test.launch.py
  # Then start the robot:
  ros2 service call /start_follower std_srvs/srv/Empty
  ros2 service call /start_driving std_srvs/srv/Empty
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
    """Spawn pose derived from the single source of truth (track_path).

    Falls back to a sensible default if the helper can't be imported.
    """
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
    default_model = os.path.join(
        pkg_share, "models_yolo", "best_traffic_nano_yolo.pt"
    )

    # Our custom models directory so Gazebo can find track_ground, traffic lights
    models_dir = os.path.join(pkg_share, "models")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    model_path = LaunchConfiguration("model_path")
    auto_demo = LaunchConfiguration("auto_demo")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw = LaunchConfiguration("yaw")
    linear_speed = LaunchConfiguration("linear_speed")
    kp = LaunchConfiguration("kp")
    kd = LaunchConfiguration("kd")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]), value_type=str
        ),
        "use_sim_time": use_sim_time,
    }

    # Prepend our models dir to GAZEBO_MODEL_PATH so Gazebo finds our custom models
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

    # ---- Application nodes ----

    follower_node = Node(
        package="follower",
        executable="follower_node",
        name="follower_node",
        output="screen",
        parameters=[
            {
                "publish_debug_image": True,
                "show_debug_window": False,
                "linear_speed": linear_speed,
                "kp": kp,
                "kd": kd,
            }
        ],
    )

    # Desktop-sim YOLO inference (finetuned traffic-light model, CPU/ultralytics).
    # Publishes /yolo/detections (consumed by traffic_light_node) and an
    # annotated /yolo/dbg_image for RViz.
    sim_yolo_node = Node(
        package="robo_description",
        executable="sim_yolo_node.py",
        name="sim_yolo_node",
        output="screen",
        parameters=[
            {
                "model_path": model_path,
                "image_topic": "/camera/rgb/image_raw",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # simulation_mode is ON: use robust HSV fallback on the top camera feed
    # to reliably detect visual spheres in the Gazebo simulation.
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

    # Hands-off demo: auto /start_driving, then switch each red light to green
    # once the car stops in front of it, so it completes the course on its own.
    demo_orchestrator = Node(
        package="robo_description",
        executable="demo_orchestrator.py",
        name="demo_orchestrator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
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
        DeclareLaunchArgument("model_path", default_value=default_model),
        DeclareLaunchArgument("auto_demo", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value=start_x),
        DeclareLaunchArgument("y_pose", default_value=start_y),
        DeclareLaunchArgument("z_pose", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value=start_yaw),
        DeclareLaunchArgument("linear_speed", default_value="0.20"),
        DeclareLaunchArgument("kp", default_value="0.035"),
        DeclareLaunchArgument("kd", default_value="0.005"),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        follower_node,
        sim_yolo_node,
        traffic_light_node,
        decision_node,
        rviz_node,
        demo_orchestrator,
    ])
