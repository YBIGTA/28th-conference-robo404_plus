import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('follower')
    
    world_name = LaunchConfiguration('world_name')
    
    # 1. Simulation Launch (starts Gazebo and loads track + obstacles + robot model)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'simulation.launch.py')
        ),
        launch_arguments={'world_name': world_name}.items()
    )
    
    # 2. Line Tracker Node (Perception)
    line_tracker_node = Node(
        package='follower',
        executable='line_tracker_node',
        name='line_tracker_node',
        output='screen'
    )
    
    # 3. YOLO Detector Node (Perception)
    yolo_detector_node = Node(
        package='follower',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen'
    )
    
    # 4. Safety Arbiter Node (Control Downstream Consumer)
    safety_arbiter_node = Node(
        package='follower',
        executable='safety_arbiter_node',
        name='safety_arbiter_node',
        output='screen'
    )
    
    # 5. rqt_image_view Node (Visualizer GUI)
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/yolo_detector_node/dbg_image'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='perception.world',
            description='Name of the world file to load (e.g., perception.world or realistic_perception.world)'
        ),
        sim_launch,
        line_tracker_node,
        yolo_detector_node,
        safety_arbiter_node,
        rqt_image_view_node
    ])
