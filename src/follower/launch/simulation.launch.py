import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Get share directory of the follower package
    pkg_share = get_package_share_directory('follower')
    
    # Declare launch configurations
    world_name = LaunchConfiguration('world_name')
    
    # Get the share directory of the robo_description package
    robo_description_share = get_package_share_directory('robo_description')
    
    # Include the Gazebo launch description from robo_description which also spawns the car
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robo_description_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': PathJoinSubstitution([pkg_share, world_name])}.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='perception.world',
            description='Name of the world file to load (e.g., perception.world or realistic_perception.world)'
        ),
        gazebo_launch
    ])
