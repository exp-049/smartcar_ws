from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim = PathJoinSubstitution([FindPackageShare('smartcar_gazebo'), 'launch', 'sim.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='smartcar_navigation.sdf'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={
            'headless': LaunchConfiguration('headless'), 'use_rviz': LaunchConfiguration('use_rviz'), 'world': LaunchConfiguration('world')}.items()),
    ])
