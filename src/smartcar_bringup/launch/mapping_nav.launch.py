from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim = PathJoinSubstitution([FindPackageShare('smartcar_gazebo'), 'launch', 'sim.launch.py'])
    slam = PathJoinSubstitution([FindPackageShare('smartcar_cartographer'), 'launch', 'cartographer.launch.py'])
    nav = PathJoinSubstitution([FindPackageShare('smartcar_navigation'), 'launch', 'navigation.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='smartcar_navigation.sdf'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={'headless': LaunchConfiguration('headless'), 'use_rviz': LaunchConfiguration('use_rviz'), 'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam), launch_arguments={'mode': 'mapping', 'mapping': 'true', 'configuration_basename': 'smartcar_2d.lua'}.items()),
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(nav), launch_arguments={'autostart': 'true', 'use_sim_time': 'true'}.items())]),
    ])
