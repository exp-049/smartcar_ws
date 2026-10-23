from launch import LaunchDescription
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def validate_localization_files(context):
    """Fail before starting Gazebo when the saved localization inputs are absent."""
    map_yaml_value = context.perform_substitution(LaunchConfiguration('map_yaml'))
    state_file_value = context.perform_substitution(LaunchConfiguration('state_file'))
    paths = [('map_yaml', map_yaml_value), ('state_file', state_file_value)]
    missing = [name + (f'={value}' if value else '')
               for name, value in paths
               if not value or not Path(value).is_file()]
    if missing:
        raise RuntimeError(
            'localization_nav requires existing map_yaml and state_file; missing: '
            + ', '.join(missing)
            + '. Run `ros2 run smartcar_tools save_map --name demo` after mapping, '
            'then pass map_yaml:=.../demo.yaml and state_file:=.../demo.pbstream.'
        )
    return []

def generate_launch_description():
    sim = PathJoinSubstitution([FindPackageShare('smartcar_gazebo'), 'launch', 'sim.launch.py'])
    slam = PathJoinSubstitution([FindPackageShare('smartcar_cartographer'), 'launch', 'cartographer.launch.py'])
    nav = PathJoinSubstitution([FindPackageShare('smartcar_navigation'), 'launch', 'navigation.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='smartcar_navigation.sdf'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('state_file', default_value=''),
        OpaqueFunction(function=validate_localization_files),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={'headless': LaunchConfiguration('headless'), 'use_rviz': LaunchConfiguration('use_rviz'), 'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(slam), launch_arguments={'mode': 'localization', 'mapping': 'false', 'configuration_basename': 'smartcar_2d_localization.lua', 'state_file': LaunchConfiguration('state_file')}.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', parameters=[{'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map_yaml')}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map_server', output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]),
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(nav), launch_arguments={'autostart': 'true', 'use_sim_time': 'true'}.items())]),
    ])
