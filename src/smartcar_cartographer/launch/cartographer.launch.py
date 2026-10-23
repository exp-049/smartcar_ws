"""Start Cartographer 2D and, in mapping mode, its OccupancyGrid converter."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    config_dir = PathJoinSubstitution([FindPackageShare('smartcar_cartographer'), 'config'])
    state = LaunchConfiguration('state_file')
    cartographer_args = ['-configuration_directory', config_dir,
                         '-configuration_basename', LaunchConfiguration('configuration_basename')]
    mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    localization = IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('configuration_basename', default_value='smartcar_2d.lua'),
        DeclareLaunchArgument('state_file', default_value=''),
        DeclareLaunchArgument('mapping', default_value='true'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             condition=mapping, arguments=cartographer_args,
             remappings=[('scan', '/car/scan'), ('imu', '/car/imu/data_raw'), ('odom', '/car/chassis/odom')],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_localization_node',
             condition=localization, arguments=cartographer_args + ['-load_state_filename', state],
             remappings=[('scan', '/car/scan'), ('imu', '/car/imu/data_raw'), ('odom', '/car/chassis/odom')],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='occupancy_grid_node',
             arguments=['-resolution', '0.025', '-publish_period_sec', '1.0'],
             condition=mapping, parameters=[{'use_sim_time': True}], output='screen'),
    ])
