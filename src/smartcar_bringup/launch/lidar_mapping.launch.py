"""Map using the real UDP lidar without a chassis, IMU, or micro-ROS Agent."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensors = PathJoinSubstitution([FindPackageShare('smartcar_sensor_hal'), 'launch', 'sensors.launch.py'])
    cartographer = PathJoinSubstitution([FindPackageShare('smartcar_cartographer'), 'launch', 'cartographer.launch.py'])
    rviz_config = PathJoinSubstitution([FindPackageShare('smartcar_cartographer'), 'config', 'lidar_mapping.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('start_sensor', default_value='true', description='Start the UDP sensor receiver'),
        DeclareLaunchArgument('device_ip', default_value='', description='Optional S3 IP for discovery'),
        DeclareLaunchArgument('device_id', default_value='', description='Optional S3 device ID'),
        DeclareLaunchArgument('broadcast_address', default_value='', description='Optional subnet broadcast address'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors), condition=IfCondition(LaunchConfiguration('start_sensor')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ('device_ip', 'device_id', 'broadcast_address')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer), launch_arguments={
                'mode': 'mapping', 'use_sim_time': 'false',
                'configuration_basename': 'smartcar_2d_lidar_only.lua',
                'use_odom': 'false', 'use_imu': 'false',
            }.items()),
        Node(package='rviz2', executable='rviz2', name='lidar_mapping_rviz',
             arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('use_rviz')),
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
