"""Start one live UDP receiver; fixed and dynamic TF belong to other packages."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start_hal(context):
    # Empty launch arguments preserve values from a user-supplied YAML file.
    overrides = {}
    for name in ('device_ip', 'device_id', 'broadcast_address'):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = ParameterValue(value, value_type=str)
    return [Node(
        package='smartcar_sensor_hal', executable='sensor_hal',
        name='smartcar_sensor_hal', output='screen',
        parameters=[LaunchConfiguration('config_file').perform(context), overrides],
    )]


def generate_launch_description():
    share = get_package_share_directory('smartcar_sensor_hal')
    config = os.path.join(share, 'config', 'sensors.yaml')
    rviz_config = os.path.join(share, 'config', 'lidar.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('device_ip', default_value='', description='Optional S3 IPv4 override; empty uses discovery'),
        DeclareLaunchArgument('device_id', default_value='', description='Optional stable 12-hex device ID; required when multiple cars are found'),
        DeclareLaunchArgument('broadcast_address', default_value='', description='Optional subnet broadcast override'),
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz with the lidar view'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        OpaqueFunction(function=_start_hal),
        Node(
            package='rviz2', executable='rviz2', name='smartcar_lidar_view', output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
