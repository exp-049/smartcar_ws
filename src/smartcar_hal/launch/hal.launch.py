"""Launch the integrated UDP sensor driver or a user-supplied replacement."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from smartcar_hal.launch_support import validate_hal


def start_hal(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    params = validate_hal(value('hal_package'), value('hal_executable'), value('hal_params_file'))
    return [Node(
        package=value('hal_package'), executable=value('hal_executable'),
        name='smartcar_sensor_hal', parameters=[params, {'use_sim_time': False}], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hal_package', default_value='smartcar_sensor_hal'),
        DeclareLaunchArgument('hal_executable', default_value='sensor_hal'),
        DeclareLaunchArgument('hal_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('smartcar_sensor_hal'), 'config', 'sensors.yaml'])),
        OpaqueFunction(function=start_hal),
    ])
