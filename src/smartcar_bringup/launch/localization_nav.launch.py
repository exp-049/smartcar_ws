"""Localization and navigation with selectable sim/real devices."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from smartcar_bringup.runtime import common_arguments, launch_stack


def generate_launch_description():
    return LaunchDescription([
        *common_arguments(), DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('state_file', default_value=''),
        OpaqueFunction(function=lambda context: launch_stack(context, 'localization')),
    ])
