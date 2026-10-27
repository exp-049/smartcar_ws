"""Launch real devices and their measured model without SLAM/navigation."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from smartcar_bringup.runtime import common_arguments, hardware_actions


def generate_launch_description():
    return LaunchDescription([
        *common_arguments(), SetLaunchConfiguration('runtime', 'real'),
        OpaqueFunction(function=lambda context: hardware_actions(context)[0]),
    ])
