"""Launch the official micro-ROS Agent independently of HAL and Gazebo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from smartcar_bringup.runtime import start_agent


def generate_launch_description():
    defaults = {'agent_transport': 'udp4', 'agent_port': '8888',
                'agent_device': '', 'agent_baudrate': '115200'}
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=default) for key, default in defaults.items()],
        OpaqueFunction(function=start_agent),
    ])
