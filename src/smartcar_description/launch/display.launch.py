"""Publish one calibrated robot model and optionally open RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def publish_model(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    path = os.path.join(get_package_share_directory('smartcar_description'), 'urdf', 'smartcar.urdf.xacro')
    xml = xacro.process_file(path, mappings={
        'use_gazebo': value('use_gazebo'), 'calibration_file': value('calibration_file')}).toxml()
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', parameters=[{
                     'robot_description': xml, 'use_sim_time': value('use_sim_time') == 'true'}], output='screen')]


def generate_launch_description():
    package_share = get_package_share_directory('smartcar_description')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='false'),
        DeclareLaunchArgument('calibration_file', default_value=os.path.join(package_share, 'config', 'nominal.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='false'),
        DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(
            package_share, 'rviz', 'smartcar_nav.rviz')),
        OpaqueFunction(function=publish_model),
        Node(package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher',
             condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
             parameters=[{'use_sim_time': clock}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config_file')],
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             parameters=[{'use_sim_time': clock}], output='screen'),
    ])
