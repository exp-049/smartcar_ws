"""Start Gazebo Harmonic, the unified robot description, and directional bridges."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare("smartcar_gazebo"), "worlds", LaunchConfiguration("world")])
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    xacro_file = PathJoinSubstitution([FindPackageShare("smartcar_description"), "urdf", "smartcar.urdf.xacro"])
    rviz_file = PathJoinSubstitution([FindPackageShare("smartcar_description"), "rviz", "smartcar_nav.rviz"])
    robot_description = Command(["xacro ", xacro_file])
    bridge_topics = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/car/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/car/imu/gz@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/car/chassis/odom/gz@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/car/chassis/cmd_vel/gz@geometry_msgs/msg/Twist]gz.msgs.Twist",
    ]
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world", default_value="smartcar_navigation.sdf"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": [world, " -r"]}.items(),
            condition=UnlessCondition(LaunchConfiguration("headless"))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": [world, " -r -s --headless-rendering"]}.items(),
            condition=IfCondition(LaunchConfiguration("headless"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}], output="screen"),
        TimerAction(period=3.0, actions=[
            Node(package="ros_gz_sim", executable="create", name="spawn_smartcar",
                 arguments=["-topic", "robot_description", "-name", "smartcar", "-x", "0", "-y", "0", "-z", "0.001"], output="screen")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="smartcar_bridge", arguments=bridge_topics, output="screen"),
        Node(package="smartcar_sim_io", executable="imu_sanitizer", name="imu_sanitizer",
             parameters=[{"use_sim_time": True, "input_topic": "/car/imu/gz", "output_topic": "/car/imu/data_raw"}], output="screen"),
        Node(package="smartcar_sim_io", executable="odom_republisher", name="odom_republisher",
             parameters=[{"use_sim_time": True, "input_topic": "/car/chassis/odom/gz", "output_topic": "/car/chassis/odom"}], output="screen"),
        Node(package="smartcar_sim_io", executable="cmd_watchdog", name="cmd_watchdog",
             parameters=[{"input_topic": "/car/chassis/cmd_vel", "output_topic": "/car/chassis/cmd_vel/gz", "timeout_sec": 0.5}], output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_file],
             condition=IfCondition(LaunchConfiguration("use_rviz")), parameters=[{"use_sim_time": True}], output="screen"),
    ])
