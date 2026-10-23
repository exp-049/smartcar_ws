"""Nav2 navigation stack for an online Cartographer map or a saved map."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('smartcar_navigation'), 'config', 'nav2_params.yaml'])
    behavior_tree = PathJoinSubstitution([FindPackageShare('smartcar_navigation'), 'behavior_trees', 'stable_detour.xml'])
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('odom', '/car/chassis/odom')]
    params_arg = [params]
    lifecycle_nodes = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor']
    def node(package, executable, name, extra=None, extra_parameters=None):
        return Node(package=package, executable=executable, name=name, output='screen', parameters=params_arg + (extra_parameters or []),
                    remappings=remappings + (extra or []))
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='smartcar_tools', executable='path_guard', name='path_guard', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        node('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_smoother', 'smoother_server', 'smoother_server'),
        node('nav2_planner', 'planner_server', 'planner_server'),
        node('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
             extra=[('is_path_valid', '/car/is_upcoming_path_valid')],
             extra_parameters=[{'default_nav_to_pose_bt_xml': behavior_tree}]),
        node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'autostart': LaunchConfiguration('autostart')}, {'node_names': lifecycle_nodes}]),
    ])
