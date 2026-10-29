"""Start Cartographer 2D and, in mapping mode, its OccupancyGrid converter."""
import json
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def cartographer_nodes(context):
    # Cartographer reads sensor settings from Lua, not ROS parameters. Create a
    # private wrapper so concurrent launches cannot overwrite each other's settings.
    def sensor_flag(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise RuntimeError(f'{name} must be true or false')
        return value

    use_odom = sensor_flag('use_odom')
    use_imu = sensor_flag('use_imu')
    stale_timeout = float(LaunchConfiguration('pose_stale_timeout_sec').perform(context))
    if not (stale_timeout >= 0.0 and stale_timeout < float('inf')):
        raise RuntimeError('pose_stale_timeout_sec must be finite and non-negative')
    basename = LaunchConfiguration('configuration_basename').perform(context)
    if (not basename or Path(basename).name != basename
            or any(ord(char) < 32 for char in basename)):
        raise RuntimeError('configuration_basename must be a Lua file name')
    source_dir = Path(get_package_share_directory('smartcar_cartographer')) / 'config'
    if not (source_dir / basename).is_file():
        raise RuntimeError(f'Cartographer configuration not found: {basename}')
    temp_dir = tempfile.TemporaryDirectory(prefix='smartcar_cartographer_')
    try:
        config_dir = Path(temp_dir.name)
        for entry in source_dir.iterdir():
            (config_dir / entry.name).symlink_to(entry.resolve(), target_is_directory=entry.is_dir())
        with tempfile.NamedTemporaryFile(mode='w', suffix='.lua', prefix='launch_',
                                         dir=config_dir, delete=False, encoding='utf-8') as wrapper:
            wrapper.write(f'include {json.dumps(basename, ensure_ascii=False)}\n'
                          f'options.use_odometry = {use_odom}\n'
                          f'TRAJECTORY_BUILDER_2D.use_imu_data = {use_imu}\n'
                          'return options\n')
            wrapper_name = Path(wrapper.name).name
        return make_nodes(config_dir, wrapper_name, stale_timeout, use_imu, use_odom) + [
            RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=lambda context: temp_dir.cleanup())]))]
    except Exception:
        temp_dir.cleanup()
        raise


def make_nodes(config_dir, basename, stale_timeout, use_imu, use_odom):
    mode = LaunchConfiguration('mode')
    state = LaunchConfiguration('state_file')
    cartographer_args = ['-configuration_directory', str(config_dir),
                         '-configuration_basename', basename]
    mapping = IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))
    localization = IfCondition(PythonExpression(["'", mode, "' == 'localization'"]))
    tf_remappings = ([('/tf', '/cartographer_internal/tf'),
                      ('/tf_static', '/cartographer_internal/tf_static')]
                     if stale_timeout > 0.0 else [])
    actions = [
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             condition=mapping, arguments=cartographer_args,
             remappings=[('scan', '/car/scan'), ('imu', '/car/imu/data_raw'),
                         ('odom', '/car/chassis/odom')] + tf_remappings,
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}], output='screen'),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_localization_node',
             condition=localization, arguments=cartographer_args + ['-load_state_filename', state],
             remappings=[('scan', '/car/scan'), ('imu', '/car/imu/data_raw'),
                         ('odom', '/car/chassis/odom')] + tf_remappings,
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}], output='screen'),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='occupancy_grid_node',
             arguments=['-resolution', '0.025', '-publish_period_sec', '1.0'],
             condition=mapping, parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}], output='screen'),
    ]
    if stale_timeout > 0.0:
        actions.insert(0, Node(package='smartcar_tools', executable='cartographer_tf_guard',
                               name='cartographer_tf_guard',
                               parameters=[{'stale_timeout_sec': stale_timeout,
                                            'require_imu': use_imu == 'true',
                                            'require_odom': use_odom == 'true',
                                            'use_sim_time': ParameterValue(
                                                LaunchConfiguration('use_sim_time'),
                                                value_type=bool)}],
                               output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('configuration_basename', default_value='smartcar_2d.lua'),
        DeclareLaunchArgument('state_file', default_value=''),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument('use_odom', default_value='true', description='Fuse odometry in Cartographer'),
        DeclareLaunchArgument('use_imu', default_value='true', description='Fuse IMU data in Cartographer'),
        DeclareLaunchArgument('pose_stale_timeout_sec', default_value='0.0',
                              description='Stop relaying Cartographer pose TF after required sensor streams go stale; 0 disables the guard'),
        OpaqueFunction(function=cartographer_nodes),
    ])
