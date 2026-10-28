"""Compose sim/real devices with the same Cartographer and Nav2 stack."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from smartcar_hal.launch_support import require_executable, validate_hal
import yaml


def share(package, *parts):
    return str(Path(get_package_share_directory(package)).joinpath(*parts))


def value(context, name):
    return LaunchConfiguration(name).perform(context)


def boolean(context, name):
    raw = value(context, name).lower()
    if raw not in ('true', 'false'):
        raise RuntimeError(f'{name} must be true or false')
    return raw == 'true'


def include(package, filename, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(package, 'launch', filename)),
        launch_arguments={key: str(val).lower() if isinstance(val, bool) else str(val)
                          for key, val in arguments.items()}.items())


def common_arguments():
    defaults = {
        'runtime': 'sim', 'headless': 'false', 'use_rviz': 'true',
        'world': 'smartcar_navigation.sdf', 'calibration_file': '',
        'start_hal': 'true', 'hal_package': 'smartcar_sensor_hal', 'hal_executable': 'sensor_hal',
        'hal_params_file': share('smartcar_sensor_hal', 'config', 'sensors.yaml'),
        'start_agent': 'true', 'agent_transport': 'udp4', 'agent_port': '8891',
        'agent_device': '', 'agent_baudrate': '460800',
        'chassis_discovery': 'true', 'chassis_discovery_port': '8892',
        'chassis_broadcast_address': '', 'chassis_device_id': '', 'chassis_device_ip': '',
        'chassis_interface': '',
        'readiness_timeout': '120.0', 'configuration_basename': '',
        'use_odom': 'true', 'use_imu': 'true',
        'nav_params_file': share('smartcar_navigation', 'config', 'nav2_params.yaml'),
        'nav_params_override': '',
    }
    return [DeclareLaunchArgument(key, default_value=default)
            for key, default in defaults.items()]


def agent_arguments(context):
    transport = value(context, 'agent_transport')
    if transport in ('udp4', 'udp6', 'tcp4', 'tcp6'):
        port = int(value(context, 'agent_port'))
        if not 1 <= port <= 65535:
            raise RuntimeError('agent_port must be between 1 and 65535')
        return [transport, '-p', str(port)]
    if transport == 'serial':
        device = value(context, 'agent_device')
        baud = int(value(context, 'agent_baudrate'))
        if not device or baud <= 0:
            raise RuntimeError('serial Agent requires agent_device and a positive baudrate')
        return ['serial', '--dev', device, '-b', str(baud)]
    raise RuntimeError(f'Unsupported Agent transport: {transport}')


def validate_agent(context):
    agent_arguments(context)
    try:
        require_executable('micro_ros_agent', 'micro_ros_agent')
    except RuntimeError as exc:
        raise RuntimeError(
            'Official micro_ros_agent is unavailable. Install its Jazzy version and '
            'source the overlay (see README); or set start_agent:=false for an '
            'external Agent. Generic MicroXRCEAgent is not selected automatically.') from exc


def start_agent(context):
    validate_agent(context)
    node = Node(package='micro_ros_agent', executable='micro_ros_agent',
                arguments=agent_arguments(context), output='screen')
    def stopped(event, context):
        if not context.is_shutdown:
            return [EmitEvent(event=Shutdown(reason=f'micro-ROS Agent exited: {event.returncode}'))]
        return []
    actions = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=stopped)), node]
    if value(context, 'agent_transport') == 'udp4' and boolean(context, 'chassis_discovery'):
        discovery_port = int(value(context, 'chassis_discovery_port'))
        if not 1024 <= discovery_port <= 65535 or discovery_port == int(value(context, 'agent_port')):
            raise RuntimeError('chassis_discovery_port must be 1024..65535 and differ from agent_port')
        discovery = Node(package='smartcar_bringup', executable='chassis_discovery', output='screen',
                         arguments=['--agent-port', value(context, 'agent_port'),
                                    '--discovery-port', str(discovery_port),
                                    '--broadcast', value(context, 'chassis_broadcast_address'),
                                    '--device-id', value(context, 'chassis_device_id'),
                                    '--device-ip', value(context, 'chassis_device_ip'),
                                    '--interface', value(context, 'chassis_interface')])
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=discovery, on_exit=stopped)), discovery])
    return actions


def validate_calibration(path, runtime):
    path = Path(path)
    if not path.is_file():
        raise RuntimeError(f'Calibration file does not exist: {path}')
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise RuntimeError('Calibration must contain metadata and model mappings')
    metadata, model = config.get('metadata', {}), config.get('model', {})
    if not isinstance(metadata, dict) or not isinstance(model, dict):
        raise RuntimeError('Calibration metadata and model must be mappings')
    for key in ('robot_id', 'calibration_version', 'status', 'measurement_method', 'source_data'):
        if not metadata.get(key):
            raise RuntimeError(f'Calibration metadata is missing {key}')
    if runtime == 'real':
        if metadata['status'] != 'measured':
            raise RuntimeError('Real runtime requires measured calibration; real.template.yaml is UNMEASURED')
        versions = config.get('device_calibration', {})
        required = ('chassis_firmware_version', 'chassis_calibration_version',
                    'imu_firmware_version', 'imu_calibration_version')
        provenance = list(metadata.values()) + [versions.get(key, '') for key in required]
        if any(str(item) in ('REQUIRED', 'UNMEASURED', 'none', '') for item in provenance):
            raise RuntimeError('Record measured provenance and chassis/IMU firmware/calibration versions')
    for key in ('wheel_radius', 'wheel_separation', 'wheel_width', 'body_length',
                'body_width', 'body_height', 'body_mass', 'inertia_xx', 'inertia_yy',
                'inertia_zz', 'footprint_radius'):
        number = float(model.get(key, float('nan')))
        if not math.isfinite(number) or number <= 0:
            raise RuntimeError(f'Calibration model.{key} must be finite and positive')
    for key in ('body_z', 'caster_clearance'):
        if not math.isfinite(float(model.get(key, float('nan')))):
            raise RuntimeError(f'Calibration model.{key} must be finite')
    vectors = {}
    for key in ('chassis_xyz', 'chassis_rpy', 'laser_xyz', 'laser_rpy', 'imu_xyz', 'imu_rpy'):
        numbers = [float(item) for item in str(model.get(key, '')).split()]
        if len(numbers) != 3 or not all(math.isfinite(item) for item in numbers):
            raise RuntimeError(f'Calibration model.{key} must contain three finite numbers')
        vectors[key] = numbers
    envelope = max(math.hypot(model['body_length'] / 2, model['body_width'] / 2),
                   math.hypot(model['wheel_radius'], (model['wheel_separation'] + model['wheel_width']) / 2),
                   math.hypot(*vectors['laser_xyz'][:2]) + 0.035,
                   math.hypot(*vectors['imu_xyz'][:2]) + 0.018, 0.082)
    if model['footprint_radius'] < envelope:
        raise RuntimeError('footprint_radius must cover the physical model envelope')
    return config


def hardware_actions(context):
    runtime = value(context, 'runtime')
    if runtime not in ('sim', 'real'):
        raise RuntimeError('runtime must be sim or real')
    if runtime == 'real':
        if boolean(context, 'start_hal'):
            validate_hal(value(context, 'hal_package'), value(context, 'hal_executable'),
                         value(context, 'hal_params_file'))
        if boolean(context, 'start_agent'):
            validate_agent(context)
    calibration = value(context, 'calibration_file') or share(
        'smartcar_description', 'config', 'nominal.yaml' if runtime == 'sim' else 'real.template.yaml')
    config = validate_calibration(calibration, runtime)
    actions = [LogInfo(msg=f'Runtime={runtime}; calibration={calibration}; metadata={config["metadata"]}; '
                      f'device versions={config.get("device_calibration", {})}')]
    if runtime == 'sim':
        actions.append(include('smartcar_gazebo', 'sim.launch.py',
                               headless=value(context, 'headless'), use_rviz=value(context, 'use_rviz'),
                               world=value(context, 'world'), calibration_file=calibration))
    else:
        if boolean(context, 'start_agent'):
            actions.extend(start_agent(context))
        if boolean(context, 'start_hal'):
            actions.append(include('smartcar_hal', 'hal.launch.py', **{
                name: value(context, name) for name in ('hal_package', 'hal_executable', 'hal_params_file')}))
        actions.append(include('smartcar_description', 'display.launch.py', use_sim_time=False,
                               use_gazebo=False, use_rviz=value(context, 'use_rviz'),
                               calibration_file=calibration))
    return actions, config


def launch_stack(context, mode):
    if mode == 'localization':
        for key in ('map_yaml', 'state_file'):
            if not value(context, key) or not Path(value(context, key)).is_file():
                raise RuntimeError(f'localization_nav requires existing {key}; save a map/pbstream first')
    timeout = float(value(context, 'readiness_timeout'))
    if not math.isfinite(timeout) or timeout <= 0:
        raise RuntimeError('readiness_timeout must be finite and positive')
    actions, calibration = hardware_actions(context)
    runtime = value(context, 'runtime')
    sim_time = runtime == 'sim'
    config = value(context, 'configuration_basename') or (
        'smartcar_2d' + ('_localization' if mode == 'localization' else '')
        + ('_real' if runtime == 'real' else '') + '.lua')
    actions.append(include('smartcar_cartographer', 'cartographer.launch.py', mode=mode,
                           configuration_basename=config, use_sim_time=sim_time,
                           use_odom=value(context, 'use_odom'), use_imu=value(context, 'use_imu'),
                           state_file=value(context, 'state_file') if mode == 'localization' else ''))
    if mode == 'localization':
        actions.extend([
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[{'use_sim_time': sim_time, 'yaml_filename': value(context, 'map_yaml')}], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map_server',
                 parameters=[{'use_sim_time': sim_time, 'autostart': True, 'node_names': ['map_server']}], output='screen'),
        ])
    ready = Node(package='smartcar_tools', executable='wait_for_ready', output='screen',
                 parameters=[{'use_sim_time': sim_time, 'timeout_sec': timeout}])
    def after_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Sensors, time, TF or map did not become ready'))]
        return [include(
            'smartcar_navigation', 'navigation.launch.py', use_sim_time=sim_time, autostart=True,
            params_file=value(context, 'nav_params_file'),
            params_override=value(context, 'nav_params_override') or share(
                'smartcar_navigation', 'config', f'{runtime}.yaml'),
            robot_radius=str(calibration['model']['footprint_radius']))]
    actions.extend([RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)), ready])
    return actions
