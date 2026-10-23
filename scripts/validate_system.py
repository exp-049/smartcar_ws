#!/usr/bin/env python3
"""Fast, dependency-light checks for the integrated Smartcar configuration.

This script intentionally does not require Gazebo/Nav2 to be installed. It
checks the contract that can be validated from the source tree before the
runtime dependency installation.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
errors = []

def fail(message): errors.append(message)

robot = ROOT / 'src/smartcar_description/urdf/smartcar.urdf.xacro'
world = ROOT / 'src/smartcar_gazebo/worlds/smartcar_navigation.sdf'
try:
    root = ET.parse(robot).getroot()
    props = {p.attrib.get('name'): p.attrib.get('value') for p in root.findall('{http://www.ros.org/wiki/xacro}property')}
    if props.get('wheel_radius') != '0.024': fail('wheel_radius is not 0.024')
    if props.get('wheel_separation') != '0.133': fail('wheel_separation is not 0.133')
    if root.find(".//plugin[@name='gz::sim::systems::DiffDrive']") is None: fail('DiffDrive plugin missing')
    if root.find(".//sensor[@name='x3pro_lidar']/lidar/scan/horizontal/samples").text != '400': fail('lidar samples are not 400')
except Exception as exc: fail(f'robot XML: {exc}')
try:
    root = ET.parse(world).getroot()
    names = {m.attrib.get('name') for m in root.findall('.//model')}
    for required in ('ground', 'wall_north', 'wall_south', 'wall_east', 'wall_west', 'inner_wall'):
        if required not in names: fail(f'world model missing: {required}')
except Exception as exc: fail(f'world XML: {exc}')
try:
    nav = yaml.safe_load((ROOT / 'src/smartcar_navigation/config/nav2_params.yaml').read_text())
    if nav['planner_server']['ros__parameters']['GridBased']['plugin'] != 'nav2_smac_planner::SmacPlanner2D': fail('planner is not SmacPlanner2D')
    if nav['controller_server']['ros__parameters']['FollowPath']['motion_model'] != 'DiffDrive': fail('MPPI motion model is not DiffDrive')
    if nav['local_costmap']['local_costmap']['ros__parameters']['global_frame'] != 'chassis_odom': fail('local costmap frame mismatch')
except Exception as exc: fail(f'Nav2 YAML: {exc}')
launch = (ROOT / 'src/smartcar_gazebo/launch/sim.launch.py').read_text()
if 'tf2_msgs' in launch or 'smartcar/gz/tf' in launch and 'tf_topic' in launch: fail('Gazebo TF is bridged or exposed as ROS /tf')
if errors:
    print('VALIDATION FAILED')
    print('\n'.join(' - ' + e for e in errors))
    sys.exit(1)
print('Smartcar source contract: OK')
