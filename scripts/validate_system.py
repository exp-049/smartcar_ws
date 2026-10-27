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
import ast
import re

ROOT = Path(__file__).resolve().parents[1]
errors = []

def fail(message): errors.append(message)

robot = ROOT / 'src/smartcar_description/urdf/smartcar.urdf.xacro'
world = ROOT / 'src/smartcar_gazebo/worlds/smartcar_navigation.sdf'
nominal = ROOT / 'src/smartcar_description/config/nominal.yaml'
real_template = ROOT / 'src/smartcar_description/config/real.template.yaml'
try:
    root = ET.parse(robot).getroot()
    calibration = yaml.safe_load(nominal.read_text())
    model = calibration['model']
    if model.get('wheel_radius') != 0.024: fail('nominal wheel_radius is not 0.024')
    if model.get('wheel_separation') != 0.133: fail('nominal wheel_separation is not 0.133')
    for name in ('calibration_file', 'use_gazebo'):
        if root.find(f"{{http://www.ros.org/wiki/xacro}}arg[@name='{name}']") is None:
            fail(f'xacro argument missing: {name}')
    if root.find(".//{http://www.ros.org/wiki/xacro}if[@value='$(arg use_gazebo)']") is None:
        fail('Gazebo plugins are not conditional on use_gazebo')
    if root.find(".//plugin[@name='gz::sim::systems::DiffDrive']") is None: fail('DiffDrive plugin missing')
    if root.find(".//sensor[@name='x3pro_lidar']/lidar/scan/horizontal/samples").text != '400': fail('lidar samples are not 400')
    real = yaml.safe_load(real_template.read_text())
    if real['metadata']['status'] != 'unmeasured': fail('real calibration template must be unmeasured')
except Exception as exc: fail(f'robot XML: {exc}')
try:
    root = ET.parse(world).getroot()
    names = {m.attrib.get('name') for m in root.findall('.//model')}
    for required in ('ground', 'wall_north', 'wall_south', 'wall_east', 'wall_west', 'inner_wall'):
        if required not in names: fail(f'world model missing: {required}')
except Exception as exc: fail(f'world XML: {exc}')
try:
    maze_path = ROOT / 'src/smartcar_gazebo/worlds/smartcar_maze.sdf'
    maze_text = maze_path.read_text()
    maze = ET.fromstring(maze_text)
    track = calibration['model']['wheel_separation']
    grid = re.search(r'grid=(\d+)x(\d+); corner_fill=occupied', maze_text)
    if not grid or (int(grid.group(1)), int(grid.group(2))) != (20, 20):
        fail('maze must be 20x20 with occupied corner joints')
    match = re.search(r'passage_width=([0-9.]+) m = 6 x wheel_track=([0-9.]+) m', maze_text)
    if not match or abs(float(match.group(1)) - 6 * track) > 1e-9:
        fail('maze passage width must equal six nominal wheel tracks')
    links = maze.findall(".//model[@name='maze_walls']/link")
    passages = []
    for link in links:
        name = link.attrib.get('name', '')
        if not (name.startswith('vertical_') or name.startswith('horizontal_')):
            continue
        dims = [float(value) for value in link.findtext('collision/geometry/box/size').split()]
        passages.append(max(dims[0], dims[1]))
    if not passages or any(abs(width - 6 * track) > 1e-9 for width in passages):
        fail('maze wall segments must preserve the requested corridor width')
    map_config_path = ROOT / 'src/smartcar_navigation/maps/smartcar_maze.yaml'
    map_config = yaml.safe_load(map_config_path.read_text())
    image = map_config_path.parent / map_config['image']
    if not image.is_file() or map_config['resolution'] <= 0:
        fail('maze occupancy map image or resolution is missing')
    if image.is_file():
        header, width_height, depth, raster = image.read_bytes().split(b'\n', 3)
        width, height = (int(value) for value in width_height.split())
        if header != b'P5' or depth != b'255' or len(raster) != width * height:
            fail('maze occupancy map PGM header or raster size is invalid')
        elif any(raster[index] != 0 for index in (0, width - 1, (height - 1) * width, width * height - 1)):
            fail('maze occupancy map corners must be occupied')
except Exception as exc: fail(f'maze assets: {exc}')
try:
    nav = yaml.safe_load((ROOT / 'src/smartcar_navigation/config/nav2_params.yaml').read_text())
    if nav['planner_server']['ros__parameters']['GridBased']['plugin'] != 'nav2_smac_planner::SmacPlanner2D': fail('planner is not SmacPlanner2D')
    if nav['controller_server']['ros__parameters']['FollowPath']['motion_model'] != 'DiffDrive': fail('MPPI motion model is not DiffDrive')
    if nav['local_costmap']['local_costmap']['ros__parameters']['global_frame'] != 'chassis_odom': fail('local costmap frame mismatch')
    if nav['controller_server']['ros__parameters']['FollowPath'].get('visualize') is not True: fail('MPPI optimal trajectory visualization must be enabled')
    if nav['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] != '/car/cmd_vel_checked': fail('collision monitor must use the guarded internal command')
    guard = nav['command_guard']['ros__parameters']
    if guard['output_topic'] != '/car/chassis/cmd_vel' or guard['timeout_sec'] != 0.5: fail('command guard public output or timeout mismatch')
    if not (ROOT / 'src/smartcar_navigation/config/real.yaml').is_file(): fail('real navigation profile missing')
except Exception as exc: fail(f'Nav2 YAML: {exc}')
try:
    rviz = yaml.safe_load((ROOT / 'src/smartcar_description/rviz/smartcar_nav.rviz').read_text())
    displays = {item['Name']: item for item in rviz['Visualization Manager']['Displays']}
    expected_topics = {
        'Global Plan': '/plan',
        'MPPI Optimal Trajectory': '/optimal_trajectory',
        'Global Costmap': '/global_costmap/costmap',
    }
    for name, topic in expected_topics.items():
        if name not in displays or displays[name].get('Topic', {}).get('Value') != topic:
            fail(f'RViz {name} display must subscribe to {topic}')
except Exception as exc: fail(f'RViz config: {exc}')
launch = (ROOT / 'src/smartcar_gazebo/launch/sim.launch.py').read_text()
if 'tf2_msgs' in launch or 'smartcar/gz/tf' in launch and 'tf_topic' in launch: fail('Gazebo TF is bridged or exposed as ROS /tf')
for path in ROOT.glob('src/smartcar_*/**/*.py'):
    try: ast.parse(path.read_text(), filename=str(path))
    except Exception as exc: fail(f'python syntax {path.relative_to(ROOT)}: {exc}')
for path in ROOT.glob('src/smartcar_cartographer/config/*_real.lua'):
    if 'include "smartcar_2d' not in path.read_text(): fail(f'real Cartographer config must inherit shared estimator: {path.name}')
if errors:
    print('VALIDATION FAILED')
    print('\n'.join(' - ' + e for e in errors))
    sys.exit(1)
print('Smartcar source contract: OK')
