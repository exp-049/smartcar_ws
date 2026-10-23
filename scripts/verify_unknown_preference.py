#!/usr/bin/env python3
"""Run real Smac + global costmap against symmetric known/unknown detours.

Uses an isolated ROS domain and no Gazebo, so it may run alongside the car.
Source ROS and the workspace first. Artifacts contain the requested maps,
actual paths and action results; this does not substitute for Gazebo tests.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=87)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    params = {
        'planner_server': {'ros__parameters': {
            'use_sim_time': False, 'planner_plugins': ['GridBased'],
            'GridBased': {'plugin': 'nav2_smac_planner::SmacPlanner2D',
                          'tolerance': 0.0, 'allow_unknown': True,
                          'cost_travel_multiplier': 5.0, 'smooth_path': True}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'use_sim_time': False, 'global_frame': 'map', 'robot_base_frame': 'base_link',
            'update_frequency': 10.0, 'publish_frequency': 5.0,
            'resolution': 0.1, 'robot_radius': 0.1, 'track_unknown_space': True,
            'always_send_full_costmap': True,
            'plugins': ['static_layer', 'inflation_layer', 'familiarity_layer'],
            'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer',
                             'map_topic': '/test_map', 'map_subscribe_transient_local': True},
            'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer',
                                'inflation_radius': 0.35, 'cost_scaling_factor': 5.0},
            'familiarity_layer': {'plugin': 'smartcar_costmap_plugins::UnknownCostLayer',
                                  'unknown_cost': 5}}}}}
    # Exercise the actual project's map interpretation, not an ideal all-zero
    # free map that could hide known cells costing more than unknown cells.
    source = Path(__file__).resolve().parents[1] / 'src/smartcar_navigation/config/nav2_params.yaml'
    project = yaml.safe_load(source.read_text())
    actual = project['global_costmap']['global_costmap']['ros__parameters']
    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    for key in ('lethal_cost_threshold', 'trinary_costmap', 'unknown_cost_value'):
        global_params[key] = actual[key]
    global_params['familiarity_layer'] = actual['familiarity_layer']
    config = args.output / 'params.yaml'
    config.write_text(yaml.safe_dump(params))
    log = (args.output / 'planner.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'run', 'nav2_planner', 'planner_server', '--ros-args', '--params-file', str(config.resolve())],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('verify_unknown_preference')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(OccupancyGrid, '/test_map', qos)
    maps = []
    node.create_subscription(Costmap, '/global_costmap/costmap_raw', maps.append, qos)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = 3.0
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)

    def wait(future, timeout=15.0):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def make_map(known_side, only_unknown=False):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.1
        grid.info.width, grid.info.height = 80, 60
        grid.info.origin.orientation.w = 1.0
        for y in range(60):
            for x in range(80):
                unknown = (y < 30) if known_side == 'north' else (y >= 30)
                if known_side == 'corridor':
                    unknown = not 27 <= y <= 32
                # A measured free probability of 20 must be no more costly
                # than ideal free 0. This caught the graded-static-map bug.
                value = -1 if unknown else 20
                if x in (0, 79) or y in (0, 59) or (known_side != 'corridor' and 39 <= x <= 40 and 20 <= y <= 39):
                    value = 100
                if known_side != 'corridor' and 39 <= x <= 40 and 20 <= y <= 39:
                    value = 65  # Confidence boundary: this must stay lethal.
                if known_side == 'corridor' and 39 <= x <= 40 and 27 <= y <= 32:
                    value = 50  # A visibility frontier is not a physical wall.
                if only_unknown == 'north' and 39 <= x <= 40 and y <= 39:
                    value = 100  # Only the northern unknown route remains.
                if only_unknown == 'south' and 39 <= x <= 40 and y >= 20:
                    value = 100
                if (x - 10)**2 + (y - 30)**2 < 9 or (x - 70)**2 + (y - 30)**2 < 9:
                    value = 0
                grid.data.append(value)
        return grid

    results = []
    try:
        publisher.publish(make_map('north'))
        client = node.create_client(ChangeState, '/planner_server/change_state')
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError('planner_server did not start')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            if not wait(client.call_async(request)).success:
                raise RuntimeError('Lifecycle transition failed')
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        if not action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('Planner action unavailable')
        scenarios = [('known_north', 'north', False, 'north'),
                     ('known_south', 'south', False, 'south'),
                     ('unknown_only', 'south', 'north', 'north'),
                     ('known_shortcut', 'corridor', False, 'direct')]
        for name, known_side, only_unknown, expected_side in scenarios:
            grid = make_map(known_side, only_unknown)
            grid.header.stamp = node.get_clock().now().to_msg()
            maps.clear()
            publisher.publish(grid)
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if maps:
                    latest = maps[-1]
                    north = latest.data[50 * 80 + 10]
                    south = latest.data[10 * 80 + 10]
                    expected = (0, 5) if known_side == 'north' else (5, 0)
                    if known_side == 'corridor':
                        expected = (5, 5)
                    if (north, south) == expected:
                        break
            else:
                raise RuntimeError('Global costmap did not represent the new known/unknown map')
            goal = ComputePathToPose.Goal()
            goal.use_start = True
            for pose, x in ((goal.start, 1.0), (goal.goal, 7.0)):
                pose.header.frame_id = 'map'
                pose.pose.position.x, pose.pose.position.y = x, 3.0
                pose.pose.orientation.w = 1.0
            goal.planner_id = 'GridBased'
            handle = wait(action.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Planning goal rejected')
            result = wait(handle.get_result_async())
            points = [[p.pose.position.x, p.pose.position.y] for p in result.result.path.poses]
            middle = [y for x, y in points if 3.8 <= x <= 4.2]
            side = 'north' if middle and sum(middle) / len(middle) > 3.0 else 'south'
            length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
            if known_side == 'corridor' and points and max(abs(y-3.0) for x, y in points) < 0.11 and length <= 6.1:
                side = 'direct'
            row = {'scenario': name, 'status': result.status, 'chosen_side': side,
                   'expected_side': expected_side, 'passed': result.status == 4 and side == expected_side,
                   'length_m': length,
                   'costs': {'known': 0, 'unknown': 5,
                             'wall': maps[-1].data[30 * 80 + (0 if known_side == 'corridor' else 39)]}, 'points': points}
            results.append(row)
            (args.output / (name + '_map.json')).write_text(json.dumps(list(grid.data)))
            print(json.dumps({k: v for k, v in row.items() if k != 'points'}), flush=True)
        (args.output / 'summary.json').write_text(json.dumps(results, indent=2))
        return 0 if all(row['passed'] and row['costs']['wall'] == 254 for row in results) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()


if __name__ == '__main__':
    raise SystemExit(main())
