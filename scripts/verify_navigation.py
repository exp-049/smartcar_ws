#!/usr/bin/env python3
"""Exercise a real running Smartcar/Nav2 stack and record navigation evidence.

Source ROS and the workspace, start mapping_nav.launch.py, then run:
  python3 scripts/verify_navigation.py 1.0 0.8 --output notes/nav_validation/run1
This sends a real goal; it does not mock the planner, controller, or simulator.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def gazebo_request(service, request_type, request):
    result = subprocess.run(
        ['gz', 'service', '-s', '/world/smartcar_navigation_world/' + service,
         '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req', request],
        capture_output=True, text=True, timeout=8.0)
    if result.returncode != 0 or 'data: true' not in result.stdout:
        raise RuntimeError('Gazebo %s failed: %s %s' % (service, result.stdout, result.stderr))


def inject_obstacle(plan):
    # This scenario starts at the world's default spawn.  Before translation,
    # map and world axes coincide closely; place a test cube on the first plan.
    points = plan['points']
    distance = 0.0
    x, y = points[-1]
    for a, b in zip(points, points[1:]):
        distance += math.hypot(b[0] - a[0], b[1] - a[1])
        if distance >= 0.8:
            x, y = b
            break
    sdf = ('<sdf version="1.9"><model name="validation_obstacle">'
           '<static>true</static><pose>%.6f %.6f 0.25 0 0 0</pose>'
           '<link name="link"><collision name="collision">'
           '<geometry><box><size>0.4 0.4 0.5</size></box></geometry></collision>'
           '<visual name="visual"><geometry><box><size>0.4 0.4 0.5</size></box>'
           '</geometry><material><ambient>1 0.5 0 1</ambient></material>'
           '</visual></link></model></sdf>') % (x, y)
    gazebo_request('create', 'gz.msgs.EntityFactory', 'sdf: ' + json.dumps(sdf))
    return {'world_xy': [x, y], 'size': [0.4, 0.4, 0.5]}


class Observer(Node):
    def __init__(self, directory):
        super().__init__('smartcar_navigation_verification')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.latest = {}
        self.samples = []
        self.plans = []
        self.directory = directory
        self.create_subscription(Clock, '/clock', self.clock_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/car/chassis/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/car/chassis/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(LaserScan, '/car/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.plan_cb, 10)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.cost_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def clock_cb(self, msg):
        self.latest['sim_time'] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def odom_cb(self, msg):
        self.latest['odom_position'] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.latest['measured_velocity'] = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]

    def cmd_cb(self, msg):
        self.latest['command'] = [msg.linear.x, msg.angular.z]

    def scan_cb(self, msg):
        self.latest['scan_min'] = min((v for v in msg.ranges if math.isfinite(v)), default=None)

    def plan_cb(self, msg):
        points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.plans.append({'sim_time': self.latest.get('sim_time'), 'points': points})
        self.latest['plans_received'] = len(self.plans)

    def cost_cb(self, msg):
        self.latest['local_obstacle_cells'] = sum(253 <= v <= 254 for v in msg.data)

    def sample(self):
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            q = transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.latest['map_pose'] = [transform.translation.x, transform.translation.y, yaw]
        except Exception:
            pass
        sample = dict(self.latest, monotonic_time=time.monotonic())
        self.samples.append(sample)
        print(json.dumps(sample), flush=True)

    def cancel_navigation(self):
        client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if client.wait_for_service(timeout_sec=3.0):
            future = client.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.destroy_client(client)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--inject-obstacle', action='store_true',
                        help='From the default spawn, insert a temporary cube on the first plan')
    parser.add_argument('--check-room', action='store_true',
                        help='Bundled world/default spawn only: reject paths outside the room')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Observer(args.output)
    started = time.monotonic()
    # /plan is volatile and may be published only once. Establish discovery
    # before submitting a goal so an early plan is not silently missed.
    deadline = started + 40.0
    while node.count_publishers('/plan') == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.count_publishers('/plan') == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Planner /plan publisher was not discovered')
    interrupted = False
    obstacle = None
    with (args.output / 'goal.log').open('w') as log:
        command = ['ros2', 'run', 'smartcar_tools', 'navigate_to_pose',
                   str(args.x), str(args.y), '--yaw', str(args.yaw)]
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            next_sample = started
            while process.poll() is None and time.monotonic() - started < args.timeout:
                rclpy.spin_once(node, timeout_sec=0.05)
                if args.inject_obstacle and obstacle is None and node.plans:
                    obstacle = inject_obstacle(node.plans[0])
                    obstacle.update(plans_before_insertion=len(node.plans),
                                    sim_time=node.latest.get('sim_time'))
                    print('OBSTACLE_INSERTED', json.dumps(obstacle), flush=True)
                if time.monotonic() >= next_sample:
                    node.sample()
                    next_sample = time.monotonic() + 2.0
        except KeyboardInterrupt:
            interrupted = True
        finally:
            timed_out = process.poll() is None
            if timed_out:
                node.cancel_navigation()
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.0)
            settle_until = time.monotonic() + 1.0
            while time.monotonic() < settle_until:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.sample()
            if obstacle is not None:
                gazebo_request('remove', 'gz.msgs.Entity',
                               'name: "validation_obstacle" type: MODEL')
                obstacle['removed'] = True
    goal_log = (args.output / 'goal.log').read_text()
    matches = re.findall(r'finished with status (\d+)', goal_log)
    status = int(matches[-1]) if matches else None
    pose = node.latest.get('map_pose')
    summary = {
        'command': command, 'goal': [args.x, args.y, args.yaw],
        'action_status': status, 'process_exit_code': process.returncode,
        'timed_out': timed_out, 'interrupted': interrupted,
        'elapsed_wall_seconds': time.monotonic() - started,
        'plans_received': len(node.plans), 'final': node.latest,
        'injected_obstacle': obstacle,
        'position_error_m': math.hypot(pose[0] - args.x, pose[1] - args.y) if pose else None,
        'yaw_error_rad': abs(math.atan2(math.sin(pose[2] - args.yaw), math.cos(pose[2] - args.yaw))) if pose else None,
    }
    summary['plan_lengths_m'] = [
        sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(p['points'], p['points'][1:]))
        for p in node.plans]
    if args.check_room:
        # This is a validation assertion for this world, not a navigation
        # geofence. Its map/world alignment assumes the default spawn.
        summary['room_exterior_plan_points'] = sum(
            abs(x) > 3.85 or abs(y) > 2.85 for p in node.plans for x, y in p['points'])
    summary['goal_tolerance_passed'] = bool(
        pose and summary['position_error_m'] <= 0.10 and summary['yaw_error_rad'] <= 0.18)
    measured = node.latest.get('measured_velocity')
    summary['stopped'] = bool(measured and all(abs(v) < 0.001 for v in measured))
    summary['plans_observed'] = bool(node.plans)
    (args.output / 'observations.json').write_text(json.dumps({
        'summary': summary, 'samples': node.samples, 'plans': node.plans}, indent=2))
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print('RESULT', json.dumps(summary), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    replanned = not args.inject_obstacle or (
        obstacle is not None and len(node.plans) > obstacle['plans_before_insertion'])
    return 0 if (status == 4 and not timed_out and replanned and summary['plans_observed']
                 and summary.get('room_exterior_plan_points', 0) == 0
                 and summary['goal_tolerance_passed'] and summary['stopped']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
