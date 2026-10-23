#!/usr/bin/env python3
"""Measure clockwise/counterclockwise motion against Gazebo truth.

Run with mapping_nav active in the bundled world and no navigation goal.
Truth uses a private validation topic, never /tf or a SLAM input.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    bridge_log = (args.output / 'truth_bridge.log').open('w')
    topic = '/world/smartcar_navigation_world/dynamic_pose/info'
    bridge = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '--ros-args', '-r', topic + ':=/validation/rotation_truth'],
        stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Node('verify_rotation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data, samples = {}, []
    received_at = [0.0]

    def truth(msg):
        # Pose_V conversion in this ros_gz version omits names. The bundled
        # world has one dynamic model: car model first, then base and wheels.
        if len(msg.transforms) == 4:
            transform = msg.transforms[0].transform
            data.update(truth_yaw=yaw(transform.rotation),
                        truth_position=[transform.translation.x, transform.translation.y])
            received_at[0] = time.monotonic()

    def imu(msg):
        data['imu_wz'] = msg.angular_velocity.z

    def odom(msg):
        data.update(odom_yaw=yaw(msg.pose.pose.orientation), odom_wz=msg.twist.twist.angular.z)

    node.create_subscription(TFMessage, '/validation/rotation_truth', truth, qos_profile_sensor_data)
    node.create_subscription(Imu, '/car/imu/data_raw', imu, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/car/chassis/odom', odom, 10)
    publisher = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    lifecycle = node.create_client(GetState, '/velocity_smoother/get_state')
    try:
        if not lifecycle.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('Velocity smoother not available')
        deadline = time.monotonic() + 30.0
        active = False
        while time.monotonic() < deadline:
            future = lifecycle.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
            if future.done() and future.result():
                active = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            elif not future.done():
                lifecycle.remove_pending_request(future)
            rclpy.spin_once(node, timeout_sec=0.05)
            if (active and all(k in data for k in ('truth_yaw', 'imu_wz', 'odom_wz'))
                    and buffer.can_transform('map', 'base_link', rclpy.time.Time())):
                break
        else:
            raise RuntimeError('Clock, truth, sensors, TF or active command chain missing')
        started = node.get_clock().now().nanoseconds * 1e-9
        last_sample = last_command = -1.0
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.005)
            t = node.get_clock().now().nanoseconds * 1e-9 - started
            if t > 18.0:
                break
            if time.monotonic() - received_at[0] > 1.0:
                raise RuntimeError('Gazebo truth stopped updating')
            if t-last_command >= 0.045:
                command = Twist()
                command.angular.z = 0.4 if t < 4 or 12 <= t < 16 else (-0.4 if t < 12 else 0.0)
                publisher.publish(command)
                last_command = t
            if t-last_sample >= 0.2:
                transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
                row = dict(data, t=t, map_yaw=yaw(transform.rotation),
                           map_position=[transform.translation.x, transform.translation.y])
                row['yaw_error'] = math.atan2(math.sin(row['map_yaw']-row['truth_yaw']),
                                              math.cos(row['map_yaw']-row['truth_yaw']))
                samples.append(row)
                last_sample = t
        else:
            raise RuntimeError('Simulation time stopped advancing')
        ratios = [abs(s['imu_wz']/s['odom_wz']) for s in samples
                  if abs(s['odom_wz']) > 0.38 and any(a < s['t'] < b for a, b in [(1, 3.8), (6, 11.8), (14, 15.8)])]
        initial = samples[0]['truth_position']
        summary = {
            'median_gyro_to_encoder_rate_ratio': statistics.median(ratios),
            'max_localization_yaw_error_rad': max(abs(s['yaw_error']) for s in samples),
            'max_truth_translation_m': max(math.hypot(s['truth_position'][0]-initial[0],
                                                     s['truth_position'][1]-initial[1]) for s in samples)}
        summary['passed'] = (abs(summary['median_gyro_to_encoder_rate_ratio']-1.0) <= 0.05
                             and summary['max_localization_yaw_error_rad'] <= math.radians(2)
                             and summary['max_truth_translation_m'] <= 0.01)
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary), flush=True)
        return 0 if summary['passed'] else 1
    finally:
        publisher.publish(Twist())
        (args.output / 'samples.json').write_text(json.dumps(samples, indent=2))
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(bridge.pid, signal.SIGINT)
            bridge.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(bridge.pid, signal.SIGKILL)
            bridge.wait()
        bridge_log.close()


if __name__ == '__main__':
    raise SystemExit(main())
