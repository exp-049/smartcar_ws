#!/usr/bin/env python3
import argparse
import pathlib
import shutil
import subprocess
import sys
import time
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node

class MapSaver(Node):
    def __init__(self):
        super().__init__('smartcar_map_saver')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
    def stop(self):
        msg = Twist()
        for _ in range(10): self.pub.publish(msg); rclpy.spin_once(self, timeout_sec=0.05)
    def cancel_navigation(self):
        client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if not client.wait_for_service(timeout_sec=1.0):
            return
        request = CancelGoal.Request()
        # A zero UUID and zero timestamp request cancellation of all goals.
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=10.0): raise RuntimeError('service unavailable: ' + name)
        future = client.call_async(request); rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None: raise RuntimeError('service failed: ' + name)
        if hasattr(response, 'status') and getattr(response.status, 'code', 0) != 0: raise RuntimeError('service status: ' + str(response.status))
        return response

def main(argv=None):
    parser = argparse.ArgumentParser(description='Finish Cartographer trajectory and save a pbstream plus ROS map')
    parser.add_argument('--name', default='demo'); parser.add_argument('--directory', default='/home/doc049/dev/smartcar_ws/maps'); parser.add_argument('--trajectory-id', type=int, default=0)
    args = parser.parse_args(argv); rclpy.init(); node = MapSaver()
    try:
        from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
        out_dir = pathlib.Path(args.directory); out_dir.mkdir(parents=True, exist_ok=True)
        prefix = out_dir / args.name; node.cancel_navigation(); node.stop(); time.sleep(0.2)
        req = FinishTrajectory.Request(); req.trajectory_id = args.trajectory_id; node.call(FinishTrajectory, '/finish_trajectory', req)
        req2 = WriteState.Request(); req2.filename = str(prefix) + '.pbstream'; req2.include_unfinished_submaps = True; node.call(WriteState, '/write_state', req2)
        converter = shutil.which('cartographer_pbstream_to_ros_map')
        if converter is None:
            # Debian ROS packages keep Cartographer's helper under the
            # package libexec directory instead of putting it on PATH.
            try:
                from ament_index_python.packages import get_package_prefix
                candidate = pathlib.Path(get_package_prefix('cartographer_ros')) / 'lib' / 'cartographer_ros' / 'cartographer_pbstream_to_ros_map'
                if candidate.is_file() and candidate.stat().st_mode & 0o111:
                    converter = str(candidate)
            except Exception:
                converter = None
        if converter is None:
            raise RuntimeError('cartographer_pbstream_to_ros_map executable is unavailable')
        cmd = [converter, '-pbstream_filename', str(prefix) + '.pbstream', '-map_filestem', str(prefix), '-resolution', '0.025']
        subprocess.run(cmd, check=True)
        print('saved', prefix.with_suffix('.pbstream')); return 0
    except Exception as exc: print('save_map failed:', exc, file=sys.stderr); return 2
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': sys.exit(main())
