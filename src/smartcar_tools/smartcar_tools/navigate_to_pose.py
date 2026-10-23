#!/usr/bin/env python3
import argparse
import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformListener

class GoalClient(Node):
    def __init__(self, x, y, yaw):
        super().__init__('smartcar_goal_client')
        self.x, self.y, self.yaw = x, y, yaw
        self._last_feedback_log_ns = 0
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_handle = None

    def wait_until_active(self, timeout=40.0):
        # The action can be discovered while bt_navigator is still inactive.
        # Sending at that point is rejected even though wait_for_server passed.
        state_client = self.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic() + timeout
        self.get_logger().info('Waiting for Nav2 to become active...')
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if not state_client.wait_for_service(timeout_sec=0.5):
                    continue
                future = state_client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if future.done() and future.result() is not None:
                    if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                        return
                elif not future.done():
                    state_client.remove_pending_request(future)
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_client(state_client)
        raise RuntimeError('Nav2 bt_navigator did not become active within %.0f s' % timeout)

    def cancel(self):
        if self.goal_handle is None:
            return
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def wait_for_localization(self, timeout=15.0):
        # Start discovery after activation.  The action can be active before
        # its newly created TF listener has received its first map transform.
        # Wait on real data instead of sending immediately or sleeping blindly.
        buffer = Buffer()
        listener = TransformListener(buffer, self)
        maps = []
        local_maps = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscription = self.create_subscription(OccupancyGrid, '/map', maps.append, qos)
        local_subscription = self.create_subscription(Costmap, '/local_costmap/costmap_raw', local_maps.append, qos)
        deadline = time.monotonic() + timeout
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if maps and maps[-1].info.width and local_maps and buffer.can_transform(
                        'map', 'base_link', rclpy.time.Time()):
                    return
        finally:
            self.destroy_subscription(subscription)
            self.destroy_subscription(local_subscription)
            listener.unregister()
        raise RuntimeError('Map, local costmap or map -> base_link TF is not ready')

    def send(self):
        self.wait_until_active()
        self.wait_for_localization()
        # Discovery of a map in this process does not establish that the
        # independent guard has received it. Check that consumer explicitly.
        guard = self.create_client(IsPathValid, '/car/is_upcoming_path_valid')
        try:
            deadline = time.monotonic() + 15.0
            ready = False
            while time.monotonic() < deadline and rclpy.ok():
                if not guard.wait_for_service(timeout_sec=0.5):
                    continue
                request = IsPathValid.Request()
                request.path.header.frame_id = 'base_link'
                request.path.poses.append(PoseStamped())
                future = guard.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
                if future.done() and future.result() and future.result().is_valid:
                    ready = True
                    break
                if not future.done():
                    guard.remove_pending_request(future)
                rclpy.spin_once(self, timeout_sec=0.1)
            if not ready:
                raise RuntimeError('Path guard is not ready, or the robot is in collision')
        finally:
            self.destroy_client(guard)
        if not self.client.wait_for_server(timeout_sec=10.0): raise RuntimeError('Nav2 /navigate_to_pose action is unavailable')
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.x, self.y
        goal.pose.pose.orientation.z = math.sin(self.yaw / 2.0); goal.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        def on_feedback(feedback):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_feedback_log_ns >= 500_000_000:
                self._last_feedback_log_ns = now_ns
                self.get_logger().info(
                    'distance remaining: %.3f m' % feedback.feedback.distance_remaining)
        future = self.client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, future); handle = future.result()
        if handle is None or not handle.accepted: raise RuntimeError('目标被 Nav2 拒绝')
        self.goal_handle = handle
        result_future = handle.get_result_async(); rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status

def main(argv=None):
    parser = argparse.ArgumentParser(description='Send a map-frame Nav2 goal; the planner handles known and unknown space')
    parser.add_argument('x', type=float); parser.add_argument('y', type=float); parser.add_argument('--yaw', type=float, default=0.0)
    args = parser.parse_args(argv)
    # Keep the ROS context alive until the cancellation request is sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoalClient(args.x, args.y, args.yaw)
    try: status = node.send(); print('NavigateToPose finished with status', status); return 0 if status == 4 else 1
    except KeyboardInterrupt:
        if rclpy.ok():
            node.cancel()
        print('Navigation cancellation requested', file=sys.stderr)
        return 130
    except Exception as exc: print(str(exc), file=sys.stderr); return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': sys.exit(main())
