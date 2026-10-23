"""Check the upcoming path against live local obstacles, not SLAM raster churn."""
import math
import time

import rclpy
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import IsPathValid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def segment_cells(a, b, resolution):
    steps = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (resolution * 0.5)))
    for i in range(steps + 1):
        yield (a[0] + (b[0] - a[0]) * i / steps,
               a[1] + (b[1] - a[1]) * i / steps)


class PathGuard(Node):
    def __init__(self):
        super().__init__('path_guard')
        self.confirmation_time = self.declare_parameter('confirmation_time', 0.6).value
        self.stale_timeout = self.declare_parameter('stale_timeout', 1.0).value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.costmap = None
        self.received_at = 0.0
        self.blocked_since = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.on_costmap, qos)
        self.create_service(IsPathValid, '/car/is_upcoming_path_valid', self.check)

    def on_costmap(self, msg):
        self.costmap = msg
        self.received_at = time.monotonic()

    def check(self, request, response):
        response.is_valid = False
        msg = self.costmap
        if not request.path.poses:
            return response
        if msg is None or time.monotonic() - self.received_at > self.stale_timeout:
            self.get_logger().warning('Path check: waiting for a fresh local costmap', throttle_duration_sec=5.0)
            return response
        if msg.metadata.resolution <= 0 or not msg.metadata.size_x or not msg.metadata.size_y:
            return response
        try:
            tf = self.buffer.lookup_transform(
                msg.header.frame_id, request.path.header.frame_id, rclpy.time.Time()).transform
        except Exception as exc:
            self.get_logger().warning('Path check: %s' % exc, throttle_duration_sec=5.0)
            return response
        angle = yaw(tf.rotation)
        c, s = math.cos(angle), math.sin(angle)
        metadata = msg.metadata
        origin = metadata.origin.position
        points = []
        for pose in request.path.poses:
            p = pose.pose.position
            points.append((c * p.x - s * p.y + tf.translation.x,
                           s * p.x + c * p.y + tf.translation.y))
        blocked = False
        blocked_point = None
        for a, b in zip(points, points[1:] or points):
            for x, y in segment_cells(a, b, metadata.resolution):
                ix = math.floor((x - origin.x) / metadata.resolution)
                iy = math.floor((y - origin.y) / metadata.resolution)
                if not (0 <= ix < metadata.size_x and 0 <= iy < metadata.size_y):
                    self.get_logger().warning('Upcoming path leaves the local costmap', throttle_duration_sec=5.0)
                    return response
                # Circular robot: local inflation already accounts for its
                # radius.  253/254 are collisions, 255 is unobserved space.
                if msg.data[iy * metadata.size_x + ix] >= 253:
                    blocked = True
                    blocked_point = (x, y)
                    break
            if blocked:
                break
        if not blocked:
            self.blocked_since = None
            response.is_valid = True
        else:
            now = time.monotonic()
            if self.blocked_since is None:
                self.blocked_since = now
            response.is_valid = now - self.blocked_since < self.confirmation_time
            if not response.is_valid:
                self.get_logger().warning(
                    'Upcoming path blocked near (%.3f, %.3f) in %s; requesting replan' %
                    (*blocked_point, msg.header.frame_id), throttle_duration_sec=2.0)
        return response


def main():
    rclpy.init()
    node = PathGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
