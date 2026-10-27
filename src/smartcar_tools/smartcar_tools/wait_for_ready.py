"""Exit successfully only after the sensors, time, TF and map are usable."""
import math
import time

import rclpy
from smartcar_tools.system_health import SystemHealth


def main():
    rclpy.init()
    node = SystemHealth('wait_for_ready')
    timeout = node.declare_parameter('timeout_sec', 120.0).value
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout_sec must be finite and positive')
    deadline = time.monotonic() + timeout
    stable_since = None
    next_log = 0.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            problems = node.problems()
            now = time.monotonic()
            if problems:
                stable_since = None
                if now >= next_log:
                    node.get_logger().info('Waiting: ' + '; '.join(problems))
                    next_log = now + 5.0
            else:
                stable_since = stable_since or now
                if now - stable_since >= 0.5:
                    node.get_logger().info('Sensors, ROS time, TF and map ready; navigation may start')
                    return 0
        node.get_logger().error('Readiness timed out: ' + '; '.join(node.problems()))
        return 1
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
