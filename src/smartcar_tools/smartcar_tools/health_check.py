"""Check data freshness, clock alignment, TF and map for sim or real."""
import time
import rclpy
from smartcar_tools.system_health import SystemHealth


def main():
    rclpy.init()
    node = SystemHealth('smartcar_health_check')
    try:
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        problems = node.problems()
        if problems:
            node.get_logger().error('; '.join(problems))
            return 1
        node.get_logger().info('Sensors, time, TF and map: OK')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
