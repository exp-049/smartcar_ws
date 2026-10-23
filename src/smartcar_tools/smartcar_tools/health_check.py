#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Health(Node):
    def __init__(self): super().__init__('smartcar_health_check')
    def run(self):
        required = ['/clock', '/car/scan', '/car/imu/data_raw', '/car/chassis/odom', '/map']
        topics = {name for name, _types in self.get_topic_names_and_types()}
        missing = [name for name in required if name not in topics]
        print('required topics: ' + ('OK' if not missing else 'MISSING ' + ', '.join(missing)))
        print('actions: ' + ('OK' if '/navigate_to_pose/_action/status' in topics else 'wait for Nav2'))
        return 0 if not missing else 1

def main():
    rclpy.init(); node = Health(); rclpy.spin_once(node, timeout_sec=1.0); code = node.run(); node.destroy_node(); rclpy.shutdown(); return code

if __name__ == '__main__': raise SystemExit(main())
