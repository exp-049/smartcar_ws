"""Stop forwarding Cartographer's predicted TF when its sensor stream is stale."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


_TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
_TF_STATIC_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CartographerTfGuard(Node):
    """Relay external TF into Cartographer and gate its output on fresh sensors."""

    def __init__(self):
        super().__init__('cartographer_tf_guard')
        self.timeout = float(self.declare_parameter('stale_timeout_sec', 0.3).value)
        self.require_imu = bool(self.declare_parameter('require_imu', True).value)
        self.require_odom = bool(self.declare_parameter('require_odom', False).value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'chassis_odom').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise ValueError('stale_timeout_sec must be finite and positive')

        self._last_rx = {}
        self._stale = True
        self._cartographer_tf_pub = self.create_publisher(
            TFMessage, '/cartographer_internal/tf', _TF_QOS)
        self._cartographer_static_tf_pub = self.create_publisher(
            TFMessage, '/cartographer_internal/tf_static', _TF_STATIC_QOS)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(TFMessage, '/tf', self._relay_system_tf, _TF_QOS)
        self.create_subscription(TFMessage, '/tf_static', self._relay_system_static_tf,
                                 _TF_STATIC_QOS)
        self.create_subscription(TFMessage, '/cartographer_internal/tf',
                                 self._gate_cartographer_tf, _TF_QOS)
        self.create_subscription(LaserScan, '/car/scan', self._on_scan,
                                 qos_profile_sensor_data)
        if self.require_imu:
            self.create_subscription(Imu, '/car/imu/data_raw', self._on_imu,
                                     qos_profile_sensor_data)
        if self.require_odom:
            self.create_subscription(Odometry, '/car/chassis/odom', self._on_odom,
                                     qos_profile_sensor_data)

        self.get_logger().info(
            f'Cartographer TF stale guard enabled: timeout={self.timeout:.3f}s, '
            f'require_imu={self.require_imu}, require_odom={self.require_odom}')

    def _mark_fresh(self, key):
        self._last_rx[key] = time.monotonic()

    def _on_scan(self, msg):
        if msg.header.frame_id == 'laser_link' and msg.ranges:
            self._mark_fresh('scan')

    def _on_imu(self, msg):
        if msg.header.frame_id == 'base_link' and all(math.isfinite(value) for value in (
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y,
                msg.linear_acceleration.z)):
            self._mark_fresh('imu')

    def _on_odom(self, msg):
        if msg.header.frame_id == self.odom_frame and msg.child_frame_id == 'base_link':
            self._mark_fresh('odom')

    def _sensors_fresh(self):
        now = time.monotonic()
        required = ['scan']
        if self.require_imu:
            required.append('imu')
        if self.require_odom:
            required.append('odom')
        return all(key in self._last_rx and now - self._last_rx[key] <= self.timeout
                   for key in required)

    def _is_estimator_tf(self, transform):
        return (transform.header.frame_id == self.map_frame
                or transform.child_frame_id == self.map_frame
                or transform.header.frame_id == self.odom_frame
                or transform.child_frame_id == self.odom_frame)

    def _relay_system_tf(self, msg):
        # Keep Cartographer's own map/odom transforms out of its input relay.
        transforms = [transform for transform in msg.transforms
                      if not self._is_estimator_tf(transform)]
        if transforms:
            self._cartographer_tf_pub.publish(TFMessage(transforms=transforms))

    def _relay_system_static_tf(self, msg):
        if msg.transforms:
            self._cartographer_static_tf_pub.publish(msg)

    def _gate_cartographer_tf(self, msg):
        transforms = [transform for transform in msg.transforms
                      if self._is_estimator_tf(transform)]
        if not transforms:
            return
        fresh = self._sensors_fresh()
        if not fresh:
            if not self._stale:
                self.get_logger().warning(
                    f'Cartographer sensor data stale for >{self.timeout:.3f}s; '
                    'freezing the published pose at its last valid TF')
            self._stale = True
            return
        if self._stale:
            self.get_logger().info('Cartographer sensor data recovered; resuming pose TF')
        self._stale = False
        self._tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerTfGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
