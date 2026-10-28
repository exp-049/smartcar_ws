"""Shared freshness, frame, time and TF checks for startup and command output."""
import math
import time

from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def finite_vector(vector):
    return all(math.isfinite(getattr(vector, axis)) for axis in ('x', 'y', 'z'))


def valid_sensor(key, msg):
    """Validate the public data contract without treating no lidar return as an error."""
    frames = {'scan': 'laser_link', 'imu': 'imu_link', 'odom': 'chassis_odom'}
    if msg.header.frame_id != frames[key] or stamp_seconds(msg.header.stamp) <= 0:
        return False
    if key == 'scan':
        # A negative increment is a valid counter-clockwise sweep: the HAL emits
        # angle_min..angle_max descending. Only a zero/degenerate step is invalid.
        return (len(msg.ranges) > 0 and math.isfinite(msg.angle_increment)
                and abs(msg.angle_increment) > 0.0 and 0 <= msg.range_min < msg.range_max
                and any(not math.isnan(item) for item in msg.ranges))
    if key == 'imu':
        return finite_vector(msg.angular_velocity) and finite_vector(msg.linear_acceleration)
    q = msg.pose.pose.orientation
    norm = sum(getattr(q, axis) ** 2 for axis in ('x', 'y', 'z', 'w'))
    return (msg.child_frame_id == 'base_link' and finite_vector(msg.pose.pose.position)
            and finite_vector(msg.twist.twist.linear) and finite_vector(msg.twist.twist.angular)
            and math.isfinite(norm) and abs(norm - 1.0) < 0.05)


class SystemHealth(Node):
    """Monitor real messages, not just advertised ROS topic names."""
    def __init__(self, name):
        super().__init__(name)
        self.sensor_timeout = self.declare_parameter('sensor_timeout_sec', 0.5).value
        self.tf_timeout = self.declare_parameter('transform_timeout_sec', 0.5).value
        if any(not math.isfinite(item) or item <= 0 for item in (self.sensor_timeout, self.tf_timeout)):
            raise ValueError('Health timeouts must be finite and positive')
        self.samples = {}
        self.received = {}
        self.timestamp_fault = ''
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(LaserScan, '/car/scan', lambda msg: self.receive('scan', msg), qos_profile_sensor_data)
        self.create_subscription(Imu, '/car/imu/data_raw', lambda msg: self.receive('imu', msg), qos_profile_sensor_data)
        # Best effort subscriptions accept both reliable and best effort drivers.
        self.create_subscription(Odometry, '/car/chassis/odom', lambda msg: self.receive('odom', msg), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.receive_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_valid = False

    def receive(self, key, msg):
        if not valid_sensor(key, msg):
            return
        previous = self.samples.get(key)
        if previous and stamp_seconds(msg.header.stamp) < stamp_seconds(previous.header.stamp):
            self.timestamp_fault = f'{key} timestamp moved backwards; restart the estimator/session'
            return
        self.samples[key] = msg
        self.received[key] = time.monotonic()

    def receive_map(self, msg):
        self.map_valid = (msg.header.frame_id == 'map' and msg.info.resolution > 0
                          and msg.info.width > 0 and msg.info.height > 0
                          and len(msg.data) == msg.info.width * msg.info.height)

    def problems(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        problems = []
        if now <= 0:
            problems.append('ROS clock is uninitialized')
        if self.timestamp_fault:
            problems.append(self.timestamp_fault)
        for key in ('scan', 'imu', 'odom'):
            msg = self.samples.get(key)
            if msg is None or wall - self.received.get(key, 0.0) > self.sensor_timeout:
                problems.append(f'{key} missing, invalid or stale')
            else:
                # A header stamp is the acquisition instant, so it legitimately
                # trails ROS now by the transport and processing latency: the UDP
                # lidar measures about 0.115 s. Reject only stamps from the future
                # (beyond a small tolerance for clock jitter) and stamps too old to
                # trust; the previous -0.1 s floor sat below the real latency and
                # rejected every scan, which latched command_guard permanently.
                age = now - stamp_seconds(msg.header.stamp)
                if not -0.02 <= age <= max(self.sensor_timeout, 1.0):
                    problems.append(f'{key} acquisition time is not aligned with ROS time')
        if not self.map_valid:
            problems.append('valid map is unavailable')
        for child in ('laser_link', 'imu_link'):
            key = 'scan' if child == 'laser_link' else 'imu'
            if key in self.samples and not self.buffer.can_transform(
                    'base_link', child, Time.from_msg(self.samples[key].header.stamp)):
                problems.append(f'base_link -> {child} TF unavailable at acquisition time')
        for parent, child in (('map', 'base_link'), ('chassis_odom', 'base_link')):
            try:
                transform = self.buffer.lookup_transform(parent, child, Time())
                if not -0.1 <= now - stamp_seconds(transform.header.stamp) <= self.tf_timeout:
                    problems.append(f'{parent} -> {child} TF is stale')
            except Exception:
                problems.append(f'{parent} -> {child} TF unavailable')
        return problems
