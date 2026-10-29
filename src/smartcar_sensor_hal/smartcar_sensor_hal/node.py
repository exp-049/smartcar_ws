"""Live UDP-to-ROS 2 HAL. This node never creates or broadcasts TF."""
from __future__ import annotations

import base64
from collections import Counter
import ipaddress
import json
import math
from pathlib import Path
import queue
import re
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .clock_sync import ClockMapper, ClockNotReadyError, SyncClient
from .core import RosTimeAnchor, ScanAssembler, TimestampGuard, timestamp_window_error
from .discovery import DiscoveryClient
from .lidar_protocol import MAX_DATAGRAM_BYTES, decode_lidar_packet, load_json_message
from .sensor_protocol import decode_imu_packet


class JsonlRecorder:
    """Bounded asynchronous recorder; disk I/O does not block UDP draining."""
    def __init__(self, path):
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        self._file = target.open('a', encoding='utf-8')
        self._queue = queue.Queue(maxsize=2048)
        self._stop = threading.Event()
        self.dropped = 0
        self.error = ''
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, data, peer, arrival_ns):
        if self.error:
            self.dropped += 1
            return
        record = {'received_host_monotonic_ns': arrival_ns,
                  'source_ip': peer[0], 'source_port': peer[1],
                  'datagram_base64': base64.b64encode(data).decode('ascii')}
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    record = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                self._file.write(json.dumps(record, separators=(',', ':')) + '\n')
        except OSError as error:
            self.error = str(error)
        finally:
            self._file.close()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)


class SmartcarSensorHal(Node):
    def __init__(self):
        super().__init__('smartcar_sensor_hal')
        self._sock = None
        self._sync = None
        self._discovery = None
        self._lease_status = None
        self._endpoint_key = None
        self._next_sync_attempt_ns = 0
        self._recorder = None
        self._counts = Counter()
        self._last_diag_counts = Counter()
        self._last_diag_ns = time.monotonic_ns()
        self._last_data_ns = {}
        self._imu_last_seq = None
        self._authoritative_boot = None
        self._last_frame = None
        self._last_imu = None
        self._guard = TimestampGuard()
        self._configured = False
        try:
            self._configure()
        except Exception:
            self.close()
            raise

    def _parameter(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _vector(self, name, default, size):
        value = [float(item) for item in self._parameter(name, default)]
        if len(value) != size or not all(math.isfinite(item) for item in value):
            raise ValueError(f'{name} must contain {size} finite values')
        return value

    def _configure(self):
        if self._parameter('use_sim_time', False):
            raise ValueError('Live sensor HAL requires use_sim_time=false')
        self.add_on_set_parameters_callback(self._on_parameters)
        self.device_ip = str(self._parameter('device_ip', '')).strip()
        if self.device_ip and ipaddress.ip_address(self.device_ip).version != 4:
            raise ValueError('device_ip must be IPv4')
        self.device_id = str(self._parameter('device_id', '')).strip().lower()
        if self.device_id and not re.fullmatch(r'[0-9a-f]{12}', self.device_id):
            raise ValueError('device_id must be 12 hexadecimal digits, or empty for automatic selection')
        self.broadcast_address = str(self._parameter('broadcast_address', '255.255.255.255'))
        if ipaddress.ip_address(self.broadcast_address).version != 4:
            raise ValueError('broadcast_address must be IPv4')
        self.bind_address = str(self._parameter('bind_address', '0.0.0.0'))
        self.data_port = int(self._parameter('data_port', 8888))
        self.discovery_port = int(self._parameter('discovery_port', 8890))
        if not (1024 <= self.data_port <= 65535 and 1024 <= self.discovery_port <= 65535):
            raise ValueError('UDP ports must be 1024..65535')
        if self.data_port == self.discovery_port:
            raise ValueError('data_port and discovery_port must be distinct')
        self.enable_lidar = bool(self._parameter('enable_lidar', True))
        self.enable_imu = bool(self._parameter('enable_imu', True))
        self.scan_frame = str(self._parameter('scan_frame_id', 'laser_link'))
        self.imu_frame = str(self._parameter('imu_frame_id', 'imu_link'))
        if not self.scan_frame or not self.imu_frame or self.scan_frame.startswith('/') or self.imu_frame.startswith('/'):
            raise ValueError('frame IDs must be nonempty and have no leading slash')
        axis_permutation = self._parameter('imu_axis_permutation', [0, 1, 2])
        if (not isinstance(axis_permutation, (list, tuple)) or len(axis_permutation) != 3 or
                any(type(axis) is not int for axis in axis_permutation) or
                sorted(axis_permutation) != [0, 1, 2]):
            raise ValueError('imu_axis_permutation must be a permutation of [0, 1, 2]')
        self.imu_axis_permutation = tuple(axis_permutation)
        axis_signs = self._parameter('imu_axis_signs', [1, 1, 1])
        if (not isinstance(axis_signs, (list, tuple)) or len(axis_signs) != 3 or
                any(type(sign) is not int or sign not in (-1, 1) for sign in axis_signs)):
            raise ValueError('imu_axis_signs must contain three values, each -1 or 1')
        self.imu_axis_signs = tuple(axis_signs)
        self.range_min = float(self._parameter('range_min_m', 0.12))
        self.range_max = float(self._parameter('range_max_m', 8.0))
        if not 0 <= self.range_min < self.range_max or not math.isfinite(self.range_max):
            raise ValueError('invalid scan range limits')
        self.reference_scan_hz = float(self._parameter('reference_scan_hz', 10.0))
        self.reference_imu_hz = float(self._parameter('reference_imu_hz', 50.0))
        self.imu_filter_delay_us = float(self._parameter('imu_filter_delay_us', 4850.0))
        self.imu_filter_delay_calibrated = bool(self._parameter('imu_filter_delay_calibrated', False))
        self.lidar_delay_calibrated = bool(self._parameter('lidar_device_delay_calibrated', False))
        self.max_sync_uncertainty_ns = int(float(self._parameter('max_sync_uncertainty_ms', 10.0)) * 1e6)
        self.max_interpolation_error_us = float(self._parameter('max_scan_interpolation_error_ms', 10.0)) * 1000.0
        self.max_sensor_age_ns = int(float(self._parameter('max_sensor_age_s', 0.5)) * 1e9)
        self.max_future_stamp_ns = int(float(self._parameter('max_future_stamp_ms', 20.0)) * 1e6)
        if not math.isfinite(self.imu_filter_delay_us) or self.imu_filter_delay_us < 0 or self.max_sync_uncertainty_ns <= 0:
            raise ValueError('invalid timing parameters')
        if (self.max_sensor_age_ns <= 0 or self.max_future_stamp_ns < 0 or
                not math.isfinite(self.max_interpolation_error_us) or self.max_interpolation_error_us <= 0 or
                not math.isfinite(self.reference_scan_hz) or self.reference_scan_hz <= 0 or
                not math.isfinite(self.reference_imu_hz) or self.reference_imu_hz <= 0):
            raise ValueError('invalid timing/rate limits')
        self.accel_bias = self._vector('imu_acceleration_bias_m_s2', [0.0] * 3, 3)
        self.gyro_bias = self._vector('imu_angular_velocity_bias_rad_s', [0.0] * 3, 3)
        self.accel_covariance = self._vector('imu_acceleration_covariance', [0.0] * 9, 9)
        self.gyro_covariance = self._vector('imu_angular_velocity_covariance', [0.0] * 9, 9)
        for covariance in (self.accel_covariance, self.gyro_covariance):
            if any(covariance[index] < 0 for index in (0, 4, 8)):
                raise ValueError('measurement covariance diagonal cannot be negative')
        self._assembler = ScanAssembler(
            sample_rate_hz=float(self._parameter('lidar_sample_rate_hz', 4000.0)),
            baudrate=int(self._parameter('lidar_baudrate', 115200)),
            device_delay_us=float(self._parameter('lidar_device_delay_us', 0.0)),
            scan_bins=int(self._parameter('scan_bins', 0)),
            correction=str(self._parameter('lidar_angle_correction', 'triangle')),
            yaw_offset_deg=float(self._parameter('laser_yaw_offset_deg', 0.0)),
            timeout_s=float(self._parameter('scan_timeout_s', 0.5)),
        )
        self.max_datagrams = int(self._parameter('max_datagrams_per_tick', 200))
        period = float(self._parameter('poll_period_s', 0.002))
        if not 1 <= self.max_datagrams <= 10000 or not 0.0005 <= period <= 0.05:
            raise ValueError('invalid receive budget or polling period')
        midpoint, ros_now, self._ros_anchor_bracket_ns = self._read_ros_clock()
        self._anchor = RosTimeAnchor(midpoint, ros_now)
        self._ros_anchor_residual_ns = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._sock.bind((self.bind_address, self.data_port))
        self._discovery = DiscoveryClient(
            data_port=self.data_port, device_id=self.device_id, target_ip=self.device_ip,
            broadcast_address=self.broadcast_address, port=self.discovery_port,
        )
        self._lease_status = self._discovery.status()
        path = str(self._parameter('record_jsonl_path', ''))
        if path:
            self._recorder = JsonlRecorder(path)
        self._scan_pub = self.create_publisher(LaserScan, '/car/scan', qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, '/car/imu/data_raw', qos_profile_sensor_data)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # Polling must continue even when wall/ROS time steps backward.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self._tick, clock=self._steady_clock)
        self.create_timer(1.0, self._publish_diagnostics, clock=self._steady_clock)
        self._configured = True
        self.get_logger().info(
            f'UDP {self.bind_address}:{self.data_port}; discovery={self.discovery_port}; '
            f'target={self.device_ip or "automatic"}; device_id={self.device_id or "automatic"}; '
            'waiting for sensor subscription and qualified time synchronization')

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'use_sim_time' and bool(parameter.value):
                return SetParametersResult(successful=False, reason='Live HAL rejects use_sim_time=true')
            if self._configured and parameter.name != 'use_sim_time':
                return SetParametersResult(successful=False, reason='Edit YAML and restart HAL to change transport/sensor parameters')
        return SetParametersResult(successful=True)

    def _read_ros_clock(self):
        before = time.monotonic_ns()
        ros_now = self.get_clock().now().nanoseconds
        after = time.monotonic_ns()
        return (before + after) // 2, ros_now, after - before

    @staticmethod
    def _lease_key(status):
        return (status.source_ip, status.boot_id, status.device_id, status.session_id, status.sync_port)

    def _active_lease(self):
        # status() has no I/O and checks the deadline, including while draining UDP.
        status = self._discovery.status() if self._discovery is not None else None
        if status is None or not status.active or self._lease_key(status) != self._endpoint_key:
            return None
        return status

    def _reset_streams(self):
        self._authoritative_boot = None
        self._imu_last_seq = None
        self._assembler.reset()
        self._last_data_ns.clear()
        self._last_frame = None
        self._last_imu = None

    def _ensure_sync(self):
        lease = self._active_lease()
        now = time.monotonic_ns()
        if lease is None or self._sync is not None or now < self._next_sync_attempt_ns:
            return
        self._next_sync_attempt_ns = now + 1_000_000_000
        try:
            self._sync = SyncClient(lease.source_ip, port=lease.sync_port,
                                    mapper=ClockMapper(), interval_s=0.5)
        except OSError as error:
            self._counts['sync_socket_errors'] += 1
            self._last_error = str(error)

    def _drain_subscription_queue(self):
        # This only clears already queued packets. The v1 data envelope has no
        # session ID, so packets still in flight remain subject to age/boot gates.
        for _ in range(256):
            try:
                self._sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError as error:
                self._counts['subscription_drain_errors'] += 1
                self._last_error = str(error)
                return
            self._counts['subscription_queued_datagrams_discarded'] += 1
        self._counts['subscription_drain_budget_exhausted'] += 1

    def _poll_discovery(self):
        self._lease_status = self._discovery.poll()
        key = self._lease_key(self._lease_status) if self._lease_status.active else None
        if key != self._endpoint_key:
            if self._sync is not None:
                self._sync.close()
                self._sync = None
            self._reset_streams()
            self._drain_subscription_queue()
            self._endpoint_key = key
            self._next_sync_attempt_ns = 0
            self._counts['subscription_transitions'] += 1
            if key is not None:
                self.get_logger().info(
                    f'Subscribed to {self._lease_status.device_id}@{self._lease_status.source_ip}; '
                    'waiting for qualified time synchronization')
        self._ensure_sync()

    def _restart_sync(self, reason, reset_ros_anchor=False):
        self._counts['sync_restarts'] += 1
        if self._sync is not None:
            self._sync.close()
            self._sync = None
        self._reset_streams()
        self._next_sync_attempt_ns = 0
        self._ensure_sync()
        if reset_ros_anchor:
            midpoint, ros_now, self._ros_anchor_bracket_ns = self._read_ros_clock()
            self._anchor = RosTimeAnchor(midpoint, ros_now)
        # Keep historical published stamps: clearing them could publish backward
        # timestamps after a backward system/ROS-clock step.
        self.get_logger().warning(f'Publication paused; resynchronizing: {reason}')

    def _qualified_status(self):
        if self._sync is None:
            return False, None
        status = self._sync.mapper.status()
        lease = self._active_lease()
        return (lease is not None and status.boot_id == lease.boot_id and
                status.locked and status.uncertainty_ns is not None and
                status.uncertainty_ns + self._ros_anchor_bracket_ns // 2 <= self.max_sync_uncertainty_ns), status

    def _stamp(self, stream, sample_us, boot_id):
        qualified, status = self._qualified_status()
        if not qualified:
            self._counts['timestamp_unqualified_drops'] += 1
            return None
        if sample_us < 0:
            self._counts['negative_sample_time_drops'] += 1
            return None
        try:
            host_ns = self._sync.mapper.to_host_monotonic_ns(int(round(sample_us)), boot_id)
        except ClockNotReadyError:
            self._counts['timestamp_mapping_drops'] += 1
            return None
        timing_error = timestamp_window_error(host_ns, time.monotonic_ns(),
                                              self.max_sensor_age_ns, self.max_future_stamp_ns)
        if timing_error:
            self._counts[f'{timing_error}_timestamp_drops'] += 1
            return None
        ros_ns = self._anchor.to_ros_ns(host_ns)
        if ros_ns < 0 or not self._guard.accept(stream, ros_ns):
            self._counts['nonmonotonic_timestamp_drops'] += 1
            self._restart_sync(f'{stream} timestamp did not advance')
            return None
        return ros_ns

    def _tick(self):
        if bool(self.get_parameter('use_sim_time').value):
            self._counts['sim_time_rejected_ticks'] += 1
            return
        midpoint, ros_now, self._ros_anchor_bracket_ns = self._read_ros_clock()
        clock_jump = self._anchor.observe(midpoint, ros_now)
        self._ros_anchor_residual_ns = self._anchor.last_residual_ns
        self._poll_discovery()
        if clock_jump:
            self._counts['ros_clock_jumps'] += 1
            self._restart_sync('ROS/system clock step >50 ms')
        if self._sync is not None:
            status = self._sync.poll()  # Timestamp replies immediately after recvfrom.
            lease = self._active_lease()
            if lease is not None and status.boot_id is not None:
                if status.boot_id != lease.boot_id:
                    self._counts['sync_lease_boot_mismatch'] += 1
                    self._restart_sync('sync boot differs from current subscription')
                elif status.boot_id != self._authoritative_boot:
                    self._authoritative_boot = status.boot_id
                    self._assembler.reset()
                    self._imu_last_seq = None
                    self._counts['authoritative_boot_changes'] += 1
        self._assembler.expire(time.monotonic_ns())
        for _ in range(self.max_datagrams):
            try:
                data, peer = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as error:
                self._counts['socket_errors'] += 1
                self.get_logger().error(f'UDP receive error: {error}')
                break
            received_ns = time.monotonic_ns()
            lease = self._active_lease()
            if lease is None:
                self._counts['inactive_subscription_drops'] += 1
                continue
            if peer[0] != lease.source_ip:
                self._counts['wrong_source_drops'] += 1
                continue
            self._counts['datagrams'] += 1
            if self._recorder:
                self._recorder.submit(data, peer, received_ns)
            if len(data) > MAX_DATAGRAM_BYTES:
                self._counts['oversize_datagram_drops'] += 1
                self._assembler.invalidate()
                continue
            kind = None
            try:
                packet = load_json_message(data)
                kind = packet.get('type')
                if kind == 'heartbeat':
                    self._heartbeat(packet, peer)
                elif kind == 'lidar_packet' and self.enable_lidar:
                    self._lidar(packet, received_ns)
                elif kind == 'imu_packet' and self.enable_imu:
                    self._imu(packet, received_ns)
                else:
                    self._counts['unknown_or_disabled_datagrams'] += 1
            except (ValueError, UnicodeError, OverflowError, RecursionError) as error:
                self._counts['invalid_datagrams'] += 1
                if kind in (None, 'lidar_packet'):
                    self._assembler.invalidate()
                self._last_error = str(error)
        else:
            self._counts['receive_budget_exhausted'] += 1

    def _heartbeat(self, packet, peer):
        boot = packet.get('boot_id')
        sequence = packet.get('seq')
        if (type(packet.get('version')) is not int or packet['version'] != 1 or
                not isinstance(boot, str) or not re.fullmatch(r'[0-9a-f]{16}', boot) or
                type(sequence) is not int or not 0 <= sequence <= 0xFFFFFFFF or
                type(packet.get('uptime_us')) is not int or not 0 <= packet['uptime_us'] <= 0x7FFFFFFFFFFFFFFF):
            raise ValueError('invalid heartbeat')
        lease = self._active_lease()
        if lease is None or peer[0] != lease.source_ip or boot != lease.boot_id:
            self._counts['unsubscribed_heartbeat_drops'] += 1
            return
        try:
            self._sock.sendto(f'ACK {boot} {sequence}'.encode('ascii'), peer)
            self._counts['heartbeat_acks'] += 1
        except OSError:
            self._counts['heartbeat_ack_errors'] += 1

    def _same_authoritative_boot(self, boot_id):
        lease = self._active_lease()
        if lease is None or boot_id != lease.boot_id or boot_id != self._authoritative_boot:
            self._counts['unsynchronized_boot_drops'] += 1
            return False
        return True

    def _lidar(self, packet, received_ns):
        parsed = decode_lidar_packet(packet, self.range_min, self.range_max)
        self._counts['valid_lidar_packets'] += 1
        self._last_data_ns['lidar'] = received_ns
        if not self._same_authoritative_boot(parsed.boot_id):
            return
        frame = self._assembler.push(parsed, received_ns)
        if frame is None:
            return
        self._last_frame = frame
        stamp = self._stamp('scan', frame.first_sample_us, frame.boot_id)
        if stamp is None:
            return
        status = self._sync.mapper.status()
        scale = status.scale_ns_per_us / 1000.0
        message = LaserScan()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = self.scan_frame
        message.angle_min = frame.angle_min
        message.angle_max = frame.angle_max
        message.angle_increment = frame.angle_increment
        message.scan_time = frame.scan_time_s * scale
        message.time_increment = frame.time_increment * scale
        message.range_min = self.range_min
        message.range_max = self.range_max
        message.ranges = list(frame.ranges_m)
        message.intensities = []
        self._scan_pub.publish(message)
        self._counts['scan_published'] += 1

    def _imu(self, packet, received_ns):
        parsed = decode_imu_packet(packet)
        self._counts['valid_imu_packets'] += 1
        self._last_data_ns['imu'] = received_ns
        if not self._same_authoritative_boot(parsed.boot_id):
            return
        if self._imu_last_seq is not None:
            delta = (parsed.seq - self._imu_last_seq) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                self._counts['imu_duplicate_or_out_of_order_drops'] += 1
                return
            self._counts['imu_sequence_gap_packets'] += delta - 1
        self._imu_last_seq = parsed.seq
        self._last_imu = parsed
        stamp = self._stamp('imu', parsed.sample_us - self.imu_filter_delay_us, parsed.boot_id)
        if stamp is None:
            return
        message = Imu()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id = self.imu_frame
        message.orientation_covariance[0] = -1.0
        axes = getattr(self, 'imu_axis_permutation', (0, 1, 2))
        signs = getattr(self, 'imu_axis_signs', (1, 1, 1))
        acceleration = tuple(signs[axis] * parsed.acceleration_m_s2[index]
                             for axis, index in enumerate(axes))
        angular_velocity = tuple(signs[axis] * parsed.angular_velocity_rad_s[index]
                                 for axis, index in enumerate(axes))
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = (
            value - bias for value, bias in zip(acceleration, self.accel_bias))
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = (
            value - bias for value, bias in zip(angular_velocity, self.gyro_bias))
        message.linear_acceleration_covariance = [
            signs[row] * signs[column] * self.accel_covariance[3 * axes[row] + axes[column]]
            for row in range(3) for column in range(3)]
        message.angular_velocity_covariance = [
            signs[row] * signs[column] * self.gyro_covariance[3 * axes[row] + axes[column]]
            for row in range(3) for column in range(3)]
        self._imu_pub.publish(message)
        self._counts['imu_published'] += 1

    def _publish_diagnostics(self):
        now = time.monotonic_ns()
        period = max((now - self._last_diag_ns) / 1e9, 1e-6)
        qualified, status = self._qualified_status()
        values = {key: str(value) for key, value in sorted(self._counts.items())}
        scan_hz = (self._counts['scan_published'] - self._last_diag_counts['scan_published']) / period
        imu_hz = (self._counts['imu_published'] - self._last_diag_counts['imu_published']) / period
        values.update({'scan_hz': f'{scan_hz:.3f}', 'imu_hz': f'{imu_hz:.3f}',
                       'reference_scan_hz': str(self.reference_scan_hz),
                       'reference_imu_hz': str(self.reference_imu_hz),
                       'scan_timestamp_mode': 'estimated UART backprojection + uniform-angle/time rebin',
                       'imu_timestamp_mode': 'data-ready polling midpoint minus DLPF3 estimate',
                       'imu_filter_delay_us': str(self.imu_filter_delay_us),
                       'imu_filter_delay_calibrated': str(self.imu_filter_delay_calibrated),
                       'lidar_device_delay_calibrated': str(self.lidar_delay_calibrated),
                       'imu_filter_phase_uncertainty': 'uncharacterized unless delay is calibrated',
                       'ros_anchor.residual_ns': str(self._ros_anchor_residual_ns),
                       'ros_anchor.sampling_bracket_ns': str(self._ros_anchor_bracket_ns),
                       'ros_anchor.sampling_uncertainty_ns': str(self._ros_anchor_bracket_ns // 2),
                       'last_error': getattr(self, '_last_error', '')})
        if status is not None and status.uncertainty_ns is not None:
            values['estimated_clock_uncertainty_ns'] = str(status.uncertainty_ns + self._ros_anchor_bracket_ns // 2)
        for name in ('locked', 'reason', 'boot_id', 'samples', 'rtt_ns', 'offset_ns', 'drift_ppm',
                     'residual_ns', 'uncertainty_ns', 'accepted', 'rejected', 'restarts', 'last_rejection'):
            values[f'sync.{name}'] = str(getattr(status, name, None))
        lease = self._discovery.status() if self._discovery is not None else None
        for name in ('active', 'state', 'reason', 'source_ip', 'boot_id', 'device_id',
                     'session_id', 'sync_port', 'lease_remaining_ms', 'candidates'):
            values[f'discovery.{name}'] = str(getattr(lease, name, None))
        if self._discovery is not None:
            for name, value in self._discovery.counters.items():
                values[f'discovery.count.{name}'] = str(value)
        for name, value in self._assembler.counters.items():
            values[f'lidar.{name}'] = str(value)
        for sensor in ('lidar', 'imu'):
            last = self._last_data_ns.get(sensor)
            values[f'{sensor}.last_data_age_s'] = 'never' if last is None else f'{(now-last)/1e9:.3f}'
        if self._last_frame:
            values['lidar.raw_point_count'] = str(self._last_frame.raw_point_count)
            values['lidar.valid_point_count'] = str(self._last_frame.valid_point_count)
            values['lidar.coverage_deg'] = f'{self._last_frame.coverage_deg:.3f}'
            values['lidar.max_time_interpolation_error_us'] = f'{self._last_frame.max_time_interpolation_error_us:.1f}'
        if self._last_imu:
            values['imu.polling_uncertainty_us'] = str(self._last_imu.uncertainty_us)
            values['imu.read_latency_us'] = str(self._last_imu.read_end_us - self._last_imu.sample_us)
            values['imu.temperature_c'] = f'{self._last_imu.temperature_c:.2f}'
        if self._recorder:
            values['recorder.dropped'] = str(self._recorder.dropped)
            values['recorder.error'] = self._recorder.error
        diagnostic = DiagnosticStatus()
        diagnostic.name = 'smartcar_sensor_hal'
        diagnostic.hardware_id = getattr(lease, 'device_id', '') or self.device_id or 'undiscovered'
        if lease is None or not lease.active:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = (f'Waiting for sensor subscription: {getattr(lease, "state", "starting")}: '
                                  f'{getattr(lease, "reason", "not discovered")}')
        elif not qualified:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'Sensor publication blocked: time synchronization not qualified'
        elif self._last_frame and self._last_frame.max_time_interpolation_error_us > self.max_interpolation_error_us:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = 'Scan angle/time interpolation error exceeds configured warning threshold'
        elif self.enable_lidar and self._last_frame and self._last_frame.valid_point_count == 0:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = 'Scan has no valid ranges'
        elif ((self.enable_lidar and scan_hz < self.reference_scan_hz * 0.9) or
              (self.enable_imu and imu_hz < self.reference_imu_hz * 0.9)):
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = 'Measured publication rate below 90% reference; verify actual sensor rate'
        else:
            diagnostic.level = DiagnosticStatus.WARN if not (self.imu_filter_delay_calibrated and self.lidar_delay_calibrated) else DiagnosticStatus.OK
            diagnostic.message = 'Receiving sensors; acquisition timing remains estimated until delays are calibrated'
        diagnostic.values = [KeyValue(key=key, value=value) for key, value in sorted(values.items())]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [diagnostic]
        self._diagnostic_pub.publish(message)
        self._last_diag_ns = now
        self._last_diag_counts = self._counts.copy()

    def close(self):
        if self._discovery is not None:
            self._discovery.close()  # Best-effort release; do not wait for its ACK.
            self._discovery = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        if self._sync is not None:
            self._sync.close()
            self._sync = None
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SmartcarSensorHal()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
