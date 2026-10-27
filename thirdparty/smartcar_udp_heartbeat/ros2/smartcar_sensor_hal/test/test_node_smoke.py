"""Node behavior under lightweight ROS interface stubs, not a ROS runtime test."""
from collections import Counter
import importlib.util
from pathlib import Path
import struct
import sys
import time
import types
import unittest
from unittest.mock import Mock, patch

from smartcar_sensor_hal.core import TimestampGuard
from smartcar_sensor_hal.sensor_protocol import compute_imu_crc32


BOOT = '0123456789abcdef'
DEVICE_ID = '0123456789ab'


def lease_status(active=True, **changes):
    values = dict(active=active, state='active' if active else 'discovering',
                  reason='subscribed' if active else 'no_offers', source_ip='10.0.0.2',
                  boot_id=BOOT, device_id=DEVICE_ID, session_id='0011223344556677',
                  sync_port=8889, lease_remaining_ms=6000 if active else 0, candidates=())
    values.update(changes)
    return types.SimpleNamespace(**values)


class FakeHeader:
    def __init__(self):
        self.stamp = types.SimpleNamespace(sec=0, nanosec=0)
        self.frame_id = ''


class FakeImu:
    def __init__(self):
        self.header = FakeHeader()
        self.orientation_covariance = [0.0] * 9
        self.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class FakeLaserScan:
    def __init__(self):
        self.header = FakeHeader()


class FakeDiagnosticArray:
    def __init__(self):
        self.header = FakeHeader()


class FakeDiagnosticStatus:
    OK, WARN, ERROR = 0, 1, 2


def fake_module(**attributes):
    result = types.ModuleType('stub')
    result.__dict__.update(attributes)
    return result


def load_node_with_stubs():
    stubs = {
        'rclpy': fake_module(),
        'rclpy.node': fake_module(Node=type('Node', (), {})),
        'rclpy.clock': fake_module(Clock=object),
        'rclpy.clock_type': fake_module(ClockType=types.SimpleNamespace(STEADY_TIME=1)),
        'rclpy.qos': fake_module(qos_profile_sensor_data=object()),
        'rcl_interfaces': fake_module(),
        'rcl_interfaces.msg': fake_module(SetParametersResult=lambda **kw: types.SimpleNamespace(**kw)),
        'sensor_msgs': fake_module(),
        'sensor_msgs.msg': fake_module(Imu=FakeImu, LaserScan=FakeLaserScan),
        'diagnostic_msgs': fake_module(),
        'diagnostic_msgs.msg': fake_module(DiagnosticArray=FakeDiagnosticArray,
                                          DiagnosticStatus=FakeDiagnosticStatus,
                                          KeyValue=lambda **kw: types.SimpleNamespace(**kw)),
    }
    path = Path(__file__).parents[1] / 'smartcar_sensor_hal' / 'node.py'
    specification = importlib.util.spec_from_file_location('smartcar_sensor_hal._node_smoke_copy', path)
    module = importlib.util.module_from_spec(specification)
    with patch.dict(sys.modules, stubs):
        specification.loader.exec_module(module)
    return module


class NodeSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_with_stubs()

    def node(self):
        node = object.__new__(self.module.SmartcarSensorHal)
        node._counts = Counter()
        node._configured = True
        node._lease_status = lease_status()
        node._endpoint_key = node._lease_key(node._lease_status)
        node._discovery = Mock()
        node._discovery.status.return_value = node._lease_status
        node._discovery.poll.return_value = node._lease_status
        node._discovery.counters = Counter()
        node._sync = None
        node._sock = Mock()
        node._sock.recvfrom.side_effect = BlockingIOError()
        node._next_sync_attempt_ns = 0
        node._assembler = Mock()
        node._authoritative_boot = None
        node._imu_last_seq = None
        node._last_data_ns = {}
        node._last_frame = None
        node._last_imu = None
        node._guard = TimestampGuard()
        node.get_logger = Mock(return_value=Mock())
        return node

    def ticking_node(self):
        node = self.node()
        node.get_parameter = lambda name: types.SimpleNamespace(value=False)
        node._read_ros_clock = Mock(return_value=(100, 100, 0))
        node._anchor = types.SimpleNamespace(observe=lambda *args: False, last_residual_ns=0)
        node.max_datagrams = 3
        node._recorder = None
        node._sock = Mock()
        node._sock.recvfrom.side_effect = BlockingIOError()
        return node

    def test_heartbeat_send_error_is_counted_not_raised(self):
        node = self.node()
        node._sock = Mock()
        node._sock.sendto.side_effect = BlockingIOError()
        node._heartbeat({'type': 'heartbeat', 'version': 1, 'boot_id': BOOT,
                         'seq': 0, 'uptime_us': 123}, ('10.0.0.2', 30000))
        self.assertEqual(node._counts['heartbeat_ack_errors'], 1)
        self.assertEqual(node._counts['heartbeat_acks'], 0)

    def test_runtime_sim_time_and_unapplied_configuration_are_rejected(self):
        node = self.node()
        self.assertFalse(node._on_parameters([types.SimpleNamespace(name='use_sim_time', value=True)]).successful)
        self.assertFalse(node._on_parameters([types.SimpleNamespace(name='device_ip', value='10.0.0.3')]).successful)
        self.assertTrue(node._on_parameters([types.SimpleNamespace(name='use_sim_time', value=False)]).successful)

    def test_imu_message_units_unknown_orientation_and_filter_delay(self):
        node = self.node()
        node._authoritative_boot = BOOT
        node._imu_last_seq = None
        node._last_data_ns = {}
        node.imu_filter_delay_us = 4850.0
        node.imu_frame = 'imu_link'
        node.accel_bias = [0.0] * 3
        node.gyro_bias = [0.0] * 3
        node.accel_covariance = [0.0] * 9
        node.gyro_covariance = [0.0] * 9
        node._stamp = Mock(return_value=1_790_000_000_123_456_789)
        node._imu_pub = Mock()
        raw = struct.pack('>7h', 0, 0, 16384, 0, 0, 0, 131)
        packet = {'type': 'imu_packet', 'version': 1, 'boot_id': BOOT,
                  'seq': 0, 'sample_us': 1_000_000, 'uncertainty_us': 100,
                  'read_end_us': 1_000_200, 'data_hex': raw.hex(),
                  'crc32': compute_imu_crc32(BOOT, 0, 1_000_000, 100, 1_000_200, raw)}
        node._imu(packet, 5_000_000_000)
        node._stamp.assert_called_once_with('imu', 995150.0, BOOT)
        message = node._imu_pub.publish.call_args.args[0]
        self.assertEqual(message.header.frame_id, 'imu_link')
        self.assertEqual(message.orientation_covariance[0], -1.0)
        self.assertAlmostEqual(message.linear_acceleration.z, 9.80665)
        self.assertAlmostEqual(message.angular_velocity.z, 0.017453292519943295)
        self.assertEqual(message.linear_acceleration_covariance, [0.0] * 9)
        self.assertEqual(node._counts['imu_published'], 1)

    def test_future_mapping_does_not_pollute_monotonic_guard(self):
        node = self.node()
        node._qualified_status = lambda: (True, None)
        node._sync = types.SimpleNamespace(mapper=types.SimpleNamespace(
            to_host_monotonic_ns=lambda *args: time.monotonic_ns() + 1_000_000_000))
        node.max_sensor_age_ns = 500_000_000
        node.max_future_stamp_ns = 20_000_000
        node._guard = TimestampGuard()
        node._guard.last_published['imu'] = 123
        self.assertIsNone(node._stamp('imu', 1_000_000, BOOT))
        self.assertEqual(node._guard.last_published, {'imu': 123})
        self.assertEqual(node._counts['future_timestamp_drops'], 1)

    def test_oversize_datagram_is_rejected_before_json(self):
        node = self.ticking_node()
        node._sync = types.SimpleNamespace(poll=lambda: types.SimpleNamespace(boot_id=BOOT))
        node._authoritative_boot = BOOT
        node._assembler = Mock()
        node.max_datagrams = 3
        node.device_ip = '10.0.0.2'
        node._recorder = None
        node._sock = Mock()
        node._sock.recvfrom.side_effect = [(b'x' * 1201, ('10.0.0.2', 30000)), BlockingIOError()]
        node._tick()
        self.assertEqual(node._counts['oversize_datagram_drops'], 1)
        node._assembler.invalidate.assert_called_once()

    def test_empty_ip_configuration_waits_for_discovery_without_sync_socket(self):
        node = self.node()
        node._configured = False
        node._parameter = lambda name, default: default
        node.add_on_set_parameters_callback = Mock()
        node._read_ros_clock = Mock(return_value=(100, 100, 0))
        node.create_publisher = Mock()
        node.create_timer = Mock()
        node._sock = None
        node._recorder = None
        discovery = Mock()
        discovery.status.return_value = lease_status(False)
        with patch.object(self.module, 'DiscoveryClient', return_value=discovery) as constructor, \
                patch.object(self.module, 'SyncClient') as sync_constructor, \
                patch.object(self.module, 'Clock'), \
                patch.object(self.module.socket, 'socket') as socket_constructor:
            node._configure()
        self.assertEqual(node.device_ip, '')
        self.assertIsNone(node._sync)
        sync_constructor.assert_not_called()
        constructor.assert_called_once_with(data_port=8888, device_id='', target_ip='',
                                            broadcast_address='255.255.255.255', port=8890)
        socket_constructor.return_value.bind.assert_called_once_with(('0.0.0.0', 8888))

    def test_undiscovered_tick_with_no_sync_is_safe(self):
        node = self.ticking_node()
        node._endpoint_key = None
        node._discovery.status.return_value = lease_status(False)
        node._discovery.poll.return_value = lease_status(False)
        with patch.object(self.module, 'SyncClient') as constructor:
            node._tick()
        constructor.assert_not_called()
        self.assertEqual(node._qualified_status(), (False, None))

    def test_active_lease_starts_sync_at_advertised_endpoint(self):
        node = self.node()
        node._endpoint_key = None
        node._guard.last_published['scan'] = 123
        with patch.object(self.module, 'SyncClient') as constructor:
            node._poll_discovery()
        self.assertEqual(constructor.call_args.args, ('10.0.0.2',))
        self.assertEqual(constructor.call_args.kwargs['port'], 8889)
        node._assembler.reset.assert_called_once()
        self.assertEqual(node._guard.last_published, {'scan': 123})

    def test_expired_lease_closes_sync_and_recovery_preserves_stamp_guard(self):
        node = self.node()
        old_sync = Mock()
        node._sync = old_sync
        node._authoritative_boot = BOOT
        node._imu_last_seq = 10
        node._last_data_ns['imu'] = 100
        node._guard.last_published['imu'] = 456
        expired = lease_status(False, state='expired', reason='lease_expired')
        node._discovery.status.return_value = expired
        node._discovery.poll.return_value = expired
        with patch.object(self.module, 'SyncClient') as constructor:
            node._poll_discovery()
            constructor.assert_not_called()
        old_sync.close.assert_called_once()
        self.assertIsNone(node._sync)
        self.assertIsNone(node._imu_last_seq)
        self.assertEqual(node._last_data_ns, {})
        new_lease = lease_status(source_ip='10.0.0.3', boot_id='f' * 16,
                                 session_id='8899aabbccddeeff')
        node._discovery.status.return_value = new_lease
        node._discovery.poll.return_value = new_lease
        with patch.object(self.module, 'SyncClient') as constructor:
            node._poll_discovery()
        self.assertEqual(constructor.call_args.args, ('10.0.0.3',))
        self.assertEqual(node._endpoint_key, node._lease_key(new_lease))
        self.assertIsNone(node._authoritative_boot)
        self.assertEqual(node._guard.last_published, {'imu': 456})

    def test_ip_boot_session_or_sync_port_change_resets_model(self):
        for change in ({'source_ip': '10.0.0.3'}, {'boot_id': 'f' * 16},
                       {'session_id': '8899aabbccddeeff'}, {'sync_port': 9999}):
            with self.subTest(change=change):
                node = self.node()
                old_sync = Mock()
                node._sync = old_sync
                new_lease = lease_status(**change)
                node._discovery.poll.return_value = new_lease
                node._discovery.status.return_value = new_lease
                with patch.object(self.module, 'SyncClient') as constructor:
                    node._poll_discovery()
                old_sync.close.assert_called_once()
                constructor.assert_called_once()
                node._assembler.reset.assert_called_once()

    def test_endpoint_change_discards_queued_datagrams_before_starting_sync(self):
        node = self.node()
        node._endpoint_key = None
        node._sock.recvfrom.side_effect = [(b'old', ('10.0.0.2', 30000)),
                                           (b'old2', ('10.0.0.2', 30000)), BlockingIOError()]
        def start_sync(*args, **kwargs):
            self.assertEqual(node._counts['subscription_queued_datagrams_discarded'], 2)
            return Mock()
        with patch.object(self.module, 'SyncClient', side_effect=start_sync):
            node._poll_discovery()
        self.assertEqual(node._sock.recvfrom.call_count, 3)

    def test_subscription_queue_drain_is_bounded_and_counts_errors(self):
        node = self.node()
        node._sock.recvfrom.side_effect = None
        node._sock.recvfrom.return_value = (b'old', ('10.0.0.2', 30000))
        node._drain_subscription_queue()
        self.assertEqual(node._sock.recvfrom.call_count, 256)
        self.assertEqual(node._counts['subscription_queued_datagrams_discarded'], 256)
        self.assertEqual(node._counts['subscription_drain_budget_exhausted'], 1)
        node._sock.recvfrom.side_effect = OSError('test receive error')
        node._drain_subscription_queue()
        self.assertEqual(node._counts['subscription_drain_errors'], 1)

    def test_ros_clock_step_with_active_lease_restarts_sync(self):
        node = self.ticking_node()
        old_sync = Mock()
        node._sync = old_sync
        node._anchor.observe = lambda *args: True
        node._guard.last_published['scan'] = 789
        new_sync = Mock()
        new_sync.poll.return_value = types.SimpleNamespace(boot_id=BOOT)
        with patch.object(self.module, 'SyncClient', return_value=new_sync):
            node._tick()
        old_sync.close.assert_called_once()
        new_sync.poll.assert_called_once()
        self.assertEqual(node._counts['ros_clock_jumps'], 1)
        self.assertEqual(node._guard.last_published, {'scan': 789})

    def test_expired_lease_rejects_data_and_heartbeat_without_ack(self):
        node = self.ticking_node()
        node._endpoint_key = None
        expired = lease_status(False, reason='lease_expired')
        node._discovery.poll.return_value = expired
        node._discovery.status.return_value = expired
        node._sock.recvfrom.side_effect = [(b'{}', ('10.0.0.2', 30000)), BlockingIOError()]
        node._tick()
        node._heartbeat({'type': 'heartbeat', 'version': 1, 'boot_id': BOOT,
                         'seq': 0, 'uptime_us': 123}, ('10.0.0.2', 30000))
        self.assertEqual(node._counts['inactive_subscription_drops'], 1)
        self.assertEqual(node._counts['unsubscribed_heartbeat_drops'], 1)
        node._sock.sendto.assert_not_called()

    def test_heartbeat_wrong_boot_or_ip_is_not_acknowledged(self):
        node = self.node()
        node._sock = Mock()
        for boot, ip in [('f' * 16, '10.0.0.2'), (BOOT, '10.0.0.3')]:
            node._heartbeat({'type': 'heartbeat', 'version': 1, 'boot_id': boot,
                             'seq': 0, 'uptime_us': 123}, (ip, 30000))
        self.assertEqual(node._counts['unsubscribed_heartbeat_drops'], 2)
        node._sock.sendto.assert_not_called()

    def test_waiting_diagnostics_without_sync_describe_discovery_reason(self):
        node = self.node()
        node._discovery.status.return_value = lease_status(False, state='busy', reason='subscriber_busy')
        node._last_diag_counts = Counter()
        node._last_diag_ns = time.monotonic_ns()
        node._ros_anchor_bracket_ns = 0
        node._ros_anchor_residual_ns = 0
        node._assembler.counters = Counter()
        node._recorder = None
        node.reference_scan_hz, node.reference_imu_hz = 10.0, 50.0
        node.imu_filter_delay_us = 4850.0
        node.imu_filter_delay_calibrated = False
        node.lidar_delay_calibrated = False
        node.device_id = ''
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: object()))
        node._diagnostic_pub = Mock()
        node._publish_diagnostics()
        status = node._diagnostic_pub.publish.call_args.args[0].status[0]
        self.assertEqual(status.level, FakeDiagnosticStatus.WARN)
        self.assertIn('subscriber_busy', status.message)
        self.assertIn('discovery.active', {item.key for item in status.values})


if __name__ == '__main__':
    unittest.main()
