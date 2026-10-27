"""Core regression checks runnable on macOS without ROS or hardware."""
import math
from types import SimpleNamespace
import unittest

from smartcar_sensor_hal.core import RosTimeAnchor, ScanAssembler, TimestampGuard, timestamp_window_error


BOOT = '0123456789abcdef'


def packet(sequence, frame, sector, *, boot=BOOT, offset=0.0):
    count = 5
    first_time = 1_000_000 + frame * 100_000 + sector * 12_500
    raw = b'\0' * (10 + count * 2)
    wire_time = len(raw) * 10.0 / 115200 * 1e6
    return SimpleNamespace(
        boot_id=boot, seq=sequence,
        rx_complete_us=round(first_time + (count - 1) * 2500 + wire_time),
        raw=raw, ct=1 if sector == 0 else 0, start_of_scan=sector == 0,
        point_count=count,
        angles_cw_deg=tuple((offset + sector * 45 + index * 9) % 360 for index in range(count)),
        ranges_m=tuple(1.0 + (sector * 5 + index) / 100 for index in range(count)),
    )


class AssemblyTests(unittest.TestCase):
    def make_assembler(self, **kwargs):
        return ScanAssembler(sample_rate_hz=400.0, correction='none', **kwargs)

    def full_frame(self, assembler, frame=0, first_sequence=0, offset=0.0):
        result = None
        for sector in range(8):
            result = assembler.push(packet(first_sequence + sector, frame, sector, offset=offset),
                                    int((frame + 1) * 1e9 + sector * 12_500_000))
            self.assertIsNone(result)
        return assembler.push(packet(first_sequence + 8, frame + 1, 0, offset=offset),
                              int((frame + 1) * 1e9 + 100_000_000))

    def test_complete_circle_preserves_clockwise_order_and_positive_time(self):
        frame = self.full_frame(self.make_assembler())
        self.assertIsNotNone(frame)
        self.assertEqual(frame.raw_point_count, 40)
        self.assertEqual(frame.valid_point_count, 40)
        self.assertEqual(frame.first_sample_us, 1_000_000)
        self.assertAlmostEqual(frame.scan_time_s, 0.1)
        self.assertLess(frame.angle_increment, 0)
        self.assertGreater(frame.angle_min, frame.angle_max)
        self.assertGreater(frame.time_increment, 0)
        self.assertAlmostEqual(frame.time_increment, 0.0025)
        self.assertAlmostEqual(frame.angle_max,
                               frame.angle_min + 39 * frame.angle_increment)
        self.assertEqual(frame.ranges_m, tuple(1.0 + index / 100 for index in range(40)))

    def test_initial_partial_circle_is_not_published(self):
        assembler = self.make_assembler()
        sequence = 0
        for sector in range(3, 8):
            self.assertIsNone(assembler.push(packet(sequence, 0, sector), sequence * 10_000_000))
            sequence += 1
        for sector in range(8):
            self.assertIsNone(assembler.push(packet(sequence, 1, sector), sequence * 10_000_000))
            sequence += 1
        self.assertIsNotNone(assembler.push(packet(sequence, 2, 0), sequence * 10_000_000))
        self.assertEqual(assembler.counters['initial_partial_packets'], 5)

    def test_packet_gap_drops_circle_and_next_complete_circle_recovers(self):
        assembler = self.make_assembler()
        for sector in (0, 1, 3, 4, 5, 6, 7):
            assembler.push(packet(sector, 0, sector), sector * 12_500_000)
        self.assertIsNone(assembler.push(packet(8, 1, 0), 100_000_000))
        for sector in range(1, 8):
            assembler.push(packet(8 + sector, 1, sector), 100_000_000 + sector * 12_500_000)
        self.assertIsNotNone(assembler.push(packet(16, 2, 0), 200_000_000))
        self.assertEqual(assembler.counters['sequence_gap_packets'], 1)
        self.assertEqual(assembler.counters['invalid_dropped_scans'], 1)

    def test_duplicate_does_not_rewind_sequence_or_publish_bad_circle(self):
        assembler = self.make_assembler()
        assembler.push(packet(0, 0, 0), 0)
        assembler.push(packet(1, 0, 1), 10_000_000)
        assembler.push(packet(1, 0, 1), 11_000_000)
        self.assertEqual(assembler.last_seq, 1)
        for sector in range(2, 8):
            assembler.push(packet(sector, 0, sector), sector * 12_500_000)
        self.assertIsNone(assembler.push(packet(8, 1, 0), 100_000_000))
        self.assertEqual(assembler.counters['duplicate_or_out_of_order_packets'], 1)

    def test_invalid_transport_marker_discards_pending_scan(self):
        assembler = self.make_assembler()
        for sector in range(8):
            assembler.push(packet(sector, 0, sector), sector * 12_500_000)
            if sector == 3:
                assembler.invalidate()
        self.assertIsNone(assembler.push(packet(8, 1, 0), 100_000_000))

    def test_timeout_does_not_publish_incomplete_circle(self):
        assembler = self.make_assembler()
        assembler.push(packet(0, 0, 0), 0)
        assembler.expire(600_000_000)
        self.assertIsNone(assembler.push(packet(8, 1, 0), 610_000_000))
        self.assertEqual(assembler.counters['timeout_dropped_scans'], 1)

    def test_boot_change_discards_previous_partial_circle(self):
        assembler = self.make_assembler()
        assembler.push(packet(0, 0, 0), 0)
        self.assertIsNone(assembler.push(packet(0, 1, 4, boot='fedcba9876543210'), 100_000_000))
        self.assertIsNone(assembler._samples)
        self.assertEqual(assembler.counters['boot_resets'], 1)

    def test_unwrap_across_zero_preserves_order(self):
        frame = self.full_frame(self.make_assembler(), offset=358.0)
        self.assertIsNotNone(frame)
        self.assertAlmostEqual(frame.coverage_deg, 351.0)
        self.assertAlmostEqual(frame.angle_min, math.radians(2.0))
        self.assertEqual(frame.ranges_m[0], 1.0)

    def test_distance_correction_before_scan_start_is_dropped_not_wrapped(self):
        assembler = ScanAssembler(sample_rate_hz=400.0, correction='triangle')
        first = packet(0, 0, 0)
        first.angles_cw_deg = (0.0, 1.0, 18.0, 27.0, 36.0)
        first.ranges_m = (None, 1.0, 1.02, 1.03, 1.04)
        assembler.push(first, 0)
        for sector in range(1, 8):
            assembler.push(packet(sector, 0, sector), sector * 12_500_000)
        frame = assembler.push(packet(8, 1, 0), 100_000_000)
        self.assertIsNotNone(frame)
        self.assertGreater(assembler.counters['corrected_outside_scan_points'], 0)
        self.assertTrue(math.isinf(frame.ranges_m[-1]))
        self.assertNotIn(1.0, frame.ranges_m)

    def test_nearest_bin_past_end_is_dropped_not_wrapped(self):
        assembler = self.make_assembler(scan_bins=8)
        frame = self.full_frame(assembler)
        self.assertGreater(assembler.counters['rounded_outside_scan_points'], 0)
        self.assertEqual(frame.ranges_m[0], 1.0)


class TimeTests(unittest.TestCase):
    def test_old_and_future_samples_are_rejected_without_affecting_guard(self):
        guard = TimestampGuard()
        self.assertEqual(timestamp_window_error(0, 1000, 500, 20), 'stale')
        self.assertEqual(timestamp_window_error(1021, 1000, 500, 20), 'future')
        self.assertIsNone(timestamp_window_error(800, 1000, 500, 20))
        self.assertEqual(guard.last_published, {})

    def test_timestamp_guard_rejects_duplicates_and_backwards_without_clamping(self):
        guard = TimestampGuard()
        self.assertTrue(guard.accept('scan', 100))
        self.assertFalse(guard.accept('scan', 100))
        self.assertFalse(guard.accept('scan', 90))
        self.assertEqual(guard.last_published['scan'], 100)
        self.assertTrue(guard.accept('scan', 101))
        self.assertTrue(guard.accept('imu', 10))

    def test_ros_anchor_is_fixed_and_detects_both_jump_directions(self):
        anchor = RosTimeAnchor(1_000_000_000, 10_000_000_000)
        self.assertEqual(anchor.to_ros_ns(1_010_000_000), 10_010_000_000)
        self.assertFalse(anchor.jumped(2_000_000_000, 11_000_000_000))
        self.assertTrue(anchor.jumped(2_000_000_000, 11_100_000_000))
        self.assertTrue(anchor.jumped(2_000_000_000, 10_900_000_000))

    def test_ros_anchor_tracks_slow_slew_without_accumulating_old_offset(self):
        anchor = RosTimeAnchor(0, 10_000_000_000)
        for index in range(1, 201):
            monotonic = index * 1_000_000_000
            ros = 10_000_000_000 + monotonic + index * 500_000
            self.assertFalse(anchor.observe(monotonic, ros))
            self.assertEqual(anchor.to_ros_ns(monotonic), ros)
        self.assertEqual(anchor.last_residual_ns, 500_000)
        self.assertTrue(anchor.observe(monotonic + 1_000_000_000, ros + 1_100_000_000))


if __name__ == '__main__':
    unittest.main()
