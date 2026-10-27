"""Pure-Python scan assembly and timestamp guards; ROS is not required.

The input is the validated ParsedLidarPacket interface from lidar_protocol.
Arrival order is acquisition order. Rebinning preserves clockwise angular order
and produces a negative ROS angle_increment with positive time_increment.
Point timing remains an approximation: UART software receive timestamps do not
measure device buffering, and distance-dependent angle correction is nonuniform.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ScanFrame:
    boot_id: str
    first_sample_us: int
    scan_time_s: float
    angle_min: float
    angle_max: float
    angle_increment: float
    time_increment: float
    ranges_m: tuple[float, ...]
    raw_point_count: int
    valid_point_count: int
    coverage_deg: float
    max_time_interpolation_error_us: float
    packet_count: int
    timestamp_mode: str = 'uart_receive_backprojection_and_uniform_angle_time_estimate'


class ScanAssembler:
    """Close scans only when the next valid CT&1 boundary is received."""

    def __init__(self, *, sample_rate_hz=4000.0, baudrate=115200,
                 device_delay_us=0.0, scan_bins=0, correction='triangle',
                 yaw_offset_deg=0.0, timeout_s=0.5, min_coverage_deg=300.0,
                 min_points=8, max_points=10000):
        if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0 or baudrate <= 0:
            raise ValueError('sample rate and baudrate must be positive')
        if scan_bins < 0 or (scan_bins and not 8 <= scan_bins <= 10000):
            raise ValueError('scan_bins must be zero or 8..10000')
        if correction not in ('none', 'triangle'):
            raise ValueError('correction must be none or triangle')
        if timeout_s <= 0 or not math.isfinite(timeout_s):
            raise ValueError('timeout must be positive')
        for value in (device_delay_us, yaw_offset_deg, min_coverage_deg):
            if not math.isfinite(value):
                raise ValueError('scan parameters must be finite')
        if device_delay_us < 0:
            raise ValueError('device delay must not be negative')
        self.sample_period_us = 1_000_000.0 / sample_rate_hz
        self.baudrate = baudrate
        self.device_delay_us = device_delay_us
        self.scan_bins = scan_bins
        self.correction = correction
        self.yaw_offset_rad = math.radians(yaw_offset_deg)
        self.timeout_ns = int(timeout_s * 1e9)
        self.min_coverage_deg = min_coverage_deg
        self.min_points = min_points
        self.max_points = max_points
        self.counters = Counter()
        self.reset()

    def reset(self):
        self.boot_id = None
        self.last_seq = None
        self.last_arrival_ns = None
        self._samples = None
        self._invalid = False
        self._last_raw = None
        self._last_unwrapped = None
        self._packet_count = 0

    def invalidate(self):
        """An invalid transport/UART packet invalidates the current circle."""
        if self._samples is not None:
            self._invalid = True
        self.counters['invalidated'] += 1

    def expire(self, now_ns):
        if (self._samples is not None and self.last_arrival_ns is not None and
                now_ns - self.last_arrival_ns > self.timeout_ns):
            self._samples = None
            self._invalid = False
            self.counters['timeout_dropped_scans'] += 1

    def _first_time(self, packet):
        wire_us = len(packet.raw) * 10.0 / self.baudrate * 1_000_000.0
        last_us = packet.rx_complete_us - wire_us - self.device_delay_us
        return last_us - (packet.point_count - 1) * self.sample_period_us

    def push(self, packet, arrival_monotonic_ns):
        self.expire(arrival_monotonic_ns)
        if packet.boot_id != self.boot_id:
            if self.boot_id is not None:
                self.counters['boot_resets'] += 1
            self.reset()
            self.boot_id = packet.boot_id
        self.last_arrival_ns = arrival_monotonic_ns
        if self.last_seq is not None:
            delta = (packet.seq - self.last_seq) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                self.counters['duplicate_or_out_of_order_packets'] += 1
                self.invalidate()
                return None
            if delta != 1:
                self.counters['sequence_gap_packets'] += delta - 1
                self.invalidate()
        self.last_seq = packet.seq

        first_time = self._first_time(packet)
        completed = None
        if packet.start_of_scan:
            if self._samples is not None:
                completed = self._finish(first_time)
            self._samples = []
            self._invalid = False
            self._last_raw = None
            self._last_unwrapped = None
            self._packet_count = 0
        if self._samples is None:
            self.counters['initial_partial_packets'] += 1
            return None
        if self._invalid:
            return completed
        if packet.point_count == 0 or first_time < 0:
            self.invalidate()
            return completed

        self._packet_count += 1
        for index, (angle, distance) in enumerate(zip(packet.angles_cw_deg, packet.ranges_m)):
            raw = float(angle) % 360.0
            if self._last_raw is None:
                unwrapped = raw
            else:
                change = (raw - self._last_raw) % 360.0
                if change > 180.0:
                    self.counters['angle_discontinuities'] += 1
                    self.invalidate()
                    break
                unwrapped = self._last_unwrapped + change
            self._samples.append((unwrapped, distance, first_time + index * self.sample_period_us))
            self._last_raw = raw
            self._last_unwrapped = unwrapped
            if len(self._samples) > self.max_points:
                self.counters['oversize_scans'] += 1
                self.invalidate()
                break
        return completed

    def _angle_correction_deg(self, distance_m):
        if self.correction == 'none' or distance_m is None or distance_m <= 0:
            return 0.0
        distance_mm = distance_m * 1000.0
        return math.degrees(math.atan(21.8 * (155.3 - distance_mm) / (155.3 * distance_mm)))

    def _finish(self, next_first_us):
        samples = self._samples
        if self._invalid or samples is None or len(samples) < self.min_points:
            self.counters['invalid_dropped_scans'] += 1
            return None
        coverage = samples[-1][0] - samples[0][0]
        first_us = samples[0][2]
        scan_time = (next_first_us - first_us) / 1_000_000.0
        if not self.min_coverage_deg <= coverage <= 380.0 or not 0.03 <= scan_time <= 0.5:
            self.counters['geometry_or_time_dropped_scans'] += 1
            return None
        count = self.scan_bins or len(samples)
        step_deg = 360.0 / count
        start = samples[0][0] + self._angle_correction_deg(samples[0][1])
        ranges = [math.inf] * count
        angular_errors = [math.inf] * count
        max_time_error = 0.0
        for raw_angle, distance, estimate_us in samples:
            if distance is None or not math.isfinite(distance) or distance <= 0:
                continue
            corrected = raw_angle + self._angle_correction_deg(distance)
            relative = corrected - start
            # Distance-dependent correction can move a ray just before the
            # current start. It must not wrap to the end and gain a full-circle
            # timestamp; similarly, rounding past the last bin cannot wrap to 0.
            if not 0.0 <= relative < 360.0:
                self.counters['corrected_outside_scan_points'] += 1
                continue
            index = int(math.floor(relative / step_deg + 0.5))
            if index >= count:
                self.counters['rounded_outside_scan_points'] += 1
                continue
            angular_error = abs(relative - index * step_deg)
            if angular_error < angular_errors[index]:
                ranges[index] = distance
                angular_errors[index] = angular_error
                grid_time_us = first_us + index * scan_time * 1_000_000.0 / count
                max_time_error = max(max_time_error, abs(estimate_us - grid_time_us))
        angle_min = self.yaw_offset_rad - math.radians(start)
        angle_min = (angle_min + math.pi) % (2.0 * math.pi) - math.pi
        increment = -2.0 * math.pi / count
        frame = ScanFrame(
            self.boot_id, int(round(first_us)), scan_time, angle_min,
            angle_min + (count - 1) * increment, increment, scan_time / count,
            tuple(ranges), len(samples), sum(math.isfinite(value) for value in ranges),
            coverage, max_time_error, self._packet_count,
        )
        self.counters['completed_scans'] += 1
        return frame


class TimestampGuard:
    """Never clamp a timestamp to fabricate monotonicity."""
    def __init__(self):
        self.last_published = {}

    def accept(self, stream, stamp_ns):
        if stamp_ns <= self.last_published.get(stream, -1):
            return False
        self.last_published[stream] = stamp_ns
        return True


def timestamp_window_error(sample_host_ns, now_ns, max_age_ns, max_future_ns):
    """Check before updating the monotonic guard; no synthetic clamping."""
    if sample_host_ns < now_ns - max_age_ns:
        return 'stale'
    if sample_host_ns > now_ns + max_future_ns:
        return 'future'
    return None


class RosTimeAnchor:
    """Observed monotonic-to-ROS baseline; track slew and detect abrupt steps."""
    def __init__(self, monotonic_ns, ros_ns, jump_threshold_ns=50_000_000):
        self.monotonic_ns = monotonic_ns
        self.ros_ns = ros_ns
        self.jump_threshold_ns = jump_threshold_ns
        self.last_residual_ns = 0

    def jumped(self, monotonic_now_ns, ros_now_ns):
        expected = self.ros_ns + monotonic_now_ns - self.monotonic_ns
        return abs(ros_now_ns - expected) > self.jump_threshold_ns

    def observe(self, monotonic_midpoint_ns, ros_ns):
        expected = self.to_ros_ns(monotonic_midpoint_ns)
        self.last_residual_ns = ros_ns - expected
        step = abs(self.last_residual_ns) > self.jump_threshold_ns
        self.monotonic_ns = monotonic_midpoint_ns
        self.ros_ns = ros_ns
        return step

    def to_ros_ns(self, host_monotonic_ns):
        return self.ros_ns + host_monotonic_ns - self.monotonic_ns
