"""Nonblocking MCU-to-host monotonic clock synchronization, without ROS.

SCQ1 requests: <4sQQ, nonce and host monotonic nanoseconds. SCR1 replies:
<4sQQQQQ, nonce, echoed t1, device receive/send microseconds, and boot uint64.
The affine model maps device microseconds to host monotonic nanoseconds.

Uncertainty is a statistical diagnostic, NOT an upper bound on asymmetric
network delay. Convert monotonic timestamps to ROS using a separately anchored
host-monotonic/ROS offset. Reject live use_sim_time and re-anchor on ROS clock
jumps in the ROS adapter; never replace a sample timestamp with receipt time.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import ipaddress
import math
import re
import secrets
import socket
import statistics
import struct
import time
from typing import Optional

REQUEST = struct.Struct("<4sQQ")
RESPONSE = struct.Struct("<4sQQQQQ")


class ClockNotReadyError(ValueError):
    """Clock is unlocked, expired, or belongs to a different MCU boot."""


@dataclass(frozen=True)
class ClockStatus:
    locked: bool
    reason: str
    boot_id: Optional[str]
    samples: int
    rtt_ns: Optional[int]
    offset_ns: Optional[int]
    scale_ns_per_us: Optional[float]
    drift_ppm: Optional[float]
    residual_ns: Optional[float]
    uncertainty_ns: Optional[int]
    accepted: int
    rejected: int
    restarts: int
    last_rejection: str


@dataclass(frozen=True)
class _Sample:
    dev_us: float
    host_ns: int
    rtt_ns: int
    t4_ns: int


def _integer(value: object, maximum: int) -> bool:
    return type(value) is int and 0 <= value <= maximum


class ClockMapper:
    """Fit low-RTT exchanges; require five inliers and refuse stale mappings.

    add_exchange returns True when an exchange enters the bounded observation
    window, not necessarily when the fit is already locked. Transport rejection
    (source, nonce, echo) is handled by SyncClient before this method is called.
    """

    def __init__(self, min_samples: int = 5, max_samples: int = 40,
                 max_age_s: float = 5.0, max_rtt_ms: float = 100.0,
                 max_drift_ppm: float = 2000.0, max_jump_ms: float = 20.0,
                 min_span_s: float = 1.0, rtt_filter_window_ms: float = 2.0,
                 max_residual_ms: float = 5.0):
        if (type(min_samples) is not int or type(max_samples) is not int or
                min_samples < 5 or max_samples < min_samples):
            raise ValueError("require 5 <= min_samples <= max_samples")
        for value in (max_age_s, max_rtt_ms, max_drift_ppm, max_jump_ms,
                      min_span_s, rtt_filter_window_ms, max_residual_ms):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("clock limits must be positive finite numbers")
        self.min_samples = min_samples
        self.max_age_ns = int(max_age_s * 1e9)
        self.max_rtt_ns = int(max_rtt_ms * 1e6)
        self.rtt_filter_window_ns = int(rtt_filter_window_ms * 1e6)
        self.max_residual_ns = int(max_residual_ms * 1e6)
        self.max_drift_ppm = float(max_drift_ppm)
        self.max_jump_ns = int(max_jump_ms * 1e6)
        self.min_span_us = min_span_s * 1e6
        self._samples = deque(maxlen=max_samples)
        self._retired_boots = deque(maxlen=16)
        self.boot_id = None
        self.accepted = self.rejected = self.restarts = 0
        self.last_rejection = ""
        self._clear_model("not_enough_samples")

    def _clear_model(self, reason: str) -> None:
        self._reason = reason
        self._scale = None
        self._ref_dev = self._ref_host = None
        self._model_ns = None
        self._rtt = self._residual = self._uncertainty = None
        self._inliers = 0

    def _reject(self, reason: str) -> bool:
        self.rejected += 1
        self.last_rejection = reason
        return False

    def add_exchange(self, boot_id: str, t1_host_ns: int, t2_dev_us: int,
                     t3_dev_us: int, t4_host_ns: int) -> bool:
        if (not isinstance(boot_id, str) or not re.fullmatch(r"[0-9a-f]{16}", boot_id) or
                not _integer(t1_host_ns, 0xFFFFFFFFFFFFFFFF) or
                not _integer(t4_host_ns, 0xFFFFFFFFFFFFFFFF) or
                not _integer(t2_dev_us, 0x7FFFFFFFFFFFFFFF) or
                not _integer(t3_dev_us, 0x7FFFFFFFFFFFFFFF)):
            return self._reject("invalid_exchange")
        if t4_host_ns < t1_host_ns or t3_dev_us < t2_dev_us:
            return self._reject("backwards_exchange")
        rtt = (t4_host_ns - t1_host_ns) - 1000 * (t3_dev_us - t2_dev_us)
        if rtt < 0:
            return self._reject("negative_rtt")
        if rtt > self.max_rtt_ns:
            return self._reject("rtt_too_large")
        if boot_id != self.boot_id:
            if boot_id in self._retired_boots:
                return self._reject("retired_boot")
            if self.boot_id is not None:
                self._retired_boots.append(self.boot_id)
                self.restarts += 1
            self.boot_id = boot_id
            self._samples.clear()
            self._clear_model("boot_changed")
        sample = _Sample((t2_dev_us + t3_dev_us) / 2.0,
                         (t1_host_ns + t4_host_ns) // 2, rtt, t4_host_ns)
        if self._samples:
            previous = self._samples[-1]
            if sample.t4_ns <= previous.t4_ns or sample.dev_us <= previous.dev_us:
                return self._reject("old_exchange")
        if self._scale is not None and self._model_ns is not None:
            if sample.t4_ns - self._model_ns <= self.max_age_ns:
                predicted = self._map_unchecked(sample.dev_us)
                if abs(sample.host_ns - predicted) > self.max_jump_ns:
                    return self._reject("clock_jump_or_asymmetry")
        self._samples.append(sample)
        self.accepted += 1
        self._fit()
        return True

    def _fit(self) -> None:
        best_rtt = min(sample.rtt_ns for sample in self._samples)
        cutoff = best_rtt + max(self.rtt_filter_window_ns, best_rtt // 4)
        selected = [sample for sample in self._samples if sample.rtt_ns <= cutoff]
        self._clear_model("not_enough_low_rtt_samples")
        self._inliers = len(selected)
        self._rtt = best_rtt
        if len(selected) < self.min_samples:
            return
        if selected[-1].dev_us - selected[0].dev_us < self.min_span_us:
            self._reason = "insufficient_time_span"
            return
        x0, y0 = selected[0].dev_us, selected[0].host_ns
        coordinates = [(sample.dev_us - x0, sample.host_ns - y0) for sample in selected]
        slopes = [(yb - ya) / (xb - xa)
                  for index, (xa, ya) in enumerate(coordinates)
                  for xb, yb in coordinates[index + 1:] if xb - xa >= 100_000]
        if not slopes:
            self._reason = "insufficient_time_span"
            return
        rough_scale = statistics.median(slopes)
        intercepts = [y - rough_scale * x for x, y in coordinates]
        rough_offset = statistics.median(intercepts)
        deviations = [abs(value - rough_offset) for value in intercepts]
        tolerance = max(2_000_000.0, 4 * statistics.median(deviations), best_rtt / 2)
        selected = [sample for sample, deviation in zip(selected, deviations) if deviation <= tolerance]
        self._inliers = len(selected)
        if len(selected) < self.min_samples:
            self._reason = "not_enough_inliers"
            return
        coordinates = [(sample.dev_us - x0, sample.host_ns - y0) for sample in selected]
        mean_x = statistics.fmean(x for x, _ in coordinates)
        mean_y = statistics.fmean(y for _, y in coordinates)
        sxx = sum((x - mean_x) ** 2 for x, _ in coordinates)
        if sxx <= 0:
            self._reason = "insufficient_time_span"
            return
        scale = sum((x - mean_x) * (y - mean_y) for x, y in coordinates) / sxx
        drift = (scale / 1000.0 - 1) * 1e6
        if not math.isfinite(scale) or abs(drift) > self.max_drift_ppm:
            self._reason = "extreme_drift"
            return
        residual = math.sqrt(statistics.fmean(
            ((y - mean_y) - scale * (x - mean_x)) ** 2 for x, y in coordinates))
        if residual > self.max_residual_ns:
            self._reason = "excessive_residual"
            return
        self._scale = scale
        self._ref_dev = x0 + mean_x
        self._ref_host = y0 + mean_y
        self._model_ns = max(sample.t4_ns for sample in selected)
        self._residual = residual
        self._uncertainty = math.ceil(best_rtt / 2 + residual)
        self._reason = "locked"

    def _map_unchecked(self, dev_us: float) -> int:
        return round(self._ref_host + self._scale * (dev_us - self._ref_dev))

    def status(self, now_ns: Optional[int] = None) -> ClockStatus:
        now = time.monotonic_ns() if now_ns is None else now_ns
        if not _integer(now, 0xFFFFFFFFFFFFFFFF):
            raise ValueError("invalid host monotonic time")
        reason = self._reason
        if self._scale is not None:
            if now < self._model_ns:
                reason = "host_time_backwards"
            elif now - self._model_ns > self.max_age_ns:
                reason = "expired"
        locked = self._scale is not None and reason == "locked"
        offset = None
        if self._scale is not None:
            offset = round(self._ref_host - 1000.0 * self._ref_dev)
        return ClockStatus(locked, reason, self.boot_id, self._inliers,
                           self._rtt, offset, self._scale,
                           None if self._scale is None else (self._scale / 1000 - 1) * 1e6,
                           self._residual, self._uncertainty, self.accepted,
                           self.rejected, self.restarts, self.last_rejection)

    def to_host_monotonic_ns(self, dev_us: int, boot_id: str,
                             now_ns: Optional[int] = None) -> int:
        if not _integer(dev_us, 0x7FFFFFFFFFFFFFFF):
            raise ClockNotReadyError("invalid device timestamp")
        status = self.status(now_ns)
        if boot_id != self.boot_id:
            raise ClockNotReadyError("boot_id mismatch")
        if not status.locked:
            raise ClockNotReadyError(status.reason)
        return self._map_unchecked(dev_us)


class SyncClient:
    """Own a nonblocking IPv4 UDP socket; call poll from a periodic timer.

    The host argument is a numeric IPv4 address, so poll never performs DNS.
    now_ns injects monotonic time for deterministic tests. Live callers should
    omit it so t4 is measured immediately after each successful recvfrom.
    """

    def __init__(self, host: str, port: int = 8889, mapper: Optional[ClockMapper] = None,
                 interval_s: float = 0.5, request_timeout_s: float = 0.5,
                 sock: Optional[socket.socket] = None):
        self.peer = (str(ipaddress.IPv4Address(host)), port)
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("invalid sync port")
        for value in (interval_s, request_timeout_s):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("sync intervals must be positive and finite")
        self.mapper = mapper if mapper is not None else ClockMapper()
        self.interval_ns = int(interval_s * 1e9)
        self.timeout_ns = int(request_timeout_s * 1e9)
        if self.interval_ns < 1 or self.timeout_ns < 1:
            raise ValueError("sync intervals must be at least one nanosecond")
        self.socket = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.pending = {}
        self._last_send_ns = None
        self._closed = False
        self.counters = {name: 0 for name in ("sent", "accepted", "timeout", "send_errors",
                                              "unexpected_source", "invalid_response",
                                              "unknown_nonce", "bad_echo", "mapper_rejected", "recv_errors")}

    def poll(self, now_ns: Optional[int] = None) -> ClockStatus:
        if self._closed:
            raise RuntimeError("sync client is closed")
        now = time.monotonic_ns() if now_ns is None else now_ns
        if not _integer(now, 0xFFFFFFFFFFFFFFFF):
            raise ValueError("invalid host monotonic time")
        for nonce, t1 in list(self.pending.items()):
            if now - t1 > self.timeout_ns:
                del self.pending[nonce]
                self.counters["timeout"] += 1
        if (self._last_send_ns is None or now - self._last_send_ns >= self.interval_ns) and len(self.pending) < 16:
            nonce = secrets.randbits(64)
            while nonce in self.pending:
                nonce = secrets.randbits(64)
            request = REQUEST.pack(b"SCQ1", nonce, now)
            try:
                sent = self.socket.sendto(request, self.peer)
                if sent != len(request):
                    raise OSError("incomplete sync request")
                self.pending[nonce] = now
                self.counters["sent"] += 1
            except OSError:
                self.counters["send_errors"] += 1
            self._last_send_ns = now
        for _ in range(32):
            try:
                data, source = self.socket.recvfrom(RESPONSE.size + 1)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                self.counters["recv_errors"] += 1
                break
            t4 = time.monotonic_ns() if now_ns is None else now
            if source != self.peer:
                self.counters["unexpected_source"] += 1
                continue
            if len(data) != RESPONSE.size:
                self.counters["invalid_response"] += 1
                continue
            magic, nonce, echo, t2, t3, boot = RESPONSE.unpack(data)
            if magic != b"SCR1":
                self.counters["invalid_response"] += 1
                continue
            t1 = self.pending.get(nonce)
            if t1 is None:
                self.counters["unknown_nonce"] += 1
                continue
            if echo != t1:
                self.counters["bad_echo"] += 1
                continue
            del self.pending[nonce]
            if t4 - t1 > self.timeout_ns:
                self.counters["timeout"] += 1
            elif self.mapper.add_exchange(f"{boot:016x}", t1, t2, t3, t4):
                self.counters["accepted"] += 1
            else:
                self.counters["mapper_rejected"] += 1
        return self.mapper.status(now if now_ns is not None else None)

    def close(self) -> None:
        if not self._closed:
            self.socket.close()
            self.pending.clear()
            self._closed = True
