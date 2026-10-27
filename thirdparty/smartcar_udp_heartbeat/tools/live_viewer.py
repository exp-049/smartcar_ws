"""Local, read-only preview of received lidar circles and IMU measurements.

This is not a ROS publisher. Ages describe host-monotonic time since receipt;
they are never presented as physical acquisition or synchronized ROS timestamps.
Only the existing validated-data ScanAssembler is reused, without importing ROS.
"""
from __future__ import annotations

from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
from pathlib import Path
import sys
import threading
import time

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_CORE_PACKAGE_ROOT = _PROJECT_ROOT / "ros2" / "smartcar_sensor_hal"
if str(_CORE_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_CORE_PACKAGE_ROOT))
from smartcar_sensor_hal.core import ScanAssembler

_HTML_PATH = Path(__file__).with_name("lidar-viewer.html")


def _milliseconds(value):
    if value is None or type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        return None
    return value / 1_000_000.0


class _ReadyServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def service_actions(self):
        # Called inside the running serve_forever loop, making shutdown safe.
        self.ready.set()


class LiveViewer:
    """Receive validated packets on the UDP thread; serve locked HTTP snapshots."""

    def __init__(self, port=8765):
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("viewer port must be 0..65535")
        self.port = port
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._assembler = ScanAssembler()
        self._counters = Counter()
        self._device_ip = None
        self._scan = None
        self._scan_received_ns = None
        self._scan_sequence = 0
        self._last_packet_ns = None
        self._imu = None
        self._imu_received_ns = None
        self._sync = None
        self._server = None
        self._thread = None
        self._ready = threading.Event()
        self._url = None

    def set_peer(self, ip):
        address = None if ip is None else str(ipaddress.IPv4Address(ip))
        with self._lock:
            if self._device_ip is not None and address != self._device_ip:
                self._assembler.reset()
                self._scan = self._imu = None
                self._scan_received_ns = self._imu_received_ns = self._last_packet_ns = None
                self._counters["peer_changes"] += 1
            self._device_ip = address

    def set_sync(self, status):
        with self._lock:
            if status is None:
                self._sync = None
            else:
                getter = status.get if isinstance(status, dict) else lambda key: getattr(status, key, None)
                self._sync = {"locked": bool(getter("locked")),
                              "rtt_ms": _milliseconds(getter("rtt_ns")),
                              "uncertainty_ms": _milliseconds(getter("uncertainty_ns"))}

    def on_lidar(self, parsed):
        now = time.monotonic_ns()
        with self._lock:
            self._last_packet_ns = now
            self._counters["lidar_packets"] += 1
            if self._assembler.boot_id not in (None, parsed.boot_id):
                self._scan = None
                self._scan_received_ns = None
            frame = self._assembler.push(parsed, now)
            if frame is None:
                return
            points = []
            for index, distance in enumerate(frame.ranges_m):
                if not math.isfinite(distance) or distance <= 0:
                    continue
                angle = frame.angle_min + index * frame.angle_increment
                points.append([round(distance * math.cos(angle), 5),
                               round(distance * math.sin(angle), 5)])
            self._scan_sequence += 1
            self._scan = {"seq": self._scan_sequence, "points": points,
                          "valid_points": len(points), "raw_points": frame.raw_point_count,
                          "scan_hz": 1.0 / frame.scan_time_s, "boot_id": frame.boot_id}
            self._scan_received_ns = now
            self._counters["completed_scans"] += 1

    def on_bad_lidar(self):
        with self._lock:
            self._assembler.invalidate()
            self._counters["bad_lidar_packets"] += 1

    def on_imu(self, parsed):
        acceleration = list(parsed.acceleration_m_s2)
        gyro = list(parsed.angular_velocity_rad_s)
        temperature = parsed.temperature_c
        if (len(acceleration) != 3 or len(gyro) != 3 or
                not all(type(value) in (int, float) and math.isfinite(value)
                        for value in (*acceleration, *gyro, temperature))):
            raise ValueError("viewer IMU input must contain finite three-axis SI values")
        with self._lock:
            self._imu = {"accel": acceleration, "gyro": gyro, "temperature_c": temperature}
            self._imu_received_ns = time.monotonic_ns()
            self._counters["imu_packets"] += 1

    def snapshot(self):
        """Return detached JSON-safe state; ages are host receipt ages in seconds."""
        now = time.monotonic_ns()
        with self._lock:
            scan = None
            if self._scan is not None:
                scan = dict(self._scan)
                scan["points"] = [list(point) for point in self._scan["points"]]
                scan["age_s"] = max(0.0, (now - self._scan_received_ns) / 1e9)
            imu = None
            if self._imu is not None:
                imu = dict(self._imu)
                imu["accel"] = list(self._imu["accel"])
                imu["gyro"] = list(self._imu["gyro"])
                imu["age_s"] = max(0.0, (now - self._imu_received_ns) / 1e9)
            counters = dict(self._counters)
            counters.update({f"assembler.{key}": value for key, value in self._assembler.counters.items()})
            return {"device_ip": self._device_ip, "scan": scan, "imu": imu,
                    "sync": None if self._sync is None else dict(self._sync),
                    "lidar_packet_age_s": None if self._last_packet_ns is None else
                    max(0.0, (now - self._last_packet_ns) / 1e9), "counters": counters}

    def start(self):
        """Start an HTTP server bound only to 127.0.0.1 and return its URL."""
        with self._lifecycle_lock:
            if self._server is not None:
                return self._url
            viewer = self

            class Handler(BaseHTTPRequestHandler):
                def setup(self):
                    super().setup()
                    self.connection.settimeout(2)

                def do_GET(self):
                    if self.path == "/state":
                        body = json.dumps(viewer.snapshot(), separators=(",", ":"),
                                          allow_nan=False).encode("utf-8")
                        content_type = "application/json; charset=utf-8"
                    elif self.path == "/":
                        try:
                            body = _HTML_PATH.read_bytes()
                        except OSError:
                            self.send_error(404, "Viewer HTML is not available")
                            return
                        content_type = "text/html; charset=utf-8"
                    else:
                        self.send_error(404, "Unknown viewer path")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    try:
                        self.wfile.write(body)
                    except (BrokenPipeError, ConnectionResetError):
                        pass

                def log_message(self, format, *args):
                    pass

            server = _ReadyServer(("127.0.0.1", self.port), Handler)
            self._ready.clear()
            server.ready = self._ready
            thread = threading.Thread(target=server.serve_forever,
                                      kwargs={"poll_interval": 0.05}, daemon=True,
                                      name="smartcar-live-viewer")
            try:
                thread.start()
            except Exception:
                server.server_close()
                raise
            self._server, self._thread = server, thread
            self._url = f"http://127.0.0.1:{server.server_address[1]}/"
            if not self._ready.wait(timeout=2):
                server.server_close()
                thread.join(timeout=1)
                self._server = self._thread = self._url = None
                raise RuntimeError("local viewer HTTP loop did not start")
            return self._url

    def close(self):
        """Stop the serving loop and release its listening socket; safe repeatedly."""
        with self._lifecycle_lock:
            server, thread = self._server, self._thread
            if server is None:
                return
            if thread.is_alive() and self._ready.is_set():
                server.shutdown()
            server.server_close()
            thread.join(timeout=2)
            self._server = self._thread = self._url = None
