"""Real scan-core assembly plus localhost HTTP tests; no ROS or sensor needed."""
import http.client
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import live_viewer
from lidar_protocol import compute_lidar_crc32, decode_lidar_packet
from live_viewer import LiveViewer

BOOT = "0123456789abcdef"


def packet(sequence, frame, sector, *, distance_raw=4000, boot=BOOT):
    count = 5
    ct = 1 if sector == 0 else 0
    first, last = (sector * 45 * 64 << 1) | 1, ((sector * 45 + 36) * 64 << 1) | 1
    samples = [distance_raw] * count
    checksum = 0x55AA ^ (ct | count << 8) ^ first ^ last
    for distance in samples:
        checksum ^= distance
    raw = struct.pack("<HBBHHH5H", 0x55AA, ct, count, first, last, checksum, *samples)
    first_us = 1_000_000 + frame * 100_000 + sector * 12_500
    end_us = round(first_us + 4 * 250 + len(raw) * 10 / 115200 * 1e6)
    message = {"type": "lidar_packet", "version": 1, "boot_id": boot,
               "seq": sequence, "rx_complete_us": end_us, "data_hex": raw.hex(),
               "crc32": compute_lidar_crc32(boot, sequence, end_us, raw)}
    return decode_lidar_packet(message)


def finish_circle(viewer, distance_raw=4000, frame=0):
    for sector in range(8):
        viewer.on_lidar(packet(frame * 8 + sector, frame, sector, distance_raw=distance_raw))
    viewer.on_lidar(packet(frame * 8 + 8, frame + 1, 0, distance_raw=distance_raw))


class ViewerStateTests(unittest.TestCase):
    def test_empty_snapshot_contains_no_invented_data(self):
        viewer = LiveViewer(port=0)
        state = viewer.snapshot()
        self.assertIsNone(state["device_ip"])
        self.assertIsNone(state["scan"])
        self.assertIsNone(state["imu"])
        self.assertIsNone(state["sync"])
        self.assertIsNone(state["lidar_packet_age_s"])
        viewer.close()

    def test_real_validated_packets_form_circle_and_front_left_right_xy(self):
        viewer = LiveViewer(port=0)
        for sector in range(8):
            viewer.on_lidar(packet(sector, 0, sector))
        self.assertIsNone(viewer.snapshot()["scan"])
        viewer.on_lidar(packet(8, 1, 0))
        scan = viewer.snapshot()["scan"]
        self.assertEqual(scan["raw_points"], 40)
        self.assertEqual(scan["valid_points"], 40)
        self.assertEqual(scan["seq"], 1)
        self.assertAlmostEqual(scan["scan_hz"], 10.0, places=5)
        self.assertGreater(scan["points"][0][0], 0.9)  # forward x
        self.assertLess(scan["points"][10][1], -0.9)  # clockwise 90: right is negative y
        self.assertGreater(scan["points"][30][1], 0.9)  # clockwise 270: left is positive y
        for x, y in scan["points"]:
            self.assertAlmostEqual(math.hypot(x, y), 1.0, places=4)

    def test_bad_packet_invalidates_circle_and_next_full_circle_recovers(self):
        viewer = LiveViewer(port=0)
        for sector in range(8):
            viewer.on_lidar(packet(sector, 0, sector))
            if sector == 3:
                viewer.on_bad_lidar()
        viewer.on_lidar(packet(8, 1, 0))
        self.assertIsNone(viewer.snapshot()["scan"])
        for sector in range(1, 8):
            viewer.on_lidar(packet(8 + sector, 1, sector))
        viewer.on_lidar(packet(16, 2, 0))
        state = viewer.snapshot()
        self.assertIsNotNone(state["scan"])
        self.assertEqual(state["counters"]["bad_lidar_packets"], 1)
        self.assertEqual(state["counters"]["assembler.invalid_dropped_scans"], 1)

    def test_zero_range_circle_is_empty_and_json_has_no_nan(self):
        viewer = LiveViewer(port=0)
        finish_circle(viewer, distance_raw=0)
        state = viewer.snapshot()
        self.assertEqual(state["scan"]["points"], [])
        self.assertEqual(state["scan"]["valid_points"], 0)
        self.assertEqual(state["scan"]["raw_points"], 40)
        json.dumps(state, allow_nan=False)

    def test_monotonic_ages_imu_sync_and_detached_snapshot(self):
        viewer = LiveViewer(port=0)
        imu = SimpleNamespace(acceleration_m_s2=(0.0, -1.0, 9.80665),
                              angular_velocity_rad_s=(0.1, 0.2, 0.3), temperature_c=32.0)
        with patch.object(live_viewer.time, "monotonic_ns", return_value=10_000_000_000):
            viewer.set_peer("192.0.2.5")
            finish_circle(viewer)
            viewer.on_imu(imu)
        viewer.set_sync(SimpleNamespace(locked=True, rtt_ns=2_000_000, uncertainty_ns=1_500_000))
        with patch.object(live_viewer.time, "monotonic_ns", return_value=12_500_000_000):
            state = viewer.snapshot()
            self.assertEqual(state["scan"]["age_s"], 2.5)
            self.assertEqual(state["imu"]["age_s"], 2.5)
            self.assertEqual(state["lidar_packet_age_s"], 2.5)
            self.assertEqual(state["sync"], {"locked": True, "rtt_ms": 2.0, "uncertainty_ms": 1.5})
            state["scan"]["points"][0][0] = 999
            state["imu"]["accel"][0] = 999
            state["counters"]["completed_scans"] = 999
            fresh = viewer.snapshot()
            self.assertNotEqual(fresh["scan"]["points"][0][0], 999)
            self.assertNotEqual(fresh["imu"]["accel"][0], 999)
            self.assertEqual(fresh["counters"]["completed_scans"], 1)

    def test_peer_change_clears_previous_device_data(self):
        viewer = LiveViewer(port=0)
        viewer.set_peer("192.0.2.1")
        finish_circle(viewer)
        viewer.set_peer("192.0.2.2")
        self.assertIsNone(viewer.snapshot()["scan"])
        self.assertIsNone(viewer.snapshot()["lidar_packet_age_s"])


class ViewerHTTPTests(unittest.TestCase):
    def setUp(self):
        self.viewer = LiveViewer(port=0)
        self.url = self.viewer.start()
        address = urlsplit(self.url)
        self.connection = http.client.HTTPConnection(address.hostname, address.port, timeout=2)

    def tearDown(self):
        self.connection.close()
        self.viewer.close()

    def test_localhost_state_and_only_fixed_paths(self):
        self.assertTrue(self.url.startswith("http://127.0.0.1:"))
        self.connection.request("GET", "/state")
        response = self.connection.getresponse()
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Cache-Control"), "no-store")
        self.assertIsNone(json.loads(response.read())["scan"])
        for path in ("/../sensor_protocol.py", "/%2e%2e/sensor_protocol.py", "/state/", "/state?file=x"):
            self.connection.request("GET", path)
            response = self.connection.getresponse()
            self.assertEqual(response.status, 404, path)
            response.read()

    def test_root_serves_one_fixed_html_file(self):
        with tempfile.TemporaryDirectory() as directory:
            html = Path(directory) / "viewer.html"
            html.write_text("<!doctype html><title>test viewer</title>", encoding="utf-8")
            with patch.object(live_viewer, "_HTML_PATH", html):
                self.connection.request("GET", "/")
                response = self.connection.getresponse()
                self.assertEqual(response.status, 200)
                self.assertIn(b"test viewer", response.read())

    def test_close_and_start_are_idempotent_without_shutdown_deadlock(self):
        self.assertEqual(self.viewer.start(), self.url)
        thread = self.viewer._thread
        self.viewer.close()
        self.assertFalse(thread.is_alive())
        self.viewer.close()


if __name__ == "__main__":
    unittest.main()
