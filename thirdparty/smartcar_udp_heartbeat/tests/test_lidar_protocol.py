"""No-network regressions for the staged UART/UDP decoder and receiver.

Run from the project folder: python3 -m unittest discover -s tests -v
The static C encoder vectors were produced independently by the firmware tests.
"""
from __future__ import annotations

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from lidar_protocol import (
    LidarCRCError, LidarProtocolError, compute_lidar_crc32,
    decode_lidar_packet, load_json_message,
)
import udp_receiver


GOLDEN = {
    "type": "lidar_packet", "version": 1, "boot_id": "0123456789abcdef",
    "seq": 7, "rx_complete_us": 123456789,
    "data_hex": "aa55a10301af01054b8fe001a00f007d", "crc32": "94e5b781",
}
C_ENCODER_GOLDEN = {
    "type": "lidar_packet", "version": 1, "boot_id": "0123456789abcdef",
    "seq": 42, "rx_complete_us": 123456789,
    "data_hex": "aa55a1030100012deb6ba00f401f0000", "crc32": "d223eed5",
}


def make_uart(samples, first_deg=0, last_deg=90, ct=0xA1):
    count = len(samples)
    first = (int(first_deg * 64) << 1) | 1
    last = (int(last_deg * 64) << 1) | 1
    checksum = 0x55AA ^ (count << 8 | ct) ^ first ^ last
    for sample in samples:
        checksum ^= sample
    return struct.pack("<HBBHHH", 0x55AA, ct, count, first, last, checksum) + struct.pack(f"<{count}H", *samples)


def envelope(raw, seq=7, boot_id="0123456789abcdef", rx_complete_us=123456789):
    return {
        "type": "lidar_packet", "version": 1, "boot_id": boot_id,
        "seq": seq, "rx_complete_us": rx_complete_us, "data_hex": raw.hex(),
        "crc32": compute_lidar_crc32(boot_id, seq, rx_complete_us, raw),
    }


class ProtocolTests(unittest.TestCase):
    def test_static_golden_packet_and_full_ct_checksum(self):
        parsed = decode_lidar_packet(GOLDEN)
        self.assertEqual(parsed.ct, 0xA1)
        self.assertTrue(parsed.start_of_scan)
        self.assertEqual(parsed.point_count, 3)
        self.assertEqual(parsed.ranges_m, (0.12, 1.0, 8.0))
        self.assertEqual(parsed.angles_cw_deg, (350.0, 0.0, 10.0))
        self.assertEqual(
            compute_lidar_crc32("0123456789abcdef", 7, 123456789, parsed.raw),
            "94e5b781",
        )

    def test_independent_c_encoder_golden(self):
        parsed = decode_lidar_packet(C_ENCODER_GOLDEN)
        self.assertEqual(parsed.seq, 42)
        self.assertEqual(parsed.angles_cw_deg, (0.0, 45.0, 90.0))
        self.assertEqual(parsed.ranges_m, (1.0, 2.0, None))

    def test_maximum_c_encoder_vector(self):
        raw = bytes.fromhex("aa55a1ff0100012dab88") + bytes.fromhex("a00f") * 255
        packet = {
            "type": "lidar_packet", "version": 1, "boot_id": "ffffffffffffffff",
            "seq": 0xFFFFFFFF, "rx_complete_us": 0x7FFFFFFFFFFFFFFF,
            "data_hex": raw.hex(), "crc32": "8b5ca684",
        }
        self.assertEqual(len(raw), 520)
        parsed = decode_lidar_packet(packet)
        self.assertEqual(parsed.point_count, 255)
        self.assertTrue(all(distance == 1.0 for distance in parsed.ranges_m))
        self.assertEqual(parsed.angles_cw_deg[0], 0.0)
        self.assertEqual(parsed.angles_cw_deg[-1], 90.0)

    def test_metadata_corruption_is_crc_error(self):
        for field, value in [
            ("seq", 8), ("rx_complete_us", 123456790), ("boot_id", "0123456789abcdee"),
        ]:
            with self.subTest(field=field):
                packet = dict(GOLDEN)
                packet[field] = value
                with self.assertRaises(LidarCRCError):
                    decode_lidar_packet(packet)

    def test_raw_corruption_is_crc_error(self):
        packet = dict(GOLDEN)
        raw = bytearray.fromhex(packet["data_hex"])
        raw[-1] ^= 1
        packet["data_hex"] = raw.hex()
        with self.assertRaises(LidarCRCError):
            decode_lidar_packet(packet)

    def test_inner_checksum_is_checked_after_valid_transport_crc(self):
        raw = bytearray.fromhex(GOLDEN["data_hex"])
        raw[8] ^= 1
        with self.assertRaisesRegex(LidarProtocolError, "XOR"):
            decode_lidar_packet(envelope(bytes(raw)))

    def test_invalid_ranges_become_none(self):
        parsed = decode_lidar_packet(envelope(make_uart([0, 1, 479, 480, 32000, 32001, 65535])))
        self.assertEqual(parsed.ranges_m, (None, None, None, 0.12, 8.0, None, None))

    def test_single_point_and_no_start_flag(self):
        parsed = decode_lidar_packet(envelope(make_uart([4000], 15, 15, ct=0xA0)))
        self.assertEqual(parsed.angles_cw_deg, (15.0,))
        self.assertFalse(parsed.start_of_scan)

    def test_endpoint_360_matches_firmware_bounds_and_keeps_full_span(self):
        near_wrap = decode_lidar_packet(envelope(make_uart([4000] * 3, 359, 360)))
        self.assertEqual(near_wrap.angles_cw_deg, (359.0, 359.5, 0.0))
        full_span = decode_lidar_packet(envelope(make_uart([4000] * 3, 0, 360)))
        self.assertEqual(full_span.angles_cw_deg, (0.0, 180.0, 0.0))
        single = decode_lidar_packet(envelope(make_uart([4000], 360, 360)))
        self.assertEqual(single.angles_cw_deg, (0.0,))

    def test_truncated_or_malformed_uart_packets(self):
        valid = bytearray.fromhex(GOLDEN["data_hex"])
        variants = []
        variants.append(bytes(valid[:-2]))
        no_count = bytearray(valid)
        no_count[3] = 0
        variants.append(bytes(no_count))
        bad_header = bytearray(valid)
        bad_header[0] = 0
        variants.append(bytes(bad_header))
        no_check_bit = bytearray(valid)
        no_check_bit[4] &= 0xFE
        variants.append(bytes(no_check_bit))
        bad_angle = bytearray(valid)
        struct.pack_into("<H", bad_angle, 4, (361 * 64 << 1) | 1)
        variants.append(bytes(bad_angle))
        for raw in variants:
            with self.subTest(raw=raw.hex()):
                with self.assertRaises(LidarProtocolError):
                    decode_lidar_packet(envelope(raw))

    def test_bad_envelope_fields_are_rejected(self):
        variants = [
            ("version", True), ("version", 2), ("type", "scan"),
            ("boot_id", "ABCDEF0123456789"), ("boot_id", 0),
            ("seq", True), ("seq", -1), ("seq", 0x100000000),
            ("rx_complete_us", False), ("rx_complete_us", -1),
            ("rx_complete_us", 0x8000000000000000),
            ("data_hex", "a"), ("data_hex", "zz" * 10),
            ("data_hex", "aa" * 521), ("data_hex", "aa 55" * 4),
            ("crc32", "ABCDEF12"), ("crc32", "000"), ("crc32", 0),
        ]
        for field, value in variants:
            with self.subTest(field=field, value=value):
                packet = dict(GOLDEN)
                packet[field] = value
                with self.assertRaises(LidarProtocolError):
                    decode_lidar_packet(packet)
        for field in GOLDEN:
            packet = dict(GOLDEN)
            del packet[field]
            with self.subTest(missing=field):
                with self.assertRaises(LidarProtocolError):
                    decode_lidar_packet(packet)

    def test_json_malformed_input(self):
        for data in [b"not-json", b"[]", b"null", b"\xff", b'{"x":NaN}', b'{"x":Infinity}']:
            with self.subTest(data=data):
                with self.assertRaises((ValueError, UnicodeError)):
                    load_json_message(data)

    def test_hex_case_does_not_change_raw_bytes_or_crc(self):
        packet = dict(GOLDEN)
        packet["data_hex"] = packet["data_hex"].upper()
        self.assertEqual(decode_lidar_packet(packet).raw, bytes.fromhex(GOLDEN["data_hex"]))

    def test_range_bounds_are_validated(self):
        for low, high in [(float("nan"), 8), (0.12, float("inf")), (2, 1), (-1, 8), (False, 8)]:
            with self.subTest(bounds=(low, high)):
                with self.assertRaises(ValueError):
                    decode_lidar_packet(GOLDEN, low, high)


class SequenceTests(unittest.TestCase):
    def test_first_nonzero_sequence_is_baseline(self):
        tracker = udp_receiver.SequenceTracker()
        self.assertEqual(tracker.observe("10.0.0.1", "a", 500), udp_receiver.SequenceObservation("first"))
        self.assertEqual(tracker.observe("10.0.0.1", "a", 501), udp_receiver.SequenceObservation("new"))

    def test_gap_duplicate_and_late_are_separate(self):
        tracker = udp_receiver.SequenceTracker()
        tracker.observe("10.0.0.1", "a", 10)
        self.assertEqual(tracker.observe("10.0.0.1", "a", 13), udp_receiver.SequenceObservation("new", 2))
        self.assertEqual(tracker.observe("10.0.0.1", "a", 13).kind, "duplicate")
        self.assertEqual(tracker.observe("10.0.0.1", "a", 11).kind, "out_of_order")
        self.assertEqual(tracker.observe("10.0.0.1", "a", 11).kind, "duplicate")
        self.assertEqual(tracker.observe("10.0.0.1", "a", 14).kind, "new")

    def test_wrap_and_session_separation(self):
        tracker = udp_receiver.SequenceTracker()
        tracker.observe("10.0.0.1", "a", 0xFFFFFFFF)
        self.assertEqual(tracker.observe("10.0.0.1", "a", 0), udp_receiver.SequenceObservation("new"))
        self.assertEqual(tracker.observe("10.0.0.1", "b", 0).kind, "first")
        self.assertEqual(tracker.observe("10.0.0.2", "a", 0).kind, "first")

    def test_bounded_history_never_treats_old_repeat_as_fresh(self):
        tracker = udp_receiver.SequenceTracker(history_size=2, max_sessions=2)
        for seq in [10, 11, 12]:
            tracker.observe("one", "a", seq)
        self.assertEqual(tracker.observe("one", "a", 10).kind, "out_of_order")
        tracker.observe("two", "a", 0)
        tracker.observe("three", "a", 0)
        self.assertEqual(len(tracker.sessions), 2)


class FakeSocket:
    def __init__(self, messages):
        self.messages = iter(messages)
        self.replies = []
        self.closed = False
        self.receive_sizes = []

    def bind(self, address):
        self.bound = address

    def settimeout(self, timeout):
        pass

    def setblocking(self, blocking):
        pass

    def getsockname(self):
        return ("0.0.0.0", 8888)

    def recvfrom(self, size):
        self.receive_sizes.append(size)
        try:
            return next(self.messages)
        except StopIteration:
            raise KeyboardInterrupt

    def sendto(self, data, peer):
        self.replies.append((data, peer))
        return len(data)

    def close(self):
        self.closed = True


class ReceiverTests(unittest.TestCase):
    def test_oversize_datagram_is_rejected_then_valid_data_recovers(self):
        data = json.dumps(GOLDEN).encode()
        overlong = b" " * (1201 - len(data)) + data
        peer = ("10.0.0.2", 5000)
        fake = FakeSocket([(overlong, peer), (data, peer)])
        output = io.StringIO()
        with patch.object(udp_receiver.socket, "socket", return_value=fake), patch.object(udp_receiver.select, "select", return_value=([fake], [], [])), patch.object(udp_receiver, "show_mac_addresses"), patch.object(sys, "argv", ["udp_receiver.py", "--no-sync", "--legacy"]), redirect_stdout(output):
            self.assertEqual(udp_receiver.main(), 0)
        self.assertEqual(set(fake.receive_sizes), {1201})
        self.assertIn("有效数据包=1 无效数据包=1", output.getvalue())
        self.assertIn("雷达新包=1", output.getvalue())
        self.assertFalse(fake.replies)

    def test_lidar_has_no_per_packet_ack_and_is_recorded_only_when_requested(self):
        raw = bytes.fromhex(GOLDEN["data_hex"])
        valid = [envelope(raw, seq=55), envelope(raw, seq=55), envelope(raw, seq=54), envelope(raw, seq=57)]
        bad_crc = dict(valid[0], crc32="00000000")
        heartbeat = {"type": "heartbeat", "version": 1, "boot_id": "0123456789abcdef", "seq": 0, "uptime_us": 123}
        packets = [bad_crc] + valid + [heartbeat]
        peer = ("10.0.0.2", 5000)
        messages = [(b"not-json", peer)] + [(json.dumps(packet).encode(), peer) for packet in packets]
        fake = FakeSocket(messages)
        with tempfile.TemporaryDirectory() as directory:
            record_path = Path(directory) / "capture.jsonl"
            output = io.StringIO()
            with patch.object(udp_receiver.socket, "socket", return_value=fake), patch.object(udp_receiver.select, "select", return_value=([fake], [], [])), patch.object(udp_receiver, "show_mac_addresses"), patch.object(sys, "argv", ["udp_receiver.py", "--no-sync", "--legacy", "--record", str(record_path)]), redirect_stdout(output):
                self.assertEqual(udp_receiver.main(), 0)
            self.assertTrue(fake.closed)
            self.assertEqual(fake.replies, [(b"ACK 0123456789abcdef 0", peer)])
            records = [json.loads(line) for line in record_path.read_text().splitlines()]
            self.assertEqual(len(records), 5)
            self.assertEqual(records[0]["packet"], valid[0])
            self.assertEqual(records[0]["peer"], {"ip": "10.0.0.2", "port": 5000})
            self.assertIs(type(records[0]["host_receive_unix_ns"]), int)
            text = output.getvalue()
            self.assertEqual(text.count("LIDAR packets/s="), 1)
            self.assertIn("crc_errors=1 invalid_errors=1", text)
            self.assertIn("雷达新包=2 重复=1 乱序=1 序号缺口估计=1", text)

    def test_heartbeat_compatibility_and_no_default_record_write(self):
        heartbeat = {"type": "heartbeat", "version": 1, "boot_id": "0123456789abcdef", "seq": 0, "uptime_us": 100}
        data = json.dumps(heartbeat).encode()
        self.assertEqual(udp_receiver.decode_heartbeat(data), heartbeat)
        peer = ("10.0.0.2", 1234)
        fake = FakeSocket([(data, peer)])
        with patch.object(udp_receiver.socket, "socket", return_value=fake), patch.object(udp_receiver.select, "select", return_value=([fake], [], [])), patch.object(udp_receiver, "show_mac_addresses"), patch.object(Path, "open", side_effect=AssertionError("unexpected record write")), patch.object(sys, "argv", ["udp_receiver.py", "--no-sync", "--legacy"]), redirect_stdout(io.StringIO()):
            self.assertEqual(udp_receiver.main(), 0)
        self.assertEqual(fake.replies, [(b"ACK 0123456789abcdef 0", peer)])


if __name__ == "__main__":
    unittest.main()
