"""Wire CRC, strict metadata and MPU6050 signed/SI conversion tests."""
import math
from pathlib import Path
import struct
import sys
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from lidar_protocol import LidarCRCError
from sensor_protocol import compute_imu_crc32, decode_imu_packet, SensorProtocolError


def message(raw=None, **overrides):
    if raw is None:
        raw = struct.pack(">7h", 16384, -8192, 0, 340, 131, -262, 0)
    packet = dict(type="imu_packet", version=1, boot_id="0123456789abcdef", seq=42,
                  sample_us=123456789, uncertainty_us=17, read_end_us=123456900,
                  data_hex=raw.hex())
    packet.update(overrides)
    packet["crc32"] = compute_imu_crc32(packet["boot_id"], packet["seq"], packet["sample_us"],
                                           packet["uncertainty_us"], packet["read_end_us"], raw)
    return packet


class ImuProtocolTests(unittest.TestCase):
    def test_si_units_sign_axes_and_temperature(self):
        parsed = decode_imu_packet(message())
        self.assertAlmostEqual(parsed.acceleration_m_s2[0], 9.80665)
        self.assertAlmostEqual(parsed.acceleration_m_s2[1], -4.903325)
        self.assertEqual(parsed.acceleration_m_s2[2], 0)
        self.assertAlmostEqual(parsed.angular_velocity_rad_s[0], math.pi / 180)
        self.assertAlmostEqual(parsed.angular_velocity_rad_s[1], -2 * math.pi / 180)
        self.assertAlmostEqual(parsed.temperature_c, 37.53)
        self.assertEqual(parsed.raw, bytes.fromhex(message()["data_hex"]))

    def test_crc_independent_header_vector(self):
        packet = message()
        raw = bytes.fromhex(packet["data_hex"])
        header = struct.pack("<4sQIQIQH", b"SC1I", 0x0123456789ABCDEF,
                             42, 123456789, 17, 123456900, 14)
        self.assertEqual(len(header), 38)
        self.assertEqual(packet["crc32"], f"{zlib.crc32(header + raw):08x}")

    def test_crc_covers_each_metadata_field_and_raw(self):
        for field in ("boot_id", "seq", "sample_us", "uncertainty_us", "read_end_us", "data_hex"):
            packet = message()
            if field == "boot_id":
                packet[field] = "1123456789abcdef"
            elif field == "data_hex":
                packet[field] = "00" + packet[field][2:]
            else:
                packet[field] += 1
            with self.subTest(field=field), self.assertRaises(LidarCRCError):
                decode_imu_packet(packet)

    def test_integer_bool_float_negative_and_overflow_rejected(self):
        limits = {"version": 1, "seq": 0xFFFFFFFF, "sample_us": 0x7FFFFFFFFFFFFFFF,
                  "uncertainty_us": 0xFFFFFFFF, "read_end_us": 0x7FFFFFFFFFFFFFFF}
        for field, limit in limits.items():
            for value in (True, 1.0, -1, limit + 1, float("nan"), float("inf"), "1", None):
                packet = message()
                packet[field] = value
                with self.subTest(field=field, value=value), self.assertRaises(SensorProtocolError):
                    decode_imu_packet(packet)

    def test_format_and_read_order_rejected(self):
        for field, value in (("boot_id", "0123456789ABCDEF"), ("boot_id", "0" * 15),
                             ("data_hex", "00" * 13), ("data_hex", "gg" * 14),
                             ("crc32", "DEADBEEF"), ("type", "heartbeat"),
                             ("read_end_us", 1)):
            packet = message()
            packet[field] = value
            with self.subTest(field=field), self.assertRaises(SensorProtocolError):
                decode_imu_packet(packet)
        with self.assertRaises(SensorProtocolError):
            decode_imu_packet([])

    def test_boundaries_and_signed_minimum(self):
        raw = struct.pack(">7h", -32768, 32767, 0, -32768, -32768, 32767, 0)
        parsed = decode_imu_packet(message(raw, seq=0xFFFFFFFF,
            sample_us=0x7FFFFFFFFFFFFFFF, read_end_us=0x7FFFFFFFFFFFFFFF,
            uncertainty_us=0xFFFFFFFF))
        self.assertAlmostEqual(parsed.acceleration_m_s2[0], -2 * 9.80665)
        self.assertTrue(all(math.isfinite(value) for value in (*parsed.acceleration_m_s2,
                                                              *parsed.angular_velocity_rad_s)))


if __name__ == "__main__":
    unittest.main()
