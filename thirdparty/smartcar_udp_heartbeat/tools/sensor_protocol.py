"""Decode raw MPU6050 packets; no attitude, axis remapping or DLPF correction.

The sample timestamp is the midpoint of the firmware's data-ready polling
interval. Its uncertainty describes that interval, not IMU filter group delay.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
import struct
import zlib

try:
    from .lidar_protocol import LidarCRCError, LidarProtocolError
except ImportError:
    from lidar_protocol import LidarCRCError, LidarProtocolError

SensorProtocolError = LidarProtocolError
SensorCRCError = LidarCRCError
IMU_RAW_BYTES = 14
STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True)
class ParsedImuPacket:
    boot_id: str
    seq: int
    sample_us: int
    uncertainty_us: int
    read_end_us: int
    acceleration_m_s2: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]
    temperature_c: float
    raw: bytes


def _uint(value: object, name: str, maximum: int) -> int:
    if type(value) is not int or not 0 <= value <= maximum:
        raise SensorProtocolError(f"invalid {name}")
    return value


def _boot(value: object) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{16}", value):
        raise SensorProtocolError("invalid boot_id")
    return value


def compute_imu_crc32(boot_id: str, seq: int, sample_us: int,
                      uncertainty_us: int, read_end_us: int, raw: bytes) -> str:
    """Return the CRC-32/ISO-HDLC envelope digest, as eight lowercase hex digits."""
    _boot(boot_id)
    _uint(seq, "seq", 0xFFFFFFFF)
    _uint(sample_us, "sample_us", 0x7FFFFFFFFFFFFFFF)
    _uint(uncertainty_us, "uncertainty_us", 0xFFFFFFFF)
    _uint(read_end_us, "read_end_us", 0x7FFFFFFFFFFFFFFF)
    if read_end_us < sample_us:
        raise SensorProtocolError("read_end_us precedes sample_us")
    if not isinstance(raw, bytes) or len(raw) != IMU_RAW_BYTES:
        raise SensorProtocolError("IMU raw packet must be exactly 14 bytes")
    header = struct.pack("<4sQIQIQH", b"SC1I", int(boot_id, 16), seq,
                         sample_us, uncertainty_us, read_end_us, IMU_RAW_BYTES)
    return f"{zlib.crc32(header + raw) & 0xFFFFFFFF:08x}"


def decode_imu_packet(packet: dict) -> ParsedImuPacket:
    """Validate CRC/types/bounds, decode BE signed axes, and convert to SI.

    Firmware must use MPU6050 +/-2 g and +/-250 degrees/second full scales.
    Acceleration includes gravity; orientation and calibration are not inferred.
    """
    if not isinstance(packet, dict):
        raise SensorProtocolError("packet is not an object")
    if (packet.get("type") != "imu_packet" or
            type(packet.get("version")) is not int or packet["version"] != 1):
        raise SensorProtocolError("unsupported type/version")
    boot_id = _boot(packet.get("boot_id"))
    seq = _uint(packet.get("seq"), "seq", 0xFFFFFFFF)
    sample_us = _uint(packet.get("sample_us"), "sample_us", 0x7FFFFFFFFFFFFFFF)
    uncertainty_us = _uint(packet.get("uncertainty_us"), "uncertainty_us", 0xFFFFFFFF)
    read_end_us = _uint(packet.get("read_end_us"), "read_end_us", 0x7FFFFFFFFFFFFFFF)
    data_hex = packet.get("data_hex")
    if not isinstance(data_hex, str) or not re.fullmatch(r"[0-9a-fA-F]{28}", data_hex):
        raise SensorProtocolError("invalid IMU data_hex")
    crc32 = packet.get("crc32")
    if not isinstance(crc32, str) or not re.fullmatch(r"[0-9a-f]{8}", crc32):
        raise SensorProtocolError("invalid crc32")
    raw = bytes.fromhex(data_hex)
    if compute_imu_crc32(boot_id, seq, sample_us, uncertainty_us, read_end_us, raw) != crc32:
        raise SensorCRCError("IMU transport CRC32 mismatch")
    ax, ay, az, temperature, gx, gy, gz = struct.unpack(">7h", raw)
    acceleration = tuple(value * STANDARD_GRAVITY / 16384.0 for value in (ax, ay, az))
    angular_velocity = tuple(math.radians(value / 131.0) for value in (gx, gy, gz))
    temperature_c = temperature / 340.0 + 36.53
    if not all(math.isfinite(value) for value in (*acceleration, *angular_velocity, temperature_c)):
        raise SensorProtocolError("non-finite decoded IMU value")
    return ParsedImuPacket(boot_id, seq, sample_us, uncertainty_us, read_end_us,
                           acceleration, angular_velocity, temperature_c, raw)
