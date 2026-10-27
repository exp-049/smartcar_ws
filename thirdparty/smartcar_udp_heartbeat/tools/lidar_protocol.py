"""Validate staged X3 PRO UART-over-UDP messages using only the stdlib.

Accepts standard no-intensity AA55 CT LSN FSA LSA CS + 2*N packets, with
little-endian words and the complete eight-bit CT in the XOR. Distance is raw
sample / 4000 metres. Angles are linearly interpolated clockwise through zero
in the lidar's original coordinate system. No model-specific secondary angle
correction or ROS frame conversion is applied. One packet need not be one scan.
rx_complete_us is MCU monotonic time after validation, not physical first-ray
sampling time and not ROS/UTC time. CRC32 is the standard zlib/binascii CRC over
struct.pack('<4sQIQH', b'SC1L', int(boot_id, 16), seq, rx_complete_us, len(raw))
followed by raw bytes, protecting both packet bytes and envelope metadata.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import struct
from typing import Optional
import zlib

MAX_RAW_BYTES = 520
MAX_DATAGRAM_BYTES = 1200
DEFAULT_RANGE_MIN_M = 0.12
DEFAULT_RANGE_MAX_M = 8.0


class LidarProtocolError(ValueError):
    """Malformed metadata or an invalid UART packet."""


class LidarCRCError(LidarProtocolError):
    """Envelope CRC32 does not match its metadata and raw bytes."""


@dataclass(frozen=True)
class ParsedLidarPacket:
    boot_id: str
    seq: int
    rx_complete_us: int
    raw: bytes
    ct: int
    start_of_scan: bool
    angles_cw_deg: tuple[float, ...]
    ranges_m: tuple[Optional[float], ...]

    @property
    def point_count(self) -> int:
        return len(self.ranges_m)


def _reject_constant(value: str) -> None:
    raise LidarProtocolError(f"non-standard JSON constant: {value}")


def load_json_message(data: bytes) -> dict:
    packet = json.loads(data.decode("utf-8"), parse_constant=_reject_constant)
    if not isinstance(packet, dict):
        raise LidarProtocolError("packet is not an object")
    return packet


def _validate_uint(value: object, name: str, maximum: int) -> int:
    if type(value) is not int or not 0 <= value <= maximum:
        raise LidarProtocolError(f"invalid {name}")
    return value


def _validate_boot_id(value: object) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{16}", value):
        raise LidarProtocolError("invalid boot_id")
    return value


def compute_lidar_crc32(boot_id: str, seq: int, rx_complete_us: int, raw: bytes) -> str:
    """Return the eight-character lowercase CRC32 of the transport envelope."""
    _validate_boot_id(boot_id)
    _validate_uint(seq, "seq", 0xFFFFFFFF)
    _validate_uint(rx_complete_us, "rx_complete_us", 0x7FFFFFFFFFFFFFFF)
    if not isinstance(raw, bytes) or len(raw) > MAX_RAW_BYTES:
        raise LidarProtocolError("invalid raw byte buffer")
    header = struct.pack("<4sQIQH", b"SC1L", int(boot_id, 16), seq, rx_complete_us, len(raw))
    return f"{zlib.crc32(header + raw) & 0xFFFFFFFF:08x}"


def decode_lidar_packet(
    packet: dict,
    range_min_m: float = DEFAULT_RANGE_MIN_M,
    range_max_m: float = DEFAULT_RANGE_MAX_M,
) -> ParsedLidarPacket:
    """Validate envelope/UART checksums and return raw-frame points.

    Distances outside inclusive default 0.12..8 m become None; a valid transport
    packet may contain no valid measurements. This does not produce LaserScan.
    """
    if not isinstance(packet, dict):
        raise LidarProtocolError("packet is not an object")
    if packet.get("type") != "lidar_packet" or type(packet.get("version")) is not int or packet["version"] != 1:
        raise LidarProtocolError("unsupported type/version")
    boot_id = _validate_boot_id(packet.get("boot_id"))
    seq = _validate_uint(packet.get("seq"), "seq", 0xFFFFFFFF)
    rx_complete_us = _validate_uint(packet.get("rx_complete_us"), "rx_complete_us", 0x7FFFFFFFFFFFFFFF)
    data_hex = packet.get("data_hex")
    if (not isinstance(data_hex, str) or not 20 <= len(data_hex) <= MAX_RAW_BYTES * 2
            or len(data_hex) % 2 or not re.fullmatch(r"[0-9a-fA-F]+", data_hex)):
        raise LidarProtocolError("invalid data_hex")
    crc32 = packet.get("crc32")
    if not isinstance(crc32, str) or not re.fullmatch(r"[0-9a-f]{8}", crc32):
        raise LidarProtocolError("invalid crc32")
    raw = bytes.fromhex(data_hex)
    if compute_lidar_crc32(boot_id, seq, rx_complete_us, raw) != crc32:
        raise LidarCRCError("transport CRC32 mismatch")
    if raw[:2] != b"\xaa\x55":
        raise LidarProtocolError("invalid UART header")
    ct, count = raw[2], raw[3]
    if count == 0 or len(raw) != 10 + count * 2:
        raise LidarProtocolError("invalid UART packet length/sample count")
    first_angle, last_angle, checksum = struct.unpack_from("<HHH", raw, 4)
    if not first_angle & 1 or not last_angle & 1:
        raise LidarProtocolError("missing UART angle check bit")
    first_deg = (first_angle >> 1) / 64.0
    last_deg = (last_angle >> 1) / 64.0
    if first_deg > 360 or last_deg > 360:
        raise LidarProtocolError("UART angle out of range")
    samples = struct.unpack_from(f"<{count}H", raw, 10)
    calculated = 0x55AA ^ (ct | (count << 8)) ^ first_angle ^ last_angle
    for sample in samples:
        calculated ^= sample
    if calculated != checksum:
        raise LidarProtocolError("UART XOR checksum mismatch")
    for value in (range_min_m, range_max_m):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("range bounds must be finite numbers")
    if range_min_m < 0 or range_max_m < range_min_m:
        raise ValueError("invalid range bounds")
    span = last_deg - first_deg
    if span < 0:
        span += 360.0
    step = span / (count - 1) if count > 1 else 0.0
    angles = tuple((first_deg + index * step) % 360.0 for index in range(count))
    ranges = tuple(
        sample / 4000.0 if sample and range_min_m <= sample / 4000.0 <= range_max_m else None
        for sample in samples
    )
    return ParsedLidarPacket(boot_id, seq, rx_complete_us, raw, ct, bool(ct & 1), angles, ranges)
