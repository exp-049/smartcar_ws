#!/usr/bin/env python3
"""Receive heartbeat and staged lidar_packet JSON datagrams using the stdlib.

By default the PC actively discovers and subscribes to exactly one MCU. Sensor
data and heartbeat ACKs require an active matching device/boot lease. --legacy
keeps the old passive receiver for firmware without the subscription protocol.
Lidar packets are validated and aggregated
once a second, never ACKed individually. Lidar and heartbeat use independent seq
streams. Sessions are identified by source IP and boot_id. Duplicate and old/
out-of-order packets are counted separately and excluded from fresh sample rates.
The bounded history distinguishes duplicates within 8192 unique sequence numbers;
older repeats are safely classified as old/out-of-order. gaps_est counts observed
forward skips, including possible sender-side drops and reordering. It is not a
final network packet-loss rate and is not reduced when late packets arrive.

This does not publish ROS messages or certify complete scans, corrected angles,
physical acquisition timing, or the ten-minute acceptance. --record writes valid
original messages only when requested. host_receive_unix_ns is host receipt wall
clock time, not physical acquisition/ROS time. Records do not include system
network configuration or Wi-Fi credentials.
"""
from __future__ import annotations

import argparse
from collections import OrderedDict, deque
from dataclasses import dataclass
import ipaddress
import json
import math
from pathlib import Path
import re
import select
import socket
import subprocess
import time
from typing import Optional

try:
    from .lidar_protocol import MAX_DATAGRAM_BYTES, LidarCRCError, decode_lidar_packet, load_json_message
    from .sensor_protocol import decode_imu_packet
    from .clock_sync import SyncClient
    from .discovery import DiscoveryClient
except ImportError:
    from lidar_protocol import MAX_DATAGRAM_BYTES, LidarCRCError, decode_lidar_packet, load_json_message
    from sensor_protocol import decode_imu_packet
    from clock_sync import SyncClient
    from discovery import DiscoveryClient


def show_mac_addresses() -> None:
    """Show non-loopback LAN IPv4 candidates, using macOS interface names."""
    labels = {}
    try:
        result = subprocess.run(
            ["/usr/sbin/networksetup", "-listallhardwareports"],
            capture_output=True, text=True, check=False,
        )
        label = ""
        for line in result.stdout.splitlines():
            if line.startswith("Hardware Port: "):
                label = line.split(": ", 1)[1]
            elif line.startswith("Device: "):
                labels[line.split(": ", 1)[1]] = label
        result = subprocess.run(
            ["/sbin/ifconfig"], capture_output=True, text=True, check=False,
        )
        interface = ""
        found = False
        for line in result.stdout.splitlines():
            if line and not line[0].isspace():
                interface = line.split(":", 1)[0]
            match = re.search(r"\binet (\d+\.\d+\.\d+\.\d+)\b", line)
            if not match or not interface.startswith(("en", "bridge")):
                continue
            address = ipaddress.IPv4Address(match.group(1))
            if address.is_loopback or address.is_link_local:
                continue
            print(f"Mac IPv4 候选：{address}  接口={interface}  {labels.get(interface, '')}", flush=True)
            found = True
        if not found:
            print("未能自动列出局域网 IPv4；请在 macOS 网络设置中查看当前连接的 IP 地址。", flush=True)
    except (OSError, ValueError):
        print("请在 macOS 网络设置中查看当前连接的局域网 IPv4 地址。", flush=True)


def validate_heartbeat(packet: dict) -> dict:
    if packet.get("type") != "heartbeat" or type(packet.get("version")) is not int or packet["version"] != 1:
        raise ValueError("unsupported type/version")
    if not isinstance(packet.get("boot_id"), str) or not re.fullmatch(r"[0-9a-f]{16}", packet["boot_id"]):
        raise ValueError("invalid boot_id")
    if type(packet.get("seq")) is not int or not 0 <= packet["seq"] <= 0xFFFFFFFF:
        raise ValueError("invalid sequence")
    if type(packet.get("uptime_us")) is not int or not 0 <= packet["uptime_us"] <= 0x7FFFFFFFFFFFFFFF:
        raise ValueError("invalid uptime")
    return packet


def decode_heartbeat(data: bytes) -> dict:
    """Compatibility entry point for the original heartbeat-only receiver."""
    return validate_heartbeat(load_json_message(data))


def lease_allows_packet(status, peer, boot_id=None) -> bool:
    """A manual IP does not bypass subscription; check current lease per packet."""
    return bool(status.active and peer[0] == status.source_ip and
                (boot_id is None or boot_id == status.boot_id))


def drain_pending_data(sock, limit=256) -> int:
    """Discard a bounded batch from an already nonblocking data socket.

    Returning limit means the caller must keep admission paused and try another
    bounded batch next turn. This flushes local backlog, not packets in flight.
    """
    if type(limit) is not int or limit < 1:
        raise ValueError("drain limit must be positive")
    drained = 0
    for _ in range(limit):
        try:
            sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
        except BlockingIOError:
            break
        except InterruptedError:
            continue
        except OSError:
            break
        drained += 1
    return drained


@dataclass(frozen=True)
class SequenceObservation:
    kind: str
    gap_estimate: int = 0


class SequenceTracker:
    """Bounded per-(source IP, boot_id) tracking for one message type."""

    def __init__(self, history_size: int = 8192, max_sessions: int = 128):
        if history_size < 1 or max_sessions < 1:
            raise ValueError("tracking limits must be positive")
        self.history_size = history_size
        self.max_sessions = max_sessions
        self.sessions = OrderedDict()

    def observe(self, source_ip: str, boot_id: str, sequence: int) -> SequenceObservation:
        key = (source_ip, boot_id)
        if key not in self.sessions:
            self.sessions[key] = {"highest": sequence, "seen": {sequence}, "order": deque([sequence])}
            if len(self.sessions) > self.max_sessions:
                self.sessions.popitem(last=False)
            return SequenceObservation("first")
        state = self.sessions[key]
        self.sessions.move_to_end(key)
        if sequence in state["seen"]:
            return SequenceObservation("duplicate")
        delta = (sequence - state["highest"]) & 0xFFFFFFFF
        if 0 < delta < 0x80000000:
            state["highest"] = sequence
            observation = SequenceObservation("new", delta - 1)
        else:
            observation = SequenceObservation("out_of_order")
        state["seen"].add(sequence)
        state["order"].append(sequence)
        if len(state["order"]) > self.history_size:
            state["seen"].remove(state["order"].popleft())
        return observation


@dataclass
class LidarStatistics:
    fresh_packets: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    gaps_estimate: int = 0
    crc_errors: int = 0
    invalid_errors: int = 0
    window_packets: int = 0
    window_points: int = 0
    window_valid_ranges: int = 0
    window_min_m: Optional[float] = None
    last_seq: Optional[int] = None
    have_traffic: bool = False

    def report(self, elapsed: float) -> None:
        elapsed = max(elapsed, 0.000001)
        minimum = "none" if self.window_min_m is None else f"{self.window_min_m:.3f}"
        last = "none" if self.last_seq is None else str(self.last_seq)
        print(
            f"LIDAR packets/s={self.window_packets / elapsed:.1f} "
            f"points/s={self.window_points / elapsed:.1f} "
            f"valid_ranges/s={self.window_valid_ranges / elapsed:.1f} "
            f"min_m={minimum} last_seq={last} "
            f"crc_errors={self.crc_errors} invalid_errors={self.invalid_errors} "
            f"duplicate={self.duplicates} out_of_order={self.out_of_order} "
            f"gaps_est={self.gaps_estimate}", flush=True,
        )
        self.window_packets = 0
        self.window_points = 0
        self.window_valid_ranges = 0
        self.window_min_m = None


@dataclass
class ImuStatistics:
    fresh_packets: int = 0
    duplicates: int = 0
    out_of_order: int = 0
    gaps_estimate: int = 0
    window_packets: int = 0
    last: object = None

    def report(self, elapsed: float) -> None:
        if self.last is None:
            return
        sample = self.last
        acceleration = ','.join(f'{v:.3f}' for v in sample.acceleration_m_s2)
        gyro = ','.join(f'{v:.4f}' for v in sample.angular_velocity_rad_s)
        print(f'IMU packets/s={self.window_packets / max(elapsed, 0.000001):.1f} '
              f'accel_m_s2=({acceleration}) gyro_rad_s=({gyro}) '
              f'temp_c={sample.temperature_c:.2f} seq={sample.seq} '
              f'ready_bracket_half_us={sample.uncertainty_us} '
              f'duplicate={self.duplicates} out_of_order={self.out_of_order} '
              f'gaps_est={self.gaps_estimate}', flush=True)
        self.window_packets = 0


def main() -> int:
    parser = argparse.ArgumentParser(description="ESP32-S3 UDP 心跳、雷达数据接收工具")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--duration", type=float, default=0, help="运行秒数；0 表示一直运行")
    parser.add_argument("--show-ips", action="store_true", help="仅列出 Mac 的局域网 IPv4")
    parser.add_argument("--record", type=Path, help="可选：将有效接收消息追加保存为 JSONL")
    parser.add_argument('--legacy', action='store_true', help='兼容旧固件：被动监听固定目标数据，不执行发现/订阅')
    parser.add_argument('--device-id', default='', help='可选：选择指定设备的12位MAC十六进制ID')
    parser.add_argument('--device-ip', help='可选：手工指定设备IPv4；自动模式仍须完成发现和订阅')
    parser.add_argument('--discovery-address', default='255.255.255.255', help='广播/定向广播地址；用于主动发现')
    parser.add_argument('--discovery-port', type=int, default=8890, help='设备发现与订阅端口')
    parser.add_argument('--sync-port', type=int, default=8889)
    parser.add_argument('--no-sync', action='store_true', help='关闭本机诊断工具的自动对时')
    parser.add_argument('--view', action='store_true', help='在本机浏览器预览最新一圈雷达数据')
    parser.add_argument('--view-port', type=int, default=8765, help='本机预览端口；仅绑定 127.0.0.1')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535 or not 1 <= args.sync_port <= 65535 or not 1024 <= args.discovery_port <= 65535 or not 1 <= args.view_port <= 65535 or args.duration < 0 or not math.isfinite(args.duration):
        parser.error("port/duration out of range")
    if args.legacy and args.device_id:
        parser.error('--device-id requires active discovery; omit --legacy')
    if args.device_ip:
        try:
            args.device_ip = str(ipaddress.IPv4Address(args.device_ip))
        except ValueError:
            parser.error('device-ip must be an IPv4 address')
    if args.show_ips:
        show_mac_addresses()
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.bind, args.port))
    except OSError as error:
        sock.close()
        parser.exit(1, f"无法监听 UDP {args.bind}:{args.port}：{error}\n")
    record_file = None
    if args.record is not None:
        try:
            args.record.parent.mkdir(parents=True, exist_ok=True)
            record_file = args.record.open("a", encoding="utf-8", buffering=1)
        except OSError as error:
            sock.close()
            parser.exit(1, f"无法打开记录文件：{error}\n")
    sock.setblocking(False)
    discovery = None
    try:
        if not args.legacy:
            discovery = DiscoveryClient(data_port=sock.getsockname()[1], device_id=args.device_id,
                                         target_ip=args.device_ip or '',
                                         broadcast_address=args.discovery_address, port=args.discovery_port)
    except (OSError, ValueError) as error:
        sock.close()
        if record_file is not None:
            record_file.close()
        parser.exit(1, f'无法初始化主动发现：{error}\n')
    viewer = None
    if args.view:
        try:
            from live_viewer import LiveViewer
            viewer = LiveViewer(port=args.view_port)
            url = viewer.start()
            print(f'实时雷达预览：{url}', flush=True)
        except (OSError, RuntimeError, ImportError) as error:
            sock.close()
            if discovery is not None:
                discovery.close()
            if record_file is not None:
                record_file.close()
            if viewer is not None:
                viewer.close()
            parser.exit(1, f'无法启动本机预览：{error}\n')
    print(f"UDP receiver: bind={args.bind} port={sock.getsockname()[1]}", flush=True)
    show_mac_addresses()
    print(('旧固件被动监听模式。' if args.legacy else
           '主动发现并订阅设备；无需配置电脑或小车IP。多设备请用 --device-id；广播不通可用 --device-ip/--discovery-address。') +
          '心跳回复ACK，雷达每秒汇总且不逐包ACK；Control+C结束。', flush=True)
    received_count = 0
    invalid_count = 0
    ack_failed = 0
    heartbeat_tracker = SequenceTracker()
    lidar_tracker = SequenceTracker()
    imu_tracker = SequenceTracker()
    lidar_stats = LidarStatistics()
    imu_stats = ImuStatistics()
    sync_client = None
    endpoint = None
    discovery_note = None
    unauthorized_count = 0
    pending_endpoint_drain = False
    started = time.monotonic()
    last_notice = started
    last_report = started
    exit_code = 0

    def refresh_subscription():
        nonlocal sync_client, endpoint, heartbeat_tracker, lidar_tracker, imu_tracker, discovery_note, pending_endpoint_drain
        if discovery is None:
            return None
        status = discovery.poll()
        key = ((status.source_ip, status.boot_id, status.device_id, status.session_id, status.sync_port)
               if status.active else None)
        if key != endpoint:
            pending_endpoint_drain = True
            if sync_client is not None:
                sync_client.close()
                sync_client = None
            heartbeat_tracker, lidar_tracker, imu_tracker = SequenceTracker(), SequenceTracker(), SequenceTracker()
            lidar_stats.window_packets = lidar_stats.window_points = lidar_stats.window_valid_ranges = 0
            lidar_stats.window_min_m = lidar_stats.last_seq = None
            imu_stats.window_packets = 0
            imu_stats.last = None
            if viewer is not None:
                viewer.set_peer(None)
                viewer.set_sync(None)
                if status.active:
                    viewer.set_peer(status.source_ip)
            if status.active and not args.no_sync:
                sync_client = SyncClient(status.source_ip, port=status.sync_port)
            endpoint = key
        if pending_endpoint_drain:
            drained = drain_pending_data(sock)
            discovery.counters['data_queue_drained'] += drained
            pending_endpoint_drain = drained == 256
            if pending_endpoint_drain:
                discovery.counters['data_queue_drain_budget_hits'] += 1
        note = (status.state, status.reason, status.source_ip, status.boot_id, status.candidates)
        if note != discovery_note:
            print(f'DISCOVERY state={status.state} reason={status.reason} device_id={status.device_id} '
                  f'ip={status.source_ip} boot={status.boot_id} candidates={list(status.candidates)}', flush=True)
            discovery_note = note
        return status

    try:
        while args.duration == 0 or time.monotonic() - started < args.duration:
            now = time.monotonic()
            discovery_status = refresh_subscription()
            sync_status = sync_client.poll() if sync_client is not None else None
            if viewer is not None and sync_status is not None:
                viewer.set_sync(sync_status)
            if now - last_report >= 1:
                if lidar_stats.have_traffic:
                    lidar_stats.report(now - last_report)
                imu_stats.report(now - last_report)
                if sync_status is not None:
                    def milliseconds(value):
                        return 'none' if value is None else f'{value / 1_000_000:.3f}'
                    print(f'SYNC locked={sync_status.locked} reason={sync_status.reason} '
                          f'rtt_ms={milliseconds(sync_status.rtt_ns)} '
                          f'drift_ppm={sync_status.drift_ppm} '
                          f'estimate_uncertainty_ms={milliseconds(sync_status.uncertainty_ns)}', flush=True)
                if discovery_status is not None:
                    print(f'LEASE active={discovery_status.active} state={discovery_status.state} '
                          f'remaining_ms={discovery_status.lease_remaining_ms} '
                          f'unauthorized_datagrams={unauthorized_count} '
                          f'endpoint_queue_drained={discovery.counters["data_queue_drained"]}', flush=True)
                last_report = now
            try:
                wait_sockets = [sock]
                if discovery is not None:
                    wait_sockets.append(discovery.socket)
                if sync_client is not None:
                    wait_sockets.append(sync_client.socket)
                readable, _, _ = select.select(wait_sockets, [], [], 0.1)
                if discovery is not None and discovery.socket in readable:
                    discovery_status = refresh_subscription()
                if sync_client is not None and sync_client.socket in readable:
                    sync_client.poll()
                if sock not in readable:
                    raise socket.timeout
                data, peer = sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
                host_receive_unix_ns = time.time_ns()
            except (socket.timeout, BlockingIOError):
                if received_count == 0 and time.monotonic() - last_notice >= 10:
                    print(
                        (f"尚未收到有效数据：确认旧固件目标IP和UDP端口 {sock.getsockname()[1]}。" if args.legacy else
                         '尚未收到订阅数据：确认设备固件支持发现、处于同一AP；若广播被隔离，用 --device-ip 或定向 --discovery-address。'),
                        flush=True,
                    )
                    last_notice = time.monotonic()
                continue
            if args.legacy and args.device_ip and peer[0] != args.device_ip:
                continue
            lease = discovery.status() if discovery is not None else None
            if lease is not None and (pending_endpoint_drain or not lease_allows_packet(lease, peer)):
                unauthorized_count += 1
                continue
            if len(data) > MAX_DATAGRAM_BYTES:
                invalid_count += 1
                lidar_stats.invalid_errors += 1
                if viewer is not None:
                    viewer.on_bad_lidar()
                continue
            packet = None
            parsed_imu = None
            try:
                packet = load_json_message(data)
                if discovery is not None and not lease_allows_packet(discovery.status(), peer, packet.get('boot_id')):
                    unauthorized_count += 1
                    continue
                if packet.get("type") == "heartbeat":
                    validate_heartbeat(packet)
                    parsed_lidar = None
                elif packet.get("type") == "lidar_packet":
                    lidar_stats.have_traffic = True
                    parsed_lidar = decode_lidar_packet(packet)
                elif packet.get('type') == 'imu_packet':
                    parsed_imu = decode_imu_packet(packet)
                    parsed_lidar = None
                else:
                    raise ValueError("unsupported packet type")
            except LidarCRCError:
                invalid_count += 1
                lidar_stats.crc_errors += 1
                if viewer is not None and packet is not None and packet.get('type') == 'lidar_packet':
                    viewer.on_bad_lidar()
                continue
            except (ValueError, UnicodeError, RecursionError):
                invalid_count += 1
                lidar_stats.invalid_errors += 1
                if viewer is not None and (packet is None or packet.get('type') == 'lidar_packet'):
                    viewer.on_bad_lidar()
                continue

            if discovery is not None and not lease_allows_packet(discovery.status(), peer, packet['boot_id']):
                unauthorized_count += 1
                continue
            received_count += 1
            if args.legacy and endpoint != (peer[0], packet['boot_id']):
                if sync_client is not None:
                    sync_client.close()
                    sync_client = None
                heartbeat_tracker, lidar_tracker, imu_tracker = SequenceTracker(), SequenceTracker(), SequenceTracker()
                if viewer is not None:
                    viewer.set_peer(None)
                    viewer.set_sync(None)
                    viewer.set_peer(peer[0])
                endpoint = (peer[0], packet['boot_id'])
            if args.legacy and sync_client is None and not args.no_sync:
                sync_client = SyncClient(peer[0], port=args.sync_port)
            if record_file is not None:
                record = {
                    "host_receive_unix_ns": host_receive_unix_ns,
                    "peer": {"ip": peer[0], "port": peer[1]},
                    "packet": packet,
                }
                record_file.write(json.dumps(record, ensure_ascii=True, allow_nan=False) + "\n")

            if parsed_imu is not None:
                observation = imu_tracker.observe(peer[0], parsed_imu.boot_id, parsed_imu.seq)
                if observation.kind == 'duplicate':
                    imu_stats.duplicates += 1
                    continue
                if observation.kind == 'out_of_order':
                    imu_stats.out_of_order += 1
                    continue
                imu_stats.fresh_packets += 1
                imu_stats.gaps_estimate += observation.gap_estimate
                imu_stats.window_packets += 1
                imu_stats.last = parsed_imu
                if viewer is not None:
                    viewer.on_imu(parsed_imu)
                continue

            if parsed_lidar is not None:
                if viewer is not None:
                    viewer.on_lidar(parsed_lidar)
                observation = lidar_tracker.observe(peer[0], parsed_lidar.boot_id, parsed_lidar.seq)
                if observation.kind == "duplicate":
                    lidar_stats.duplicates += 1
                    continue
                if observation.kind == "out_of_order":
                    lidar_stats.out_of_order += 1
                    continue
                lidar_stats.fresh_packets += 1
                lidar_stats.gaps_estimate += observation.gap_estimate
                lidar_stats.last_seq = parsed_lidar.seq
                lidar_stats.window_packets += 1
                lidar_stats.window_points += parsed_lidar.point_count
                valid = [distance for distance in parsed_lidar.ranges_m if distance is not None]
                lidar_stats.window_valid_ranges += len(valid)
                if valid:
                    packet_minimum = min(valid)
                    if lidar_stats.window_min_m is None or packet_minimum < lidar_stats.window_min_m:
                        lidar_stats.window_min_m = packet_minimum
                continue

            observation = heartbeat_tracker.observe(peer[0], packet["boot_id"], packet["seq"])
            note = {
                "first": "新启动/新设备", "duplicate": "重复心跳", "out_of_order": "乱序心跳",
            }.get(observation.kind, "")
            if observation.gap_estimate:
                note = f"观察到序号缺口估计 {observation.gap_estimate}（不等于最终丢包率）"
            if discovery is not None and not lease_allows_packet(discovery.status(), peer, packet['boot_id']):
                unauthorized_count += 1
                continue
            acknowledgement = f"ACK {packet['boot_id']} {packet['seq']}".encode("ascii")
            try:
                sock.sendto(acknowledgement, peer)
                ack_status = "已回复 ACK"
            except OSError as error:
                ack_failed += 1
                ack_status = f"回复失败：{error}"
            print(
                f"RX #{received_count} from={peer[0]}:{peer[1]} "
                f"boot={packet['boot_id']} seq={packet['seq']} "
                f"uptime={packet['uptime_us'] / 1_000_000:.3f}s {ack_status} {note}", flush=True,
            )
    except KeyboardInterrupt:
        pass
    except OSError as error:
        print(f"接收或记录失败：{error}", flush=True)
        exit_code = 1
    finally:
        if discovery is not None:
            discovery.close()
        sock.close()
        if viewer is not None:
            viewer.close()
        if sync_client is not None:
            sync_client.close()
        if record_file is not None:
            record_file.close()
        if lidar_stats.have_traffic:
            lidar_stats.report(time.monotonic() - last_report)
        imu_stats.report(time.monotonic() - last_report)
        print(
            f"结束：有效数据包={received_count} 无效数据包={invalid_count} ACK发送失败={ack_failed} "
            f"雷达新包={lidar_stats.fresh_packets} 重复={lidar_stats.duplicates} "
            f"乱序={lidar_stats.out_of_order} 序号缺口估计={lidar_stats.gaps_estimate} "
            f"IMU新包={imu_stats.fresh_packets} IMU缺口估计={imu_stats.gaps_estimate}", flush=True,
        )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
