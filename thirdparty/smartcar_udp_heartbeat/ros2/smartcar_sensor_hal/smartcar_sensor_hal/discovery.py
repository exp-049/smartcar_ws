"""Active IPv4 discovery and a renewable, single-subscriber MCU lease.

All methods are nonblocking. Only a matching discovery/control nonce can change
the endpoint. The first chosen 48-bit device ID stays pinned across reconnects;
a broadcast search that finds several devices requires an explicit device_id.
This protocol identifies a session; it is not cryptographic authentication.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import ipaddress
import re
import secrets
import socket
import struct
import time
from typing import Optional

DISCOVER = struct.Struct("<4sQ")
OFFER = struct.Struct("<4sQQQHH")
SUBSCRIBE = struct.Struct("<4sQQQH")
ACK = struct.Struct("<4sQQQQHH")
LEASE_MS = 6000


@dataclass(frozen=True)
class DiscoveryStatus:
    active: bool
    state: str
    reason: str
    source_ip: Optional[str]
    boot_id: Optional[str]
    device_id: Optional[str]
    session_id: Optional[int]
    sync_port: Optional[int]
    lease_remaining_ms: int
    candidates: tuple[str, ...]
    capabilities: int = 0


@dataclass(frozen=True)
class _Offer:
    ip: str
    device: int
    boot: int
    sync_port: int
    capabilities: int


@dataclass(frozen=True)
class _Pending:
    nonce: int
    sent_ns: int
    kind: str


class DiscoveryClient:
    """Find one device and acquire its data lease using an independent socket.

    target_ip requests unicast discovery for networks that suppress broadcasts.
    status() never performs I/O and expires an active lease immediately according
    to the monotonic clock, even between calls to poll(). now_ns is test-only
    clock injection; normal callers omit it.
    """

    def __init__(self, data_port=8888, device_id="", target_ip="",
                 broadcast_address="255.255.255.255", port=8890, sock=None):
        if type(data_port) is not int or not 1024 <= data_port <= 65535:
            raise ValueError("data_port must be 1024..65535")
        if type(port) is not int or not 1024 <= port <= 65535:
            raise ValueError("discovery port must be 1024..65535")
        if not isinstance(device_id, str):
            raise ValueError("device_id must be 12 hexadecimal MAC digits")
        device_id = device_id.lower()
        if device_id and (not re.fullmatch(r"[0-9a-f]{12}", device_id) or int(device_id, 16) == 0):
            raise ValueError("device_id must be 12 nonzero hexadecimal MAC digits")
        self.data_port = data_port
        self.port = port
        self.target_ip = str(ipaddress.IPv4Address(target_ip)) if target_ip else ""
        self.broadcast_address = str(ipaddress.IPv4Address(broadcast_address))
        self._selected_id = int(device_id, 16) if device_id else None
        self._explicit_id = bool(device_id)
        self.socket = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise
        self.counters = Counter()
        self._state, self._reason = "discovering", "waiting_for_offers"
        self._search_reason = "waiting_for_offers"
        self._closed = False
        self._offer = self._last_offer = None
        self._resume_offer = self._resume_session = None
        self._session = None
        self._pending = None
        self._discover_nonce = None
        self._collect_until_ns = None
        self._candidates = {}
        self._ambiguous_candidates = set()
        self._ambiguous = False
        self._next_discover_ns = 0
        self._next_control_ns = 0
        self._subscribe_started_ns = 0
        self._lease_deadline_ns = 0
        self._renew_failed = False

    @staticmethod
    def _now(now_ns):
        now = time.monotonic_ns() if now_ns is None else now_ns
        if type(now) is not int or not 0 <= now <= 0xFFFFFFFFFFFFFFFF:
            raise ValueError("invalid host monotonic time")
        return now

    @staticmethod
    def _nonce():
        value = secrets.randbits(64)
        return value if value else 1

    def _send(self, data, ip):
        try:
            if self.socket.sendto(data, (ip, self.port)) != len(data):
                raise OSError("incomplete discovery datagram")
            return True
        except OSError:
            self.counters["send_errors"] += 1
            return False

    def _begin_discovery(self, now):
        self._discover_nonce = self._nonce()
        self._collect_until_ns = now + 700_000_000
        self._candidates.clear()
        self._next_discover_ns = now + 1_000_000_000
        self._state = "ambiguous" if self._ambiguous else "discovering"
        self._reason = "multiple_devices; set device_id" if self._ambiguous else self._search_reason
        if self._send(DISCOVER.pack(b"SCD1", self._discover_nonce), self.target_ip or self.broadcast_address):
            self.counters["discover_sent"] += 1

    def _send_control(self, now):
        nonce = self._nonce()
        data = SUBSCRIBE.pack(b"SCS1", nonce, self._offer.boot, self._session, self.data_port)
        kind = "renew" if self._lease_deadline_ns else "subscribe"
        if self._send(data, self._offer.ip):
            self._pending = _Pending(nonce, now, kind)
            self.counters[f"{kind}_sent"] += 1
        self._next_control_ns = now + (2_000_000_000 if kind == "renew" else 1_000_000_000)

    def _lose_endpoint(self, now, reason):
        self.counters[reason] += 1
        # A lost ACK does not revoke the device's lease. Preserve our session
        # so rediscovery of the same endpoint can renew it instead of competing
        # with our own still-valid subscription using a different session ID.
        if self._offer is not None and self._session is not None:
            self._resume_offer, self._resume_session = self._offer, self._session
        self._offer = None
        self._session = None
        self._pending = None
        self._lease_deadline_ns = 0
        self._renew_failed = False
        self._discover_nonce = self._collect_until_ns = None
        self._next_discover_ns = now
        self._search_reason = reason
        self._state, self._reason = "discovering", reason

    def _handle_offer(self, data, source, now):
        if len(data) != OFFER.size or self._discover_nonce is None or now > self._collect_until_ns:
            self.counters["invalid_or_late_offer"] += 1
            return
        _, nonce, device, boot, sync_port, capabilities = OFFER.unpack(data)
        if nonce != self._discover_nonce:
            self.counters["wrong_offer_nonce"] += 1
            return
        if not 0 < device <= 0xFFFFFFFFFFFF or not 1024 <= sync_port <= 65535 or capabilities & ~3:
            self.counters["invalid_offer_fields"] += 1
            return
        if self._selected_id is not None and device != self._selected_id:
            self.counters["device_id_filtered"] += 1
            return
        if len(self._candidates) >= 64 and device not in self._candidates:
            self.counters["candidate_limit"] += 1
            return
        self._candidates[device] = _Offer(source[0], device, boot, sync_port, capabilities)
        self.counters["offers"] += 1

    def _handle_ack(self, data, source, now):
        if len(data) != ACK.size or self._pending is None or self._offer is None:
            self.counters["unexpected_ack"] += 1
            return
        _, nonce, device, boot, session, lease_ms, result = ACK.unpack(data)
        pending, offer = self._pending, self._offer
        if (source[0] != offer.ip or nonce != pending.nonce or session != self._session or
                device != offer.device or now - pending.sent_ns > 1_000_000_000):
            self.counters["ack_identity_mismatch"] += 1
            return
        if result > 3 or (result != 2 and boot != offer.boot):
            self.counters["invalid_ack_fields"] += 1
            return
        if result == 0 and (boot != offer.boot or lease_ms != LEASE_MS):
            self.counters["invalid_ack_fields"] += 1
            return
        self._pending = None
        if result == 0:
            # Conservative local expiry: the request send time precedes the
            # device renewing the lease, unlike ACK receipt time.
            self._lease_deadline_ns = pending.sent_ns + lease_ms * 1_000_000
            self._last_offer = offer
            self._renew_failed = False
            self._next_control_ns = now + 2_000_000_000
            self._state, self._reason = "active", "subscribed"
            self.counters["accepted_acks"] += 1
        else:
            reason = {1: "busy", 2: "wrong_boot", 3: "bad_port"}[result]
            self._lose_endpoint(now, reason)
            # A busy device is retried, not replaced by a different car.
            self._next_discover_ns = now + 1_000_000_000
            self._state, self._reason = reason, reason

    def poll(self, now_ns=None):
        if self._closed:
            raise RuntimeError("discovery client is closed")
        now = self._now(now_ns)
        if self._pending is not None and now - self._pending.sent_ns > 1_000_000_000:
            if self._pending.kind == "renew":
                self._renew_failed = True
            self._pending = None
            self.counters["control_timeouts"] += 1
        if self._lease_deadline_ns:
            if now >= self._lease_deadline_ns:
                self._lose_endpoint(now, "lease_expired")
            elif self._renew_failed and now >= self._lease_deadline_ns - 500_000_000:
                self._lose_endpoint(now, "renewal_lost")
        if (self._state == "subscribing" and
                now - self._subscribe_started_ns > 3_000_000_000):
            self._lose_endpoint(now, "subscribe_timeout")
        if self._offer is None and now >= self._next_discover_ns:
            self._begin_discovery(now)

        for _ in range(32):
            try:
                data, source = self.socket.recvfrom(ACK.size + 1)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                self.counters["receive_errors"] += 1
                break
            received_ns = self._now(now_ns)
            if (len(source) != 2 or source[1] != self.port or
                    (self.target_ip and source[0] != self.target_ip)):
                self.counters["wrong_source"] += 1
                continue
            try:
                ipaddress.IPv4Address(source[0])
            except ValueError:
                self.counters["wrong_source"] += 1
                continue
            if data[:4] == b"SCA1":
                self._handle_offer(data, source, received_ns)
            elif data[:4] == b"SCK1":
                self._handle_ack(data, source, received_ns)
            else:
                self.counters["unknown_datagrams"] += 1

        now = self._now(now_ns)
        if self._collect_until_ns is not None and now >= self._collect_until_ns:
            self._discover_nonce = self._collect_until_ns = None
            if len(self._candidates) > 1 and self._selected_id is None:
                self._ambiguous = True
                for offer in self._candidates.values():
                    if len(self._ambiguous_candidates) >= 64:
                        break
                    self._ambiguous_candidates.add(f"{offer.device:012x}@{offer.ip}")
                self._state, self._reason = "ambiguous", "multiple_devices; set device_id"
            elif len(self._candidates) == 1 and not self._ambiguous:
                self._offer = next(iter(self._candidates.values()))
                self._selected_id = self._offer.device
                prior = self._resume_offer
                if (prior is not None and
                        (prior.device, prior.boot, prior.ip) ==
                        (self._offer.device, self._offer.boot, self._offer.ip)):
                    self._session = self._resume_session
                    self.counters["session_reused"] += 1
                else:
                    self._session = self._nonce()
                self._resume_offer, self._resume_session = self._offer, self._session
                self._state, self._reason = "subscribing", "waiting_for_subscription_ack"
                self._subscribe_started_ns = now
                self._next_control_ns = now
            elif not self._ambiguous:
                self._reason = "no_offer; check same AP/broadcast isolation or use target_ip"
        if self._offer is not None and self._pending is None and now >= self._next_control_ns:
            self._send_control(now)
        return self.status(now)

    def status(self, now_ns=None):
        now = self._now(now_ns)
        active = not self._closed and self._state == "active" and now < self._lease_deadline_ns
        reason, state = self._reason, self._state
        if not self._closed and self._state == "active" and not active:
            reason, state = "lease_expired", "discovering"
        offer = self._offer or self._last_offer
        candidates = set(self._ambiguous_candidates)
        candidates.update(f"{value.device:012x}@{value.ip}" for value in self._candidates.values())
        return DiscoveryStatus(active, state, reason, None if offer is None else offer.ip,
                               None if offer is None else f"{offer.boot:016x}",
                               None if self._selected_id is None else f"{self._selected_id:012x}",
                               self._session, None if offer is None else offer.sync_port,
                               max(0, (self._lease_deadline_ns - now) // 1_000_000) if active else 0,
                               tuple(sorted(candidates)), 0 if offer is None else offer.capabilities)

    def close(self):
        if self._closed:
            return
        offer = self._offer or self._resume_offer
        session = self._session if self._session is not None else self._resume_session
        if offer is not None and session is not None:
            release = SUBSCRIBE.pack(b"SCE1", self._nonce(), offer.boot, session, 0)
            if self._send(release, offer.ip):
                self.counters["release_sent"] += 1
        self._closed = True
        self._lease_deadline_ns = 0
        self._pending = None
        self._state, self._reason = "closed", "closed"
        self.socket.close()
