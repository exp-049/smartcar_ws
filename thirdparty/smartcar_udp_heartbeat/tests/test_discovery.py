"""Discovery/lease state-machine tests and an isolated localhost wire exchange."""
from collections import deque
from pathlib import Path
import socket
import select
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from discovery import ACK, DISCOVER, OFFER, SUBSCRIBE, DiscoveryClient
from udp_receiver import drain_pending_data, lease_allows_packet

IP = "192.0.2.10"
DEVICE = 0xAABBCCDDEEFF
OTHER = 0x112233445566
BOOT = 0x0123456789ABCDEF
START = 1_000_000_000


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.received = deque()
        self.closed = False
        self.fail_send = self.fail_receive = False

    def setsockopt(self, *args):
        pass

    def setblocking(self, blocking):
        self.blocking = blocking

    def sendto(self, data, peer):
        if self.fail_send:
            raise OSError("network unavailable")
        self.sent.append((data, peer))
        return len(data)

    def recvfrom(self, maximum):
        if self.fail_receive:
            raise OSError("network unavailable")
        if not self.received:
            raise BlockingIOError()
        data, peer = self.received.popleft()
        return data[:maximum], peer

    def close(self):
        self.closed = True


def offer(sock, nonce, *, device=DEVICE, boot=BOOT, ip=IP, port=8890, extra=b""):
    sock.received.append((OFFER.pack(b"SCA1", nonce, device, boot, 8889, 3) + extra, (ip, port)))


def acknowledge(sock, *, result=0, boot=BOOT, device=DEVICE, lease=6000,
                nonce=None, session=None, ip=IP):
    magic, sent_nonce, _, sent_session, data_port = SUBSCRIBE.unpack(sock.sent[-1][0])
    assert magic == b"SCS1"
    sock.received.append((ACK.pack(b"SCK1", sent_nonce if nonce is None else nonce,
                                   device, boot, sent_session if session is None else session,
                                   lease, result), (ip, 8890)))


def subscribe(client, sock, *, start=START, boot=BOOT, ip=IP):
    client.poll(start)
    _, nonce = DISCOVER.unpack(sock.sent[-1][0])
    offer(sock, nonce, boot=boot, ip=ip)
    client.poll(start + 100_000_000)
    client.poll(start + 700_000_000)
    acknowledge(sock, boot=boot, ip=ip)
    return client.poll(start + 710_000_000)


class DiscoveryTests(unittest.TestCase):
    def test_endpoint_queue_drain_is_bounded_and_can_finish_next_turn(self):
        sock = FakeSocket()
        sock.received.extend((b"old data", (IP, 40000)) for _ in range(300))
        self.assertEqual(drain_pending_data(sock), 256)
        self.assertEqual(len(sock.received), 44)
        self.assertEqual(drain_pending_data(sock), 44)
        self.assertEqual(drain_pending_data(sock), 0)

    def test_receiver_requires_current_active_ip_and_boot_for_ack_and_data(self):
        status = SimpleNamespace(active=True, source_ip=IP, boot_id=f"{BOOT:016x}")
        self.assertTrue(lease_allows_packet(status, (IP, 40000), status.boot_id))
        self.assertFalse(lease_allows_packet(status, (IP, 40000), f"{BOOT+1:016x}"))
        self.assertFalse(lease_allows_packet(status, ("192.0.2.99", 40000), status.boot_id))
        status.active = False
        self.assertFalse(lease_allows_packet(status, (IP, 40000), status.boot_id))

    def test_exact_wire_sizes_and_unique_subscription(self):
        self.assertEqual((DISCOVER.size, OFFER.size, SUBSCRIBE.size, ACK.size), (12, 32, 30, 40))
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        status = subscribe(client, sock)
        self.assertTrue(status.active)
        self.assertEqual((status.device_id, status.boot_id, status.source_ip),
                         ("aabbccddeeff", "0123456789abcdef", IP))
        self.assertEqual(status.sync_port, 8889)
        self.assertEqual(status.lease_remaining_ms, 5990)
        self.assertFalse(sock.blocking)
        self.assertNotEqual(status.session_id, 0)
        client.close()
        self.assertEqual(SUBSCRIBE.unpack(sock.sent[-1][0])[0::4], (b"SCE1", 0))
        self.assertTrue(sock.closed)
        client.close()

    def test_multiple_candidates_are_latched_ambiguous(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        client.poll(START)
        nonce = DISCOVER.unpack(sock.sent[-1][0])[1]
        offer(sock, nonce)
        offer(sock, nonce, device=OTHER, ip="192.0.2.11")
        client.poll(START + 100_000_000)
        status = client.poll(START + 700_000_000)
        self.assertEqual(status.state, "ambiguous")
        self.assertEqual(len(status.candidates), 2)
        self.assertFalse(status.active)
        self.assertFalse(any(data.startswith(b"SCS1") for data, _ in sock.sent))
        # One dropped offer in a later round must not accidentally choose a car.
        client.poll(START + 1_000_000_000)
        offer(sock, DISCOVER.unpack(sock.sent[-1][0])[1])
        client.poll(START + 1_100_000_000)
        self.assertEqual(client.poll(START + 1_700_000_000).state, "ambiguous")

    def test_device_filter_and_manual_ip_still_subscribe(self):
        sock = FakeSocket()
        client = DiscoveryClient(device_id="AABBCCDDEEFF", target_ip=IP, sock=sock)
        client.poll(START)
        self.assertEqual(sock.sent[-1][1], (IP, 8890))
        nonce = DISCOVER.unpack(sock.sent[-1][0])[1]
        offer(sock, nonce, device=OTHER)
        offer(sock, nonce, ip="192.0.2.12")
        offer(sock, nonce)
        client.poll(START + 100_000_000)
        self.assertFalse(client.status(START + 100_000_000).active)
        client.poll(START + 700_000_000)
        self.assertEqual(sock.sent[-1][0][:4], b"SCS1")
        acknowledge(sock)
        self.assertTrue(client.poll(START + 710_000_000).active)
        self.assertEqual(client.counters["device_id_filtered"], 1)
        self.assertEqual(client.counters["wrong_source"], 1)

    def test_wrong_nonce_source_truncation_and_sync_magic_rejected(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        client.poll(START)
        nonce = DISCOVER.unpack(sock.sent[-1][0])[1]
        offer(sock, nonce ^ 1)
        offer(sock, nonce, port=8889)
        offer(sock, nonce, extra=b"x")
        sock.received.append((struct.pack("<4sQQ", b"SCQ1", nonce, 1), (IP, 8890)))
        client.poll(START + 100_000_000)
        status = client.poll(START + 700_000_000)
        self.assertFalse(status.active)
        self.assertFalse(status.candidates)
        self.assertEqual(client.counters["wrong_offer_nonce"], 1)
        self.assertEqual(client.counters["wrong_source"], 1)
        self.assertEqual(client.counters["invalid_or_late_offer"], 1)
        self.assertEqual(client.counters["unknown_datagrams"], 1)

    def test_lease_expiry_is_immediate_without_poll(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        subscribe(client, sock)
        deadline = START + 700_000_000 + 6_000_000_000
        self.assertTrue(client.status(deadline - 1).active)
        self.assertFalse(client.status(deadline).active)
        self.assertEqual(client.status(deadline).reason, "lease_expired")

    def test_close_during_rediscovery_releases_cached_owner_session(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        original = subscribe(client, sock)
        client.poll(START + 7_000_000_000)
        self.assertFalse(client.status(START + 7_000_000_000).active)
        self.assertEqual(sock.sent[-1][0][:4], b"SCD1")
        client.close()
        magic, _, boot, session, port = SUBSCRIBE.unpack(sock.sent[-1][0])
        self.assertEqual((magic, boot, session, port), (b"SCE1", BOOT, original.session_id, 0))

    def test_fresh_renew_ack_extends_lease_old_ack_does_not(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        first = subscribe(client, sock)
        old_ack = ACK.pack(b"SCK1", SUBSCRIBE.unpack(sock.sent[-1][0])[1], DEVICE,
                           BOOT, first.session_id, 6000, 0)
        renew_at = START + 2_710_000_000
        client.poll(renew_at)
        _, nonce, _, session, _ = SUBSCRIBE.unpack(sock.sent[-1][0])
        self.assertEqual(session, first.session_id)
        sock.received.append((old_ack, (IP, 8890)))
        client.poll(renew_at + 10_000_000)
        self.assertFalse(client.status(START + 6_700_000_000).active)
        acknowledge(sock, nonce=nonce)
        status = client.poll(renew_at + 20_000_000)
        self.assertTrue(status.active)
        self.assertTrue(client.status(START + 6_700_000_000).active)
        self.assertEqual(client.counters["ack_identity_mismatch"], 1)

    def test_restart_wrong_boot_and_ip_change_keep_device_id(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        original = subscribe(client, sock)
        renew_at = START + 2_710_000_000
        client.poll(renew_at)
        acknowledge(sock, result=2, boot=BOOT + 1, lease=0)
        self.assertFalse(client.poll(renew_at + 10_000_000).active)
        self.assertEqual(client.status(renew_at + 10_000_000).device_id, original.device_id)
        new_start = renew_at + 1_010_000_000
        client.poll(new_start)
        nonce = DISCOVER.unpack(sock.sent[-1][0])[1]
        offer(sock, nonce, device=OTHER)
        offer(sock, nonce, boot=BOOT + 1, ip="192.0.2.20")
        client.poll(new_start + 100_000_000)
        client.poll(new_start + 700_000_000)
        acknowledge(sock, boot=BOOT + 1, ip="192.0.2.20")
        status = client.poll(new_start + 710_000_000)
        self.assertTrue(status.active)
        self.assertEqual(status.source_ip, "192.0.2.20")
        self.assertEqual(status.boot_id, f"{BOOT+1:016x}")
        self.assertNotEqual(status.session_id, original.session_id)

    def test_lost_initial_ack_rediscovery_renews_own_existing_device_lease(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        client.poll(START)
        offer(sock, DISCOVER.unpack(sock.sent[-1][0])[1])
        client.poll(START + 100_000_000)
        client.poll(START + 700_000_000)
        owner_session = SUBSCRIBE.unpack(sock.sent[-1][0])[3]
        # The device accepts this subscription, but every ACK is lost. Retries
        # can extend its live lease even though the PC never became active.
        client.poll(START + 1_710_000_000)
        client.poll(START + 2_720_000_000)
        client.poll(START + 3_710_000_000)
        self.assertEqual(sock.sent[-1][0][:4], b"SCD1")
        offer(sock, DISCOVER.unpack(sock.sent[-1][0])[1])
        client.poll(START + 3_810_000_000)
        client.poll(START + 4_410_000_000)
        new_request = SUBSCRIBE.unpack(sock.sent[-1][0])
        self.assertEqual(new_request[3], owner_session)
        # Model the server ownership rule: another session would return BUSY.
        result = 0 if new_request[3] == owner_session else 1
        acknowledge(sock, result=result)
        self.assertTrue(client.poll(START + 4_420_000_000).active)
        self.assertEqual(client.counters["session_reused"], 1)

    def test_lost_renew_acks_reuses_session_and_ip_change_uses_new_session(self):
        for new_ip in (IP, "192.0.2.30"):
            with self.subTest(new_ip=new_ip):
                sock = FakeSocket()
                client = DiscoveryClient(sock=sock)
                original = subscribe(client, sock)
                client.poll(START + 2_710_000_000)
                client.poll(START + 3_720_000_000)
                client.poll(START + 4_720_000_000)
                # The device accepted both renews, extending its lease, but the
                # PC lost ACKs and conservatively starts rediscovery early.
                client.poll(START + 6_210_000_000)
                self.assertFalse(client.status(START + 6_210_000_000).active)
                self.assertEqual(sock.sent[-1][0][:4], b"SCD1")
                offer(sock, DISCOVER.unpack(sock.sent[-1][0])[1], ip=new_ip)
                client.poll(START + 6_310_000_000)
                client.poll(START + 6_910_000_000)
                session = SUBSCRIBE.unpack(sock.sent[-1][0])[3]
                if new_ip == IP:
                    self.assertEqual(session, original.session_id)
                else:
                    self.assertNotEqual(session, original.session_id)
                acknowledge(sock, ip=new_ip)
                self.assertTrue(client.poll(START + 6_920_000_000).active)

    def test_busy_wrong_session_invalid_lease_and_socket_failure(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        client.poll(START)
        offer(sock, DISCOVER.unpack(sock.sent[-1][0])[1])
        client.poll(START + 100_000_000)
        client.poll(START + 700_000_000)
        acknowledge(sock, session=1)
        client.poll(START + 710_000_000)
        acknowledge(sock, lease=65535)
        client.poll(START + 720_000_000)
        self.assertFalse(client.status(START + 720_000_000).active)
        acknowledge(sock, result=1, lease=0)
        status = client.poll(START + 730_000_000)
        self.assertEqual(status.reason, "busy")
        self.assertFalse(status.active)
        sock.fail_send = sock.fail_receive = True
        client.poll(START + 2_000_000_000)
        self.assertEqual(client.counters["send_errors"], 1)
        self.assertEqual(client.counters["receive_errors"], 1)

    def test_receive_budget_is_bounded(self):
        sock = FakeSocket()
        client = DiscoveryClient(sock=sock)
        sock.received.extend((b"invalid", (IP, 8890)) for _ in range(100))
        client.poll(START)
        self.assertEqual(len(sock.received), 68)
        self.assertEqual(client.counters["unknown_datagrams"], 32)


class LocalhostDiscoveryTests(unittest.TestCase):
    def test_real_unicast_subscribe_data_endpoint_and_release(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(server.close)
        self.addCleanup(data_socket.close)
        server.bind(("127.0.0.1", 0))
        data_socket.bind(("127.0.0.1", 0))
        server.settimeout(1)
        data_socket.settimeout(1)
        client = DiscoveryClient(data_port=data_socket.getsockname()[1], target_ip="127.0.0.1",
                                 port=server.getsockname()[1])
        try:
            client.poll(START)
            request, peer = server.recvfrom(100)
            magic, nonce = DISCOVER.unpack(request)
            self.assertEqual(magic, b"SCD1")
            server.sendto(OFFER.pack(b"SCA1", nonce, DEVICE, BOOT, 8889, 3), peer)
            self.assertTrue(select.select([client.socket], [], [], 1)[0], "offer was not delivered")
            client.poll(START + 100_000_000)
            client.poll(START + 700_000_000)
            request, peer = server.recvfrom(100)
            magic, nonce, boot, session, data_port = SUBSCRIBE.unpack(request)
            self.assertEqual((magic, boot, data_port), (b"SCS1", BOOT, data_socket.getsockname()[1]))
            server.sendto(ACK.pack(b"SCK1", nonce, DEVICE, BOOT, session, 6000, 0), peer)
            self.assertTrue(select.select([client.socket], [], [], 1)[0], "subscription ACK was not delivered")
            self.assertTrue(client.poll(START + 710_000_000).active)
            server.sendto(b"test-data", (peer[0], data_port))
            self.assertEqual(data_socket.recvfrom(100)[0], b"test-data")
            client.close()
            request, _ = server.recvfrom(100)
            self.assertEqual(SUBSCRIBE.unpack(request)[0::4], (b"SCE1", 0))
        finally:
            client.close()
            server.close()
            data_socket.close()


if __name__ == "__main__":
    unittest.main()
