"""Synthetic clocks and fake nonblocking socket tests; no ROS or hardware."""
from collections import deque
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from clock_sync import ClockMapper, ClockNotReadyError, SyncClient, REQUEST, RESPONSE

BOOT = "0123456789abcdef"
OTHER_BOOT = "1123456789abcdef"
BASE_NS = 10_000_000_000


def exchange(dev_us, scale=1000.2, outbound_ns=1_000_000, inbound_ns=1_000_000):
    t2, t3 = dev_us, dev_us + 100
    t1 = round(BASE_NS + scale * t2) - outbound_ns
    t4 = round(BASE_NS + scale * t3) + inbound_ns
    return t1, t2, t3, t4


def lock(mapper, boot=BOOT, scale=1000.2, outbound_ns=1_000_000, inbound_ns=1_000_000):
    last = None
    for index in range(8):
        last = exchange(1_000_000 + index * 500_000, scale, outbound_ns, inbound_ns)
        mapper.add_exchange(boot, *last)
    return last[-1]


class ClockMapperTests(unittest.TestCase):
    def test_drift_fit_mapping_and_diagnostics(self):
        mapper = ClockMapper()
        now = lock(mapper)
        status = mapper.status(now)
        self.assertTrue(status.locked, status)
        self.assertAlmostEqual(status.scale_ns_per_us, 1000.2, places=7)
        self.assertAlmostEqual(status.drift_ppm, 200, places=5)
        target = 4_600_000
        self.assertLess(abs(mapper.to_host_monotonic_ns(target, BOOT, now) -
                            round(BASE_NS + 1000.2 * target)), 10)
        self.assertLess(status.residual_ns, 1)
        self.assertGreaterEqual(status.uncertainty_ns, status.rtt_ns // 2)

    def test_requires_five_samples_and_span(self):
        mapper = ClockMapper()
        for index in range(4):
            last = exchange(1_000_000 + index * 500_000)
            mapper.add_exchange(BOOT, *last)
        self.assertFalse(mapper.status(last[-1]).locked)
        with self.assertRaises(ClockNotReadyError):
            mapper.to_host_monotonic_ns(1_000_000, BOOT, last[-1])
        mapper = ClockMapper()
        for index in range(5):
            last = exchange(1_000_000 + index * 1_000)
            mapper.add_exchange(BOOT, *last)
        self.assertFalse(mapper.status(last[-1]).locked)
        self.assertEqual(mapper.status(last[-1]).reason, "insufficient_time_span")

    def test_asymmetric_delay_is_not_claimed_exact(self):
        mapper = ClockMapper()
        now = lock(mapper, scale=1000.0, outbound_ns=1_000_000, inbound_ns=9_000_000)
        result = mapper.to_host_monotonic_ns(4_600_000, BOOT, now)
        self.assertAlmostEqual(result - (BASE_NS + 1000 * 4_600_000), 4_000_000, delta=2)
        self.assertTrue(mapper.status(now).locked)

    def test_expiry_and_boot_relock_without_old_boot_oscillation(self):
        mapper = ClockMapper()
        now = lock(mapper)
        self.assertTrue(mapper.status(now + 5_000_000_000).locked)
        self.assertEqual(mapper.status(now + 5_000_000_001).reason, "expired")
        with self.assertRaises(ClockNotReadyError):
            mapper.to_host_monotonic_ns(5_000_000, BOOT, now + 5_000_000_001)
        new_exchange = (now + 1_000_000_000, 1000, 1100, now + 1_002_100_000)
        self.assertTrue(mapper.add_exchange(OTHER_BOOT, *new_exchange))
        self.assertFalse(mapper.status(new_exchange[-1]).locked)
        self.assertEqual(mapper.status(new_exchange[-1]).boot_id, OTHER_BOOT)
        self.assertFalse(mapper.add_exchange(BOOT, *exchange(9_000_000)))
        self.assertEqual(mapper.boot_id, OTHER_BOOT)
        with self.assertRaises(ClockNotReadyError):
            mapper.to_host_monotonic_ns(1000, BOOT, new_exchange[-1])

    def test_negative_rtt_wide_rtt_jumps_and_old_samples_rejected(self):
        mapper = ClockMapper()
        now = lock(mapper)
        before = mapper.accepted
        self.assertFalse(mapper.add_exchange(BOOT, 100, 10, 100, 200))
        self.assertFalse(mapper.add_exchange(BOOT, *exchange(5_000_000, inbound_ns=200_000_000)))
        t1, t2, t3, t4 = exchange(5_000_000)
        self.assertFalse(mapper.add_exchange(BOOT, t1, t2 + 100_000, t3 + 100_000, t4))
        self.assertFalse(mapper.add_exchange(BOOT, *exchange(1_000_000)))
        self.assertEqual(mapper.accepted, before)
        self.assertTrue(mapper.status(now).locked)

    def test_low_rtt_filter_and_extreme_drift(self):
        mapper = ClockMapper()
        now = lock(mapper)
        # A wider-delay observation remains outside the low-RTT model.
        delayed = exchange(5_000_000, outbound_ns=20_000_000, inbound_ns=20_000_000)
        self.assertTrue(mapper.add_exchange(BOOT, *delayed))
        self.assertAlmostEqual(mapper.status(delayed[-1]).drift_ppm, 200, places=5)
        mapper = ClockMapper()
        now = lock(mapper, scale=1010.0)
        self.assertFalse(mapper.status(now).locked)
        self.assertEqual(mapper.status(now).reason, "extreme_drift")

    def test_startup_outlier_recovers_and_window_is_bounded(self):
        mapper = ClockMapper(max_samples=12)
        for index in range(30):
            t1, t2, t3, t4 = exchange(1_000_000 + index * 500_000)
            if index == 2:
                t2 += 50_000
                t3 += 50_000
            mapper.add_exchange(BOOT, t1, t2, t3, t4)
        status = mapper.status(t4)
        self.assertTrue(status.locked, status)
        self.assertAlmostEqual(status.drift_ppm, 200, places=5)
        self.assertLessEqual(len(mapper._samples), 12)


class FakeSocket:
    def __init__(self):
        self.received = deque()
        self.sent = []
        self.closed = False
        self.fail_send = self.fail_receive = False

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


class SyncClientTests(unittest.TestCase):
    def test_nonblocking_source_nonce_echo_and_response(self):
        sock = FakeSocket()
        client = SyncClient("127.0.0.1", sock=sock)
        t1 = 10_000_000_000
        client.poll(t1)
        data, peer = sock.sent[-1]
        self.assertEqual(peer, ("127.0.0.1", 8889))
        self.assertEqual(len(data), 20)
        magic, nonce, echo = REQUEST.unpack(data)
        self.assertEqual((magic, echo), (b"SCQ1", t1))
        response = RESPONSE.pack(b"SCR1", nonce, t1, 1000, 1100, int(BOOT, 16))
        sock.received.extend([
            (response, (peer[0], 8888)),
            (RESPONSE.pack(b"SCR1", nonce ^ 1, t1, 1000, 1100, int(BOOT, 16)), peer),
            (RESPONSE.pack(b"SCR1", nonce, t1 + 1, 1000, 1100, int(BOOT, 16)), peer),
            (response + b"x", peer),
            (response, peer),
        ])
        client.poll(t1 + 2_100_000)
        for name in ("unexpected_source", "unknown_nonce", "bad_echo", "invalid_response", "accepted"):
            self.assertEqual(client.counters[name], 1, name)
        self.assertFalse(sock.blocking)
        self.assertEqual(client.mapper.boot_id, BOOT)
        client.close()
        self.assertTrue(sock.closed)

    def test_expired_request_and_transient_socket_errors(self):
        sock = FakeSocket()
        client = SyncClient("127.0.0.1", sock=sock)
        client.poll(10_000_000_000)
        client.poll(10_600_000_000)
        self.assertEqual(client.counters["timeout"], 1)
        self.assertEqual(len(client.pending), 1)
        sock.fail_send = sock.fail_receive = True
        client.poll(11_200_000_000)
        self.assertEqual(client.counters["send_errors"], 1)
        self.assertEqual(client.counters["recv_errors"], 1)
        client.close()
        with self.assertRaises(RuntimeError):
            client.poll(12_000_000_000)


if __name__ == "__main__":
    unittest.main()
