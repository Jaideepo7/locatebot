"""Tests for the RPLIDAR driver. Synthetic byte streams only: these prove the parser and
assembler match the published protocol summary, not that a real C1 behaves this way."""
from __future__ import annotations

import importlib.util
import math
import struct
import time
from pathlib import Path

import numpy as np
import pytest

from locatebot.lidar.rplidar import (
    NODE_LEN,
    Node,
    RPLidar,
    ScanAssembler,
    build_request,
    parse_descriptor,
    parse_scan_nodes,
)
from locatebot.lidar.scanlog import read_scan_log
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D

ROOT = Path(__file__).resolve().parents[2]
SCAN_DESCRIPTOR = bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])


# ---------------------------------------------------------------- byte builders
def node_bytes(angle_deg: float, distance_mm: float, quality: int = 47, start: bool = False) -> bytes:
    a = int(round(angle_deg * 64)) % (360 * 64)
    d = int(round(distance_mm * 4))
    s = 1 if start else 0
    b0 = ((quality & 0x3F) << 2) | ((1 - s) << 1) | s
    b1 = ((a & 0x7F) << 1) | 1
    b2 = (a >> 7) & 0xFF
    return bytes([b0, b1, b2]) + struct.pack("<H", d)


def scan_to_bytes(scan, mount_offset_rad: float = 0.0):
    """Encode a robot-frame Scan as one sensor revolution (clockwise angles, start on first node).

    Returns (bytes, robot_angles_in_node_order, ranges_in_node_order)."""
    deg = np.degrees(mount_offset_rad - scan.angles_rad) % 360.0
    order = np.argsort(deg, kind="stable")
    out = bytearray()
    for k, i in enumerate(order):
        r = scan.ranges_m[i]
        mm = 0.0 if not np.isfinite(r) else r * 1000.0
        out += node_bytes(deg[i], mm, int(scan.quality[i]), start=(k == 0))
    return bytes(out), scan.angles_rad[order], scan.ranges_m[order]


def ang_diff(a, b):
    return (np.asarray(a) - np.asarray(b) + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- requests / descriptor
def test_build_request():
    assert build_request(0x25) == b"\xa5\x25"
    assert build_request(0x20) == b"\xa5\x20"


def test_parse_scan_descriptor():
    assert parse_descriptor(SCAN_DESCRIPTOR) == (5, 1, 0x81)


def test_parse_descriptor_bad_sync_raises():
    with pytest.raises(ValueError):
        parse_descriptor(bytes([0xA5, 0x00, 0x05, 0x00, 0x00, 0x40, 0x81]))
    with pytest.raises(ValueError):
        parse_descriptor(SCAN_DESCRIPTOR[:5])


# ---------------------------------------------------------------- node parsing
def test_parse_node_decoding():
    nodes, used = parse_scan_nodes(node_bytes(123.5, 1234.25, quality=15, start=True))
    assert used == 5
    assert len(nodes) == 1
    n = nodes[0]
    assert isinstance(n, Node)
    assert n.angle_deg == pytest.approx(123.5)
    assert n.distance_mm == pytest.approx(1234.25)
    assert n.quality == 15
    assert n.start is True
    # hand-built bytes: quality 63, no start, angle_q6 = 0x1234, distance_q2 = 0x0102
    raw = bytes([(63 << 2) | 0b10, ((0x1234 & 0x7F) << 1) | 1, 0x1234 >> 7, 0x02, 0x01])
    (m,), used = parse_scan_nodes(raw)
    assert used == 5
    assert m.angle_deg == pytest.approx(0x1234 / 64)
    assert m.distance_mm == pytest.approx(0x0102 / 4)
    assert (m.quality, m.start) == (63, False)


def test_parse_skips_bad_nodes_and_resyncs():
    good1 = node_bytes(10.0, 500.0)
    bad_check = bytearray(node_bytes(20.0, 600.0))
    bad_check[1] &= 0xFE  # check bit cleared
    bad_s = bytearray(node_bytes(30.0, 700.0))
    bad_s[0] |= 0b11  # S and !S both set
    good2 = node_bytes(40.0, 800.0)
    stream = good1 + b"\x00\x00" + bytes(bad_check) + bytes(bad_s) + good2
    nodes, used = parse_scan_nodes(stream)
    angles = [n.angle_deg for n in nodes]
    assert 10.0 in angles and 40.0 in angles
    assert 20.0 not in angles and 30.0 not in angles
    assert nodes[-1].angle_deg == pytest.approx(40.0)
    assert used == len(stream)


def test_parse_leaves_partial_tail_unconsumed():
    stream = node_bytes(10.0, 500.0) + node_bytes(11.0, 500.0)[:3]
    nodes, used = parse_scan_nodes(bytearray(stream))
    assert len(nodes) == 1 and used == 5


# ---------------------------------------------------------------- assembler
def rev(n, start_deg=0.0, dist=1000.0):
    return [Node(start_deg + 360.0 * i / n, dist, 47, i == 0) for i in range(n)]


def test_assembler_discards_partial_first_revolution_and_splits_on_start():
    asm = ScanAssembler(min_points=5, clock=lambda: 7.0)
    partial = [Node(200.0 + i, 1000.0, 47, False) for i in range(20)]
    assert asm.feed(partial) == []
    assert asm.feed(rev(10)) == []  # first start node: nothing closed yet
    out = asm.feed(rev(12))
    assert len(out) == 1
    assert len(out[0]) == 10
    assert out[0].seq == 0 and out[0].stamp_s == 7.0
    out = asm.feed(rev(8)[:1])
    assert len(out) == 1 and len(out[0]) == 12 and out[0].seq == 1


def test_assembler_multiple_revolutions_in_one_feed():
    asm = ScanAssembler(min_points=5)
    out = asm.feed(rev(10) + rev(11) + rev(12) + rev(6)[:1])
    assert [len(s) for s in out] == [10, 11, 12]
    assert [s.seq for s in out] == [0, 1, 2]


def test_assembler_clockwise_to_ccw_and_mount_offset():
    nodes = [Node(90.0, 1000.0, 47, True), Node(180.0, 2000.0, 47, False), Node(270.0, 3000.0, 47, False)]
    asm = ScanAssembler(min_points=3)
    (s,) = asm.feed(nodes + nodes[:1])
    # sensor clockwise 90 deg == robot right == -pi/2
    assert ang_diff(s.angles_rad, [-math.pi / 2, -math.pi, math.pi / 2]) == pytest.approx([0, 0, 0], abs=1e-12)
    np.testing.assert_allclose(s.ranges_m, [1.0, 2.0, 3.0])
    asm = ScanAssembler(min_points=3, mount_offset_rad=math.pi / 2)
    (s,) = asm.feed(nodes + nodes[:1])
    assert ang_diff(s.angles_rad, [0.0, -math.pi / 2, math.pi]) == pytest.approx([0, 0, 0], abs=1e-12)
    assert np.all(s.angles_rad >= -math.pi) and np.all(s.angles_rad < math.pi)


def test_assembler_distance_zero_is_nan():
    nodes = [Node(0.0, 0.0, 0, True), Node(120.0, 1500.0, 47, False), Node(240.0, 0.0, 0, False)]
    asm = ScanAssembler(min_points=3)
    (s,) = asm.feed(nodes + nodes[:1])
    assert np.isnan(s.ranges_m[0]) and np.isnan(s.ranges_m[2])
    assert s.ranges_m[1] == pytest.approx(1.5)
    assert list(s.quality) == [0, 47, 0]


def test_assembler_min_points_respected():
    asm = ScanAssembler(min_points=10)
    out = asm.feed(rev(10) + rev(9) + rev(10) + rev(10)[:1])
    assert [len(s) for s in out] == [10, 10]
    assert [s.seq for s in out] == [0, 1]


# ---------------------------------------------------------------- RPLidar with fake transport
class FakeTransport:
    """Scripted serial port: each request queues its scripted response; reset clears rx."""

    def __init__(self, responses: dict[int, bytes]):
        self.responses = responses
        self.rx = bytearray()
        self.writes: list[bytes] = []
        self.closed = False
        self.resets = 0

    def write(self, b):
        b = bytes(b)
        self.writes.append(b)
        if len(b) >= 2 and b[0] == 0xA5:
            self.rx += self.responses.get(b[1], b"")
        return len(b)

    def read(self, n):
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def reset_input_buffer(self):
        self.resets += 1
        self.rx.clear()

    def close(self):
        self.closed = True


def sim_stream(n_revs=3, mount_offset_rad=0.0, garbage=b""):
    room = Room.box(6.0, 4.0).with_box(2.0, 1.0, 2.5, 1.5)
    rng = np.random.default_rng(1)
    expected = []
    stream = bytearray()
    for k in range(n_revs):
        scan = simulate_scan(room, Pose2D(1.0, 2.0, 0.3 * k), n_beams=400, rng=rng, dropout=0.05, range_max_m=5.0)
        b, a, r = scan_to_bytes(scan, mount_offset_rad)
        expected.append((a, r))
        stream += b + garbage
    return bytes(stream), expected


def test_rplidar_iter_scans_matches_simulation():
    offset = math.radians(30.0)
    revs, expected = sim_stream(4, offset, garbage=b"\x00\x00\x00")
    partial = revs[1000:1075]  # 15 mid-revolution nodes, no start node: must be discarded
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR + partial + revs})
    with RPLidar("/dev/null", transport=fake, mount_offset_rad=offset) as lidar:
        scans = list(lidar.iter_scans(max_scans=3))
    assert fake.closed
    assert [s.seq for s in scans] == [0, 1, 2]
    for s, (a, r) in zip(scans, expected):
        assert len(s) == len(a) == 400
        np.testing.assert_array_equal(np.isnan(s.ranges_m), np.isnan(r))
        ok = np.isfinite(r)
        assert np.max(np.abs(s.ranges_m[ok] - r[ok])) <= 0.000125 + 1e-9  # q2: 0.25 mm steps
        assert np.max(np.abs(ang_diff(s.angles_rad, a))) <= math.radians(0.5 / 64) + 1e-9  # q6
    assert b"\xa5\x20" in fake.writes


def test_rplidar_stop_writes_a5_25():
    fake = FakeTransport({})
    lidar = RPLidar("/dev/null", transport=fake)
    lidar.stop()
    assert fake.writes[-1] == b"\xa5\x25"
    lidar.close()
    assert fake.closed


def test_rplidar_health_and_info():
    health = bytes([0xA5, 0x5A, 0x03, 0, 0, 0, 0x06, 0x01, 0x34, 0x12])
    info_payload = bytes([65, 1, 1, 18]) + bytes(range(16))
    info = bytes([0xA5, 0x5A, 0x14, 0, 0, 0, 0x04]) + info_payload
    fake = FakeTransport({0x52: health, 0x50: info})
    lidar = RPLidar("/dev/null", transport=fake)
    assert lidar.get_health() == (1, 0x1234)
    d = lidar.get_info()
    assert d["model"] == 65 and d["firmware"] == (1, 1) and d["hardware"] == 18
    assert d["serial"] == bytes(range(16)).hex().upper()


def test_rplidar_start_scan_rejects_wrong_descriptor():
    fake = FakeTransport({0x20: bytes([0xA5, 0x5A, 0x03, 0, 0, 0, 0x06])})
    lidar = RPLidar("/dev/null", transport=fake)
    with pytest.raises(RuntimeError):
        lidar.start_scan()


def test_rplidar_timeout_when_no_data():
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR})
    lidar = RPLidar("/dev/null", transport=fake, clock=StepClock(0.5), sleep=lambda s: None)
    with pytest.raises(TimeoutError):
        next(lidar.iter_scans())


# ---------------------------------------------------------------- robustness: watchdog, stop, recovery
STOP = b"\xa5\x25"


class StreamingTransport(FakeTransport):
    """Fake port that streams `pattern` forever.

    stale=True models a sensor still streaming from a previous run: every read returns stale
    pattern bytes (ahead of any response) until STOP is written. After SCAN is written the port
    serves the scripted response, then the pattern forever. read() fails loudly after max_reads
    so a missing watchdog shows up as a test failure, not a hang."""

    def __init__(self, responses, pattern, stale=False, max_reads=20000):
        super().__init__(responses)
        self.pattern = bytes(pattern)
        self.stale = stale
        self.streaming = False
        self.reads = 0
        self.max_reads = max_reads
        self._pos = 0

    def write(self, b):
        b = bytes(b)
        if b == STOP:
            self.stale = False
            self.streaming = False
        elif b == b"\xa5\x20":
            self.streaming = True
        return super().write(b)

    def _pattern(self, n):
        out = bytearray()
        while len(out) < n:
            take = min(n - len(out), len(self.pattern) - self._pos)
            out += self.pattern[self._pos:self._pos + take]
            self._pos = (self._pos + take) % len(self.pattern)
        return bytes(out)

    def read(self, n):
        self.reads += 1
        if self.reads > self.max_reads:
            raise AssertionError("driver kept reading forever (no watchdog)")
        if self.stale:
            return self._pattern(n)
        if self.rx:
            return super().read(n)
        return self._pattern(n) if self.streaming else b""


class StepClock:
    def __init__(self, step=0.01):
        self.t, self.step = 0.0, step

    def __call__(self):
        self.t += self.step
        return self.t


NO_START = b"".join(node_bytes(i * 0.9, 1000.0, start=False) for i in range(400))


@pytest.mark.parametrize("prefix,pattern", [
    (b"", NO_START),                                   # valid nodes, start flag never set
    (node_bytes(0.0, 1000.0, start=True), NO_START),   # one start, then never another
    (b"", bytes([0x00, 0xFF, 0x13, 0x02]) * 50),       # garbage that never parses
], ids=["no-start", "one-start", "garbage"])
def test_iter_scans_times_out_when_no_revolution_completes(prefix, pattern):
    fake = StreamingTransport({0x20: SCAN_DESCRIPTOR + prefix}, pattern)
    lidar = RPLidar("/dev/null", transport=fake, clock=StepClock(0.01), sleep=lambda s: None)
    with pytest.raises(TimeoutError):
        next(lidar.iter_scans(scan_timeout_s=0.5))
    assert len(lidar._buf) < NODE_LEN
    assert fake.writes[-1] == STOP


def test_iter_scans_timeout_real_clock_is_quick():
    fake = StreamingTransport({0x20: SCAN_DESCRIPTOR}, NO_START, max_reads=10**7)
    lidar = RPLidar("/dev/null", transport=fake, sleep=lambda s: None)
    t0 = time.monotonic()
    with pytest.raises(TimeoutError):
        next(lidar.iter_scans(scan_timeout_s=0.05))
    assert time.monotonic() - t0 < 2.0


def test_assembler_buffer_bounded_and_waits_for_next_start():
    asm = ScanAssembler(min_points=5, max_nodes=100)
    nodes = [Node(0.0, 1000.0, 47, True)] + [Node(i * 0.1 % 360, 1000.0, 47, False) for i in range(25000)]
    assert asm.feed(nodes) == []
    assert len(asm._angle) <= 100
    # the overflowed revolution is dropped: the next start opens a fresh one instead of closing it
    out = asm.feed(rev(10) + rev(10)[:1])
    assert [len(s) for s in out] == [10]


def test_assembler_stamps_scan_with_first_node_time():
    now = [1.0]
    asm = ScanAssembler(min_points=5, clock=lambda: now[0])
    asm.feed(rev(10))          # start node of revolution 0 read at t=1
    now[0] = 1.1
    (s,) = asm.feed(rev(10))   # closed at t=1.1, which also starts revolution 1
    assert s.stamp_s == 1.0
    now[0] = 1.2
    (s,) = asm.feed(rev(10)[:1])
    assert s.stamp_s == 1.1


def test_context_manager_stops_motor_after_start_scan():
    revs, _ = sim_stream(2)
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR + revs})
    with RPLidar("/dev/null", transport=fake, sleep=lambda s: None) as lidar:
        lidar.start_scan()
        fake.writes.clear()
    assert fake.writes == [STOP]
    assert fake.closed


def test_close_without_scanning_sends_nothing():
    fake = FakeTransport({})
    RPLidar("/dev/null", transport=fake).close()
    assert fake.writes == [] and fake.closed


def test_generator_close_stops_motor():
    revs, _ = sim_stream(4)
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR + revs})
    lidar = RPLidar("/dev/null", transport=fake, sleep=lambda s: None)
    gen = lidar.iter_scans()
    next(gen)
    fake.writes.clear()
    gen.close()
    assert fake.writes == [STOP]


def test_break_out_of_iteration_stops_motor():
    revs, _ = sim_stream(4)
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR + revs})
    lidar = RPLidar("/dev/null", transport=fake, sleep=lambda s: None)
    for _ in lidar.iter_scans():
        fake.writes.clear()
        break
    assert fake.writes == [STOP]


def test_max_scans_reached_stops_motor():
    revs, _ = sim_stream(4)
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR + revs})
    lidar = RPLidar("/dev/null", transport=fake, sleep=lambda s: None)
    assert len(list(lidar.iter_scans(max_scans=2))) == 2
    assert fake.writes[-1] == STOP


def test_exception_inside_iteration_stops_motor():
    revs, _ = sim_stream(4)
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR + revs})
    lidar = RPLidar("/dev/null", transport=fake, sleep=lambda s: None)
    with pytest.raises(RuntimeError, match="boom"):
        for _ in lidar.iter_scans():
            fake.writes.clear()
            raise RuntimeError("boom")
    assert fake.writes == [STOP]


def test_transport_error_inside_iteration_stops_motor():
    fake = FakeTransport({0x20: SCAN_DESCRIPTOR})  # stream goes silent -> TimeoutError
    lidar = RPLidar("/dev/null", transport=fake, sleep=lambda s: None)
    with pytest.raises(TimeoutError):
        next(lidar.iter_scans())
    assert fake.writes[-1] == STOP


def test_commands_recover_sensor_that_is_already_streaming():
    health = bytes([0xA5, 0x5A, 0x03, 0, 0, 0, 0x06, 0x00, 0x00, 0x00])
    revs, _ = sim_stream(3)
    stale = node_bytes(0.0, 900.0, start=True) + NO_START
    sleeps = []
    fake = StreamingTransport({0x52: health, 0x20: SCAN_DESCRIPTOR + revs}, stale, stale=True)
    lidar = RPLidar("/dev/null", transport=fake, sleep=sleeps.append)
    assert lidar.get_health() == (0, 0)
    assert fake.writes[:2] == [STOP, b"\xa5\x52"]
    assert sleeps and sleeps[0] >= 0.001
    fake.stale = True  # crash again while streaming
    lidar.start_scan()
    assert fake.writes[-2:] == [STOP, b"\xa5\x20"]


# ---------------------------------------------------------------- record_scan CLI
def test_record_scan_cli(tmp_path, capsys):
    spec = importlib.util.spec_from_file_location("record_scan", ROOT / "tools" / "record_scan.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    revs, _ = sim_stream(3)
    health = bytes([0xA5, 0x5A, 0x03, 0, 0, 0, 0x06, 0, 0, 0])
    info = bytes([0xA5, 0x5A, 0x14, 0, 0, 0, 0x04, 65, 1, 1, 18]) + bytes(16)
    fakes = []

    def factory(port, baudrate, mount_offset_rad):
        fakes.append(FakeTransport({0x52: health, 0x50: info, 0x20: SCAN_DESCRIPTOR + revs}))
        return RPLidar(port, baudrate=baudrate, transport=fakes[-1], mount_offset_rad=mount_offset_rad)

    out = tmp_path / "x.lbscan"
    rc = mod.main(["--port", "/dev/fake", "--out", str(out), "--seconds", "60"], lidar_factory=factory)
    assert rc == 1  # stream went silent -> timeout reported, file still closed cleanly
    assert len(list(read_scan_log(out))) == 2
    assert fakes[0].writes[-1] == b"\xa5\x25" and fakes[0].closed
    text = capsys.readouterr().out
    assert "health" in text.lower() and "scans" in text.lower()

    out2 = tmp_path / "y.lbscan"
    rc = mod.main(["--port", "/dev/fake", "--out", str(out2), "--seconds", "0"], lidar_factory=factory)
    assert rc == 0
    assert len(list(read_scan_log(out2))) == 1
    assert fakes[1].writes[-1] == b"\xa5\x25" and fakes[1].closed


# ---------------------------------------------------------------- C1 motor spin-up (found on real hardware, 2026-11-08)
class SpinUpTransport(FakeTransport):
    """After SCAN: descriptor at once, then `silent_reads` empty reads (each one a serial read
    timeout of `read_timeout_s` on `clock`), then `stream`, then silence again."""

    def __init__(self, stream, silent_reads, clock, read_timeout_s=1.0):
        super().__init__({0x20: SCAN_DESCRIPTOR})
        self.stream = bytearray(stream)
        self.silent_left = silent_reads
        self.clock, self.read_timeout_s = clock, read_timeout_s
        self.scanning = False

    def write(self, b):
        if bytes(b) == b"\xa5\x20":
            self.scanning = True
        return super().write(b)

    def read(self, n):
        if self.rx:
            return super().read(n)
        if self.scanning and self.silent_left > 0:
            self.silent_left -= 1
            self.clock.t += self.read_timeout_s
            return b""
        if self.stream:
            out = bytes(self.stream[:n]); del self.stream[:n]
            return out
        self.clock.t += self.read_timeout_s
        return b""


class ManualClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_c1_spin_up_silence_is_tolerated():
    # Real C1: descriptor immediately, first node bytes about 2.2 s later (motor spin-up).
    revs, _ = sim_stream(3)
    clock = ManualClock()
    fake = SpinUpTransport(revs, silent_reads=2, clock=clock)  # 2 x 1 s read timeouts
    lidar = RPLidar("/dev/null", transport=fake, clock=clock, sleep=lambda s: None)
    scans = list(lidar.iter_scans(max_scans=2, startup_timeout_s=5.0))
    assert len(scans) == 2
    assert fake.writes[-1] == STOP


def test_sensor_silent_past_startup_grace_times_out():
    clock = ManualClock()
    fake = SpinUpTransport(b"", silent_reads=100, clock=clock)
    lidar = RPLidar("/dev/null", transport=fake, clock=clock, sleep=lambda s: None)
    with pytest.raises(TimeoutError):
        next(lidar.iter_scans(startup_timeout_s=5.0))
    assert clock.t <= 6.0 + 1e-9  # gave up right after the grace, not later
    assert fake.writes[-1] == STOP


def test_stall_after_data_started_still_times_out_at_once():
    revs, _ = sim_stream(2)
    clock = ManualClock()
    fake = SpinUpTransport(revs, silent_reads=0, clock=clock)
    lidar = RPLidar("/dev/null", transport=fake, clock=clock, sleep=lambda s: None)
    it = lidar.iter_scans(startup_timeout_s=5.0)
    next(it)
    with pytest.raises(TimeoutError):
        for _ in it:
            pass
    assert clock.t <= 1.0 + 1e-9  # one silent read after the stream ended, no startup grace again
