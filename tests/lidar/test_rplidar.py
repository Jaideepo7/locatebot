"""Tests for the RPLIDAR driver. Synthetic byte streams only: these prove the parser and
assembler match the published protocol summary, not that a real C1 behaves this way."""
from __future__ import annotations

import importlib.util
import math
import struct
from pathlib import Path

import numpy as np
import pytest

from locatebot.lidar.rplidar import (
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
    lidar = RPLidar("/dev/null", transport=fake)
    with pytest.raises(TimeoutError):
        next(lidar.iter_scans())


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
