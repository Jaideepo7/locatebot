"""Tests for the .lbscan scan log format."""
from __future__ import annotations

import numpy as np
import pytest

from locatebot.lidar.scanlog import ScanLogWriter, read_scan_log
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, Scan


def make_scans(k=3):
    room = Room.box(5.0, 3.0)
    rng = np.random.default_rng(2)
    return [
        simulate_scan(room, Pose2D(1.0 + 0.1 * i, 1.0, 0.2 * i), n_beams=300, rng=rng, stamp_s=10.0 + 0.1 * i,
                      seq=i, dropout=0.1, range_max_m=4.0)
        for i in range(k)
    ]


def write_log(path, scans):
    with ScanLogWriter(path) as w:
        for s in scans:
            w.write(s)


def test_round_trip(tmp_path):
    p = tmp_path / "a.lbscan"
    scans = make_scans()
    write_log(p, scans)
    assert p.read_bytes()[:8] == b"LBSCAN1\0"
    back = list(read_scan_log(p))
    assert len(back) == len(scans)
    for a, b in zip(scans, back):
        assert isinstance(b, Scan)
        assert b.stamp_s == a.stamp_s and b.seq == a.seq
        np.testing.assert_allclose(b.angles_rad, a.angles_rad, rtol=1e-6, atol=1e-6)
        np.testing.assert_array_equal(np.isnan(b.ranges_m), np.isnan(a.ranges_m))
        assert np.isnan(a.ranges_m).any()
        np.testing.assert_allclose(b.ranges_m, a.ranges_m, rtol=1e-6, atol=1e-6, equal_nan=True)
        np.testing.assert_array_equal(b.quality, a.quality)


def test_empty_scan_round_trip(tmp_path):
    p = tmp_path / "e.lbscan"
    write_log(p, [Scan(1.0, 5, [], [], [])])
    (s,) = read_scan_log(p)
    assert len(s) == 0 and s.seq == 5


@pytest.mark.parametrize("cut", [1, 10, 25, 500])
def test_truncated_tail_tolerated(tmp_path, cut):
    p = tmp_path / "t.lbscan"
    write_log(p, make_scans(3))
    data = p.read_bytes()
    p.write_bytes(data[:-cut])
    back = list(read_scan_log(p))
    assert len(back) == 2


def test_crc_corruption_detected(tmp_path):
    p = tmp_path / "c.lbscan"
    write_log(p, make_scans(2))
    data = bytearray(p.read_bytes())
    data[8 + 20 + 100] ^= 0xFF  # inside the first record's payload
    p.write_bytes(bytes(data))
    with pytest.raises(ValueError):
        list(read_scan_log(p))


def test_bad_magic(tmp_path):
    p = tmp_path / "m.lbscan"
    p.write_bytes(b"NOTSCAN\0" + b"\0" * 30)
    with pytest.raises(ValueError):
        list(read_scan_log(p))


def test_flush_per_record(tmp_path):
    p = tmp_path / "f.lbscan"
    scans = make_scans(2)
    with ScanLogWriter(p) as w:
        w.write(scans[0])
        assert len(list(read_scan_log(p))) == 1
        w.write(scans[1])
        assert len(list(read_scan_log(p))) == 2
