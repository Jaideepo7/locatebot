"""Tests for the .lbscan scan log format."""
from __future__ import annotations

import struct
import zlib

import numpy as np
import pytest

from locatebot.lidar.scanlog import MAX_BEAMS, ScanLogWriter, read_scan_log
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
    assert p.read_bytes()[:8] == b"LBSCAN2\0"
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
    data[8 + 24 + 100] ^= 0xFF  # inside the first record's payload
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


REC = 24 + 9 * 300 + 4  # header + header CRC, payload of 300 beams, record CRC


@pytest.mark.parametrize("byte,mask", [(0, 0x01), (3, 0x80)], ids=["low-bit", "high-bit"])
def test_bit_flip_in_record_header_n_detected(tmp_path, byte, mask):
    p = tmp_path / "n.lbscan"
    write_log(p, make_scans(3))
    data = bytearray(p.read_bytes())
    assert len(data) == 8 + 3 * REC
    data[8 + REC + 16 + byte] ^= mask  # n field of the middle record
    p.write_bytes(bytes(data))
    with pytest.raises(ValueError, match="header CRC"):
        list(read_scan_log(p))


def test_n_above_max_beams_rejected_on_read(tmp_path):
    p = tmp_path / "big.lbscan"
    header = struct.pack("<dqI", 1.0, 0, MAX_BEAMS + 1)
    p.write_bytes(b"LBSCAN2\0" + header + struct.pack("<I", zlib.crc32(header)) + b"\0" * 64)
    with pytest.raises(ValueError, match="MAX_BEAMS|n="):
        list(read_scan_log(p))


def test_n_above_max_beams_rejected_on_write(tmp_path):
    p = tmp_path / "w.lbscan"
    n = MAX_BEAMS + 1
    with ScanLogWriter(p) as w:
        with pytest.raises(ValueError):
            w.write(Scan(0.0, 0, np.zeros(n), np.ones(n), np.zeros(n)))
    assert list(read_scan_log(p)) == []


def test_old_lbscan1_magic_rejected_clearly(tmp_path):
    p = tmp_path / "old.lbscan"
    p.write_bytes(b"LBSCAN1\0" + b"\0" * 40)
    with pytest.raises(ValueError, match="LBSCAN1"):
        list(read_scan_log(p))
