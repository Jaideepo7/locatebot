"""Tests for the .lbmap file and ROS PGM/YAML export (synthetic data only)."""
from __future__ import annotations

import os
import struct
import zlib

import numpy as np
import pytest

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.mapfile import MapFileError, export_ros_map, load_map, save_map

HDR = struct.Struct("<HdddIII")


def sample_grid() -> OccupancyGrid:
    g = OccupancyGrid(7, 5, 0.05, origin_xy=(-1.25, 2.5))
    rng = np.random.default_rng(3)
    g.log_odds[:] = rng.uniform(-4, 4, size=(5, 7)).astype(np.float32)
    return g


def test_round_trip_exact(tmp_path):
    g = sample_grid()
    p = tmp_path / "s.lbmap"
    save_map(p, g)
    g2, meta = load_map(p)
    assert meta == {}
    assert g2.log_odds.dtype == np.float32
    assert np.array_equal(g2.log_odds, g.log_odds)
    assert g2.resolution_m == g.resolution_m
    assert g2.origin_xy == g.origin_xy
    assert g2.log_odds.shape == (5, 7)


def test_file_layout(tmp_path):
    g = sample_grid()
    p = tmp_path / "s.lbmap"
    save_map(p, g, {"robot": "lb1"})
    data = p.read_bytes()
    assert data[:8] == b"LBMAP1\0\0"
    version, res, ox, oy, w, h, ml = HDR.unpack_from(data, 8)
    assert (version, res, ox, oy, w, h) == (1, 0.05, -1.25, 2.5, 7, 5)
    off = 8 + HDR.size + ml
    cells = np.frombuffer(data[off : off + w * h * 4], dtype="<f4").reshape(h, w)
    assert np.array_equal(cells, g.log_odds)  # row 0 = lowest y
    assert len(data) == off + w * h * 4 + 4
    assert struct.unpack("<I", data[-4:])[0] == zlib.crc32(data[:-4])


def test_meta_round_trip(tmp_path):
    meta = {"session": "2026-11-07-a", "robot": "locatebot", "created": 1762473600.5, "notes": "kitchen ü", "n": [1, 2]}
    p = tmp_path / "m.lbmap"
    save_map(p, sample_grid(), meta)
    _, meta2 = load_map(p)
    assert meta2 == meta


def test_corrupted_byte_raises(tmp_path):
    p = tmp_path / "c.lbmap"
    save_map(p, sample_grid(), {"a": 1})
    data = bytearray(p.read_bytes())
    for idx in (len(data) // 2, 8 + HDR.size + 2, len(data) - 1):
        bad = bytearray(data)
        bad[idx] ^= 0x01
        q = tmp_path / f"bad{idx}.lbmap"
        q.write_bytes(bytes(bad))
        with pytest.raises(MapFileError):
            load_map(q)


def test_truncated_raises(tmp_path):
    p = tmp_path / "t.lbmap"
    save_map(p, sample_grid())
    data = p.read_bytes()
    for n in (0, 5, 8 + HDR.size - 1, len(data) - 1, len(data) - 30):
        q = tmp_path / f"t{n}.lbmap"
        q.write_bytes(data[:n])
        with pytest.raises(MapFileError):
            load_map(q)


def test_bad_magic_and_version(tmp_path):
    p = tmp_path / "v.lbmap"
    save_map(p, sample_grid())
    data = bytearray(p.read_bytes())
    bad = bytearray(data)
    bad[:8] = b"NOTAMAP\0"
    bad[-4:] = struct.pack("<I", zlib.crc32(bytes(bad[:-4])))
    (tmp_path / "m.lbmap").write_bytes(bytes(bad))
    with pytest.raises(MapFileError, match="magic"):
        load_map(tmp_path / "m.lbmap")
    bad = bytearray(data)
    struct.pack_into("<H", bad, 8, 2)
    bad[-4:] = struct.pack("<I", zlib.crc32(bytes(bad[:-4])))
    (tmp_path / "v2.lbmap").write_bytes(bytes(bad))
    with pytest.raises(MapFileError, match="version"):
        load_map(tmp_path / "v2.lbmap")


def test_map_file_error_is_value_error():
    assert issubclass(MapFileError, ValueError)


def test_save_is_atomic_and_overwrites(tmp_path):
    p = tmp_path / "a.lbmap"
    save_map(p, sample_grid(), {"v": 1})
    g = sample_grid()
    g.log_odds[:] = 1.0
    save_map(str(p), g, {"v": 2})
    g2, meta = load_map(str(p))
    assert meta == {"v": 2} and np.all(g2.log_odds == 1.0)
    assert sorted(os.listdir(tmp_path)) == ["a.lbmap"]  # no temp files left behind


def read_pgm(path):
    data = open(path, "rb").read()
    parts = data.split(b"\n", 3)
    assert parts[0] == b"P5"
    w, h = map(int, parts[1].split())
    assert parts[2] == b"255"
    pix = np.frombuffer(parts[3], dtype=np.uint8)
    assert pix.size == w * h
    return w, h, pix.reshape(h, w)


def test_export_ros_map(tmp_path):
    g = OccupancyGrid(4, 3, 0.1, origin_xy=(-0.2, 0.5))
    g.log_odds[0, 0] = 4.0   # lowest y, occupied
    g.log_odds[0, 1] = -4.0  # lowest y, free
    g.log_odds[2, 3] = 4.0   # highest y, occupied
    pgm, yml = tmp_path / "s.pgm", tmp_path / "s.yaml"
    export_ros_map(g, pgm, yml)
    w, h, img = read_pgm(pgm)
    assert (w, h) == (4, 3)
    expected = np.full((3, 4), 205, np.uint8)
    expected[2, 0] = 0      # grid row 0 -> image row h-1
    expected[2, 1] = 254
    expected[0, 3] = 0      # grid row 2 (highest y) -> image row 0
    assert np.array_equal(img, expected)
    y = {}
    for line in yml.read_text().splitlines():
        if line.strip():
            k, v = line.split(":", 1)
            y[k.strip()] = v.strip()
    assert y["image"] == "s.pgm"
    assert float(y["resolution"]) == pytest.approx(0.1)
    assert [float(t) for t in y["origin"].strip("[]").split(",")] == pytest.approx([-0.2, 0.5, 0.0])
    assert int(y["negate"]) == 0
    assert float(y["occupied_thresh"]) == pytest.approx(0.65)
    assert float(y["free_thresh"]) == pytest.approx(0.196)


def test_export_thresholds(tmp_path):
    g = OccupancyGrid(3, 1, 0.05)
    g.log_odds[0, :] = [-0.4, 0.0, 0.85]  # p ~= 0.40, 0.5, 0.70
    export_ros_map(g, tmp_path / "a.pgm", tmp_path / "a.yaml")
    assert read_pgm(tmp_path / "a.pgm")[2].tolist() == [[205, 205, 0]]
    export_ros_map(g, tmp_path / "b.pgm", tmp_path / "b.yaml", occ_threshold=0.9, free_threshold=0.45)
    assert read_pgm(tmp_path / "b.pgm")[2].tolist() == [[254, 205, 205]]
