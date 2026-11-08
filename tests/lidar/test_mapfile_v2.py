"""LBMAP2 (named layers) and backward compatibility with LBMAP1 (synthetic data only)."""
from __future__ import annotations

import hashlib
import struct

import numpy as np
import pytest

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.mapfile import (MapFileError, MapLayer, export_ros_map, load_map, load_map_layers,
                                     save_map)


def sample_grid() -> OccupancyGrid:
    g = OccupancyGrid(7, 5, 0.05, origin_xy=(-1.25, 2.5))
    g.log_odds[:] = np.random.default_rng(3).uniform(-4, 4, size=(5, 7)).astype(np.float32)
    return g


def three_layers(g):
    rng = np.random.default_rng(4)
    return {
        "glass_or_low": MapLayer(rng.uniform(0, 1, g.log_odds.shape).astype(np.float32),
                                 params={"rule": "ultrasonic-only hits", "blocks_above": 0.6}),
        "wifi_rssi": MapLayer(rng.integers(0, 255, g.log_odds.shape).astype(np.uint8), scale=0.5, offset=-120.0,
                              params={"units": "dBm", "nodata": 0}),
    }


def test_v1_writer_output_is_unchanged_golden_bytes(tmp_path):
    p = tmp_path / "g.lbmap"
    save_map(p, sample_grid(), {"robot": "lb1", "n": [1, 2]})
    b = p.read_bytes()
    # sha256 of the LBMAP1 bytes written before M2 (main @ ca07a18)
    assert len(b) == 219 and b[:8] == b"LBMAP1\0\0"
    assert hashlib.sha256(b).hexdigest() == "54e08c155db33987dd794a8bc2fe2421d3227d8ccd06ccee8c14941784a28af1"
    save_map(p, sample_grid(), {"robot": "lb1", "n": [1, 2]}, layers={})  # empty layers -> still v1
    assert hashlib.sha256(p.read_bytes()).hexdigest() == hashlib.sha256(b).hexdigest()


def test_v2_round_trip_with_three_layers(tmp_path):
    g = sample_grid()
    layers = three_layers(g)
    p = tmp_path / "l.lbmap"
    save_map(p, g, {"sessions": 3}, layers=layers)
    assert p.read_bytes()[:8] == b"LBMAP2\0\0"
    g2, meta, got = load_map_layers(p)
    assert meta == {"sessions": 3}
    assert np.array_equal(g2.log_odds, g.log_odds) and g2.origin_xy == g.origin_xy
    assert list(got) == ["glass_or_low", "wifi_rssi"]
    for name, lay in layers.items():
        assert got[name].data.dtype == lay.data.dtype
        assert np.array_equal(got[name].data, lay.data)
        assert got[name].scale == lay.scale and got[name].offset == lay.offset
        assert got[name].params == lay.params
    assert np.allclose(got["wifi_rssi"].values(), layers["wifi_rssi"].data * 0.5 - 120.0)
    # load_map (old API) still works on v2: static layer only
    g3, meta3 = load_map(p)
    assert np.array_equal(g3.log_odds, g.log_odds) and meta3 == meta


def test_v1_files_load_with_no_layers(tmp_path):
    p = tmp_path / "v1.lbmap"
    save_map(p, sample_grid())
    g, meta, layers = load_map_layers(p)
    assert layers == {} and np.array_equal(g.log_odds, sample_grid().log_odds)


@pytest.mark.parametrize("where", [10, 60, -40, -2])
def test_v2_corrupted_byte_raises(tmp_path, where):
    p = tmp_path / "c.lbmap"
    g = sample_grid()
    save_map(p, g, {"a": 1}, layers=three_layers(g))
    b = bytearray(p.read_bytes())
    b[where] ^= 0x40
    p.write_bytes(bytes(b))
    with pytest.raises(MapFileError):
        load_map_layers(p)


def test_v2_truncated_and_bad_layers(tmp_path):
    g = sample_grid()
    p = tmp_path / "t.lbmap"
    save_map(p, g, layers=three_layers(g))
    p.write_bytes(p.read_bytes()[:-30])
    with pytest.raises(MapFileError):
        load_map(p)
    with pytest.raises(ValueError):
        save_map(p, g, layers={"static": np.zeros((5, 7), np.float32)})
    with pytest.raises(ValueError):
        save_map(p, g, layers={"x": np.zeros((4, 7), np.float32)})
    with pytest.raises(ValueError):
        save_map(p, g, layers={"x": np.zeros((5, 7), np.complex64)})


def test_v2_layout_header(tmp_path):
    g = sample_grid()
    p = tmp_path / "h.lbmap"
    save_map(p, g, {"k": "v"}, layers={"a": np.ones((5, 7), np.int16)})
    b = p.read_bytes()
    version, res, ox, oy, w, h, ml, nl = struct.unpack_from("<HdddIIIH", b, 8)
    assert (version, res, ox, oy, w, h, nl) == (2, 0.05, -1.25, 2.5, 7, 5, 2)
    off = 8 + struct.calcsize("<HdddIIIH") + ml
    (n,) = struct.unpack_from("<H", b, off)
    assert b[off + 2:off + 2 + n] == b"static"


def test_pgm_export_still_shows_the_static_layer(tmp_path):
    g = sample_grid()
    save_map(tmp_path / "e.lbmap", g, layers=three_layers(g))
    g2, _ = load_map(tmp_path / "e.lbmap")
    export_ros_map(g2, tmp_path / "a.pgm", tmp_path / "a.yaml")
    export_ros_map(g, tmp_path / "b.pgm", tmp_path / "b.yaml")
    assert (tmp_path / "a.pgm").read_bytes() == (tmp_path / "b.pgm").read_bytes()
