"""Wi-Fi RSSI parsing, source selection and the wifi_rssi layer (M3). Sample outputs only."""
from __future__ import annotations

import subprocess

import numpy as np

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.layers import LayeredMap
from locatebot.lidar.mapfile import load_map_layers, save_map
from locatebot.lidar.types import Pose2D
from locatebot.orin.wifi import (COUNT_LAYER, LAYER, WifiLayer, WifiReader, parse_iw_dev, parse_iw_link,
                                 parse_proc_net_wireless)

PROC = """Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
 wlan0: 0000   57.  -53.  -256        0      0      0      0     12        0
"""
PROC_UNSIGNED = PROC.replace("-53.", "176.")      # some drivers: 256 - 80
IW_LINK = """Connected to 00:11:22:33:44:55 (on wlan0)
\tSSID: team
\tfreq: 5180
\tsignal: -61 dBm
\ttx bitrate: 433.3 MBit/s
"""
IW_DEV = """phy#0
\tInterface wlP1p1s0
\t\tifindex 3
\t\ttype managed
"""


def test_parsers():
    assert parse_proc_net_wireless(PROC) == {"wlan0": -53.0}
    assert parse_proc_net_wireless(PROC_UNSIGNED) == {"wlan0": -80.0}
    assert parse_proc_net_wireless(PROC.splitlines()[0] + "\n" + PROC.splitlines()[1] + "\n") == {}
    assert parse_proc_net_wireless("garbage\nmore\nwlan0: x y z\n") == {}
    assert parse_iw_link(IW_LINK) == -61.0
    assert parse_iw_link("Not connected.\n") is None
    assert parse_iw_dev(IW_DEV) == ["wlP1p1s0"]


def test_reader_prefers_proc_then_iw_then_disabled(tmp_path):
    p = tmp_path / "wireless"
    p.write_text(PROC)
    r = WifiReader(proc_path=str(p), which=lambda _: None)
    assert r.mode == "proc" and r.iface == "wlan0" and r.read() == -53.0

    calls = []

    def run(cmd, **kw):
        calls.append(cmd)
        out = IW_DEV if cmd[-1] == "dev" else IW_LINK
        return subprocess.CompletedProcess(cmd, 0, out, "")

    r = WifiReader(proc_path=str(tmp_path / "missing"), which=lambda _: "/usr/sbin/iw", run=run)
    assert r.mode == "iw" and r.iface == "wlP1p1s0" and r.read() == -61.0
    assert calls[-1] == ["/usr/sbin/iw", "dev", "wlP1p1s0", "link"]

    off = WifiReader(proc_path=str(tmp_path / "missing"), which=lambda _: None)
    assert not off.enabled and off.read() is None

    def broken(cmd, **kw):
        raise OSError("no iw")

    r = WifiReader(iface="wlan0", proc_path=str(tmp_path / "missing"), which=lambda _: "/x/iw", run=broken)
    assert r.mode == "iw" and r.read() is None                  # a failing tool is None, not an error


def test_layer_running_mean_note_and_v2_round_trip(tmp_path):
    g = OccupancyGrid.around((0.0, 0.0), 6.0)
    g.log_odds[:] = -2.0
    lm = LayeredMap(g)
    w = WifiLayer(lm)
    for k, dbm in enumerate((-50.0, -60.0)):
        assert w.add(dbm, Pose2D(1.0, 1.0, 0.0), float(k))
    c = g.world_to_cell(np.array([[1.0, 1.0]]))[0]
    assert lm.layer(LAYER).data[c[1], c[0]] == -55.0 and lm.layer(COUNT_LAYER).data[c[1], c[0]] == 2
    assert w.note(Pose2D(1, 1, 0), 2.0) is None                  # -60 is not weak
    w.add(-82.0, Pose2D(2.0, 1.0, 0.0), 3.0)
    assert w.note(Pose2D(2, 1, 0), 3.5) == "wifi: weak here (-82 dBm); strongest nearby at (1.0, 1.0) (-55 dBm)"
    assert w.note(Pose2D(2, 1, 0), 30.0) is None                 # stale reading
    assert not w.add(float("nan"), Pose2D(0, 0, 0), 4.0)
    save_map(tmp_path / "w.lbmap", g, {}, layers=lm.save_layers())
    _, _, layers = load_map_layers(tmp_path / "w.lbmap")
    assert layers[LAYER].params["units"] == "dBm" and layers[COUNT_LAYER].data.dtype == np.uint16
    assert np.array_equal(layers[LAYER].data, lm.layer(LAYER).data, equal_nan=True)
    lm2 = LayeredMap(g, extra=layers)
    assert WifiLayer(lm2).mean is lm2.layer(LAYER)                # reloaded layer is reused
    assert lm.planning_grid().log_odds.max() == g.log_odds.max()  # never affects planning
