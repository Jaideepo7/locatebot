"""Wi-Fi link strength on the Orin -> the ``wifi_rssi`` map layer (M3).

``WifiReader`` reads the RSSI (dBm) of the current Wi-Fi link: from ``/proc/net/wireless`` when
it lists an interface, else from ``iw dev <iface> link``; the source is chosen at runtime and a
machine with neither (or no Wi-Fi link) is simply ``disabled`` (``read()`` returns None, never an
error). ``WifiLayer`` writes readings into ``wifi_rssi`` at the robot's cell as a running mean
(count in ``wifi_rssi_n``), for link dead zones; it never affects planning or occupancy.
Both layers are saved in ``.lbmap`` v2.

Parsers are tested on sample outputs; the real interface name and values on the Orin are not
verified here.
"""
from __future__ import annotations

import math
import re
import shutil
import subprocess
from dataclasses import dataclass
from typing import Callable

import numpy as np

from locatebot.lidar.layers import LayeredMap
from locatebot.lidar.types import Pose2D

LAYER = "wifi_rssi"
COUNT_LAYER = "wifi_rssi_n"


def parse_proc_net_wireless(text: str) -> dict[str, float]:
    """``/proc/net/wireless`` -> {iface: level dBm}. Two header lines, then e.g.
    ``wlan0: 0000   57.  -53.  -256        0      0      0      0      0        0``."""
    out: dict[str, float] = {}
    for line in text.splitlines()[2:]:
        if ":" not in line:
            continue
        iface, rest = line.split(":", 1)
        f = rest.split()
        if len(f) < 3:
            continue
        try:
            level = float(f[2].rstrip("."))
        except ValueError:
            continue
        if level > 0:  # some drivers report an unsigned byte
            level -= 256.0
        if -120.0 <= level < 0.0:
            out[iface.strip()] = level
    return out


_SIGNAL = re.compile(r"signal:\s*(-?\d+(?:\.\d+)?)\s*dBm")
_IFACE = re.compile(r"^\s*Interface\s+(\S+)", re.M)


def parse_iw_link(text: str) -> float | None:
    """``iw dev <iface> link`` -> signal dBm, None when ``Not connected.``."""
    m = _SIGNAL.search(text)
    return float(m.group(1)) if m else None


def parse_iw_dev(text: str) -> list[str]:
    """``iw dev`` -> interface names."""
    return _IFACE.findall(text)


class WifiReader:
    """RSSI source chosen at runtime: ``proc``, ``iw`` or ``None`` (disabled)."""

    def __init__(self, iface: str | None = None, proc_path: str = "/proc/net/wireless",
                 which: Callable[[str], str | None] = shutil.which,
                 run: Callable[..., subprocess.CompletedProcess] = subprocess.run) -> None:
        self.iface = iface
        self.proc_path = proc_path
        self._run = run
        self.mode: str | None = None
        levels = self._proc()
        if levels and (iface is None or iface in levels):
            self.mode = "proc"
            self.iface = iface or next(iter(levels))
            return
        self._iw = which("iw")
        if self._iw:
            if self.iface is None:
                names = parse_iw_dev(self._call([self._iw, "dev"]) or "")
                self.iface = names[0] if names else None
            if self.iface:
                self.mode = "iw"

    @property
    def enabled(self) -> bool:
        return self.mode is not None

    def _proc(self) -> dict[str, float]:
        try:
            with open(self.proc_path, encoding="ascii", errors="replace") as f:
                return parse_proc_net_wireless(f.read())
        except OSError:
            return {}

    def _call(self, cmd: list[str]) -> str | None:
        try:
            p = self._run(cmd, capture_output=True, text=True, timeout=2.0)
        except (OSError, subprocess.SubprocessError):
            return None
        return p.stdout if p.returncode == 0 else None

    def read(self) -> float | None:
        """Current link RSSI in dBm, or None (disabled / not connected / read failed)."""
        if self.mode == "proc":
            return self._proc().get(self.iface)
        if self.mode == "iw":
            out = self._call([self._iw, "dev", self.iface, "link"])
            return parse_iw_link(out) if out else None
        return None


@dataclass(frozen=True)
class WifiConfig:
    weak_dbm: float = -75.0        # summary note below this
    nearby_m: float = 3.0          # "strongest nearby" search radius
    stale_s: float = 5.0           # no note from a reading older than this


class WifiLayer:
    def __init__(self, layers: LayeredMap, config: WifiConfig | None = None) -> None:
        self.config = config or WifiConfig()
        self.layers = layers
        self.mean = layers.register_layer(LAYER, fill=np.nan, params={
            "units": "dBm", "rule": "running mean per cell of the Orin's Wi-Fi RSSI", "nodata": "NaN",
            "affects_planning": False})
        self.count = layers.register_layer(COUNT_LAYER, fill=0, dtype=np.uint16,
                                           params={"rule": "readings per cell for wifi_rssi"})
        self.last: tuple[float, float] | None = None   # (stamp, dBm)

    def add(self, dbm: float, pose: Pose2D, stamp_s: float) -> bool:
        if dbm is None or not math.isfinite(dbm):
            return False
        g = self.layers.static
        cr = g.world_to_cell(np.array([[pose.x, pose.y]]))
        self.last = (stamp_s, float(dbm))
        if not g.in_bounds(cr)[0]:
            return False
        c, r = int(cr[0, 0]), int(cr[0, 1])
        n = int(self.count.data[r, c])
        m = float(self.mean.data[r, c])
        self.mean.data[r, c] = dbm if n == 0 or not math.isfinite(m) else m + (dbm - m) / (n + 1)
        self.count.data[r, c] = min(n + 1, 65535)
        return True

    def strongest_near(self, pose: Pose2D) -> tuple[float, float, float] | None:
        g = self.layers.static
        m = self.mean.data
        rows, cols = np.nonzero(np.isfinite(m))
        if len(rows) == 0:
            return None
        xy = g.cell_to_world(np.column_stack((cols, rows)))
        near = np.hypot(xy[:, 0] - pose.x, xy[:, 1] - pose.y) <= self.config.nearby_m
        if not near.any():
            return None
        vals = m[rows[near], cols[near]]
        k = int(np.argmax(vals))
        return float(xy[near][k, 0]), float(xy[near][k, 1]), float(vals[k])

    def note(self, pose: Pose2D, now: float) -> str | None:
        """'wifi: weak here (-80 dBm); strongest nearby at (x, y)' when the link is weak."""
        if self.last is None or now - self.last[0] > self.config.stale_s or self.last[1] >= self.config.weak_dbm:
            return None
        dbm = self.last[1]
        s = f"wifi: weak here ({dbm:.0f} dBm)"
        best = self.strongest_near(pose)
        if best is not None and best[2] > dbm + 3.0:
            s += f"; strongest nearby at ({best[0]:.1f}, {best[1]:.1f}) ({best[2]:.0f} dBm)"
        return s
