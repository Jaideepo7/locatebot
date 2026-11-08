"""Ultrasonic-only obstacles -> the ``glass_or_low`` map layer (M3).

The forward HC-SR04 on the Pi (``link.messages.Range``) sees things the 2D LiDAR misses: glass
(the laser passes through), objects below the scan plane (a shoe, a low step), thin legs between
beams. For each reading at range ``r`` with the localized pose at its stamp: if **no** LiDAR
point inside the sensor's cone is nearer than ``r + lidar_margin_m``, the cone's arc at ``r`` is
marked in ``glass_or_low`` (+``weight`` per reading, capped at 1). If the LiDAR does see
something there, it is an ordinary obstacle and nothing is added. ``range_m=None`` (no echo)
adds nothing: unknown, never free.

The layer is never cleared by LiDAR free-space rays (the LiDAR cannot see what it marks); it
fades with a long half-life (``half_life_s``, 10 min) and blocks planning where its value is >=
``blocks_above`` (0.6: two consistent readings), via ``LayeredMap.planning_grid``.

Synthetic tests only. On the car: mount offset/yaw of the sensor, its real cone (the HC-SR04's
~15 deg is nominal and range dependent), soft surfaces that absorb the ping, floor echoes when it
is mounted low, and cross-talk.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .layers import LayeredMap
from .types import Pose2D, Scan

LAYER_NAME = "glass_or_low"


@dataclass(frozen=True)
class UltrasonicConfig:
    mount_xy: tuple[float, float] = (0.0, 0.0)   # sensor position in the robot frame (measure!)
    mount_yaw_rad: float = 0.0                   # sensor axis vs robot forward
    lidar_margin_m: float = 0.2                  # LiDAR point nearer than r + margin = LiDAR sees it
    min_range_m: float = 0.03
    max_range_m: float = 2.0                     # only trust near readings
    weight: float = 0.35                         # added per reading (low weight)
    max_value: float = 1.0
    blocks_above: float = 0.6                    # planning obstacle at/above this
    half_life_s: float = 600.0                   # slow decay
    note_recent_s: float = 3.0                   # world-state note while a mark is this fresh


def _direction(bearing_rad: float) -> str:
    d = math.degrees(math.atan2(math.sin(bearing_rad), math.cos(bearing_rad)))
    if abs(d) <= 5.0:
        return "ahead"
    return f"{abs(d):.0f}° {'left' if d > 0 else 'right'}"


class UltrasonicLayer:
    def __init__(self, layers: LayeredMap, config: UltrasonicConfig | None = None) -> None:
        self.config = config or UltrasonicConfig()
        c = self.config
        self.layers = layers
        self.layer = layers.register_layer(LAYER_NAME, fill=0.0, params={
            "rule": "ultrasonic hit with no LiDAR point in the cone nearer than r + margin",
            "weight": c.weight, "half_life_s": c.half_life_s, "blocks_above": c.blocks_above,
            "never_cleared_by_lidar": True})
        self.n_readings = 0
        self.n_marked = 0
        self.n_lidar_saw = 0
        self._last_mark: tuple[float, np.ndarray, float] | None = None  # (stamp, map xy, range)

    def on_range(self, range_m: float | None, fov_rad: float, pose: Pose2D, scan: Scan | None,
                 stamp_s: float) -> bool:
        """One reading with the robot pose (and the LiDAR scan) at its stamp. True = marked."""
        c = self.config
        self.n_readings += 1
        if range_m is None or not math.isfinite(range_m) or not c.min_range_m <= range_m <= c.max_range_m:
            return False
        half = max(1e-3, float(fov_rad) / 2.0)
        mx, my = c.mount_xy
        cy, sy = math.cos(c.mount_yaw_rad), math.sin(c.mount_yaw_rad)
        if scan is not None:
            p = scan.points_xy()
            if len(p):
                qx, qy = p[:, 0] - mx, p[:, 1] - my
                lx, ly = cy * qx + sy * qy, -sy * qx + cy * qy        # sensor frame
                inside = (np.abs(np.arctan2(ly, lx)) <= half) & (np.hypot(lx, ly) < range_m + c.lidar_margin_m)
                if inside.any():
                    self.n_lidar_saw += 1
                    return False
        g = self.layers.static
        n = max(3, int(math.ceil(2.0 * half * range_m / (g.resolution_m / 2.0))) + 1)
        a = np.linspace(-half, half, n) + c.mount_yaw_rad
        local = np.column_stack((mx + range_m * np.cos(a), my + range_m * np.sin(a)))
        world = pose.transform_points(local)
        flat = np.unique(g._flat(g.world_to_cell(world)))
        if len(flat):
            d = self.layer.data.reshape(-1)
            d[flat] = np.minimum(d[flat] + np.float32(c.weight), np.float32(c.max_value))
        self.n_marked += 1
        centre = pose.transform_points(np.array([[mx + range_m * math.cos(c.mount_yaw_rad),
                                                  my + range_m * math.sin(c.mount_yaw_rad)]]))[0]
        self._last_mark = (stamp_s, centre, range_m)
        return True

    def note(self, pose: Pose2D, now: float) -> str | None:
        """'glass or low obstacle: 0.6 m ahead (ultrasonic only)' while a mark is fresh."""
        if self._last_mark is None or now - self._last_mark[0] > self.config.note_recent_s:
            return None
        _, xy, _ = self._last_mark
        dx, dy = xy[0] - pose.x, xy[1] - pose.y
        b = math.atan2(dy, dx) - pose.theta
        return f"glass or low obstacle: {math.hypot(dx, dy):.1f} m {_direction(b)} (ultrasonic only)"

    def stats(self) -> dict:
        return {"readings": self.n_readings, "marked": self.n_marked, "lidar_saw": self.n_lidar_saw}
