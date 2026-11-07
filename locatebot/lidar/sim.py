"""Synthetic rooms and scans for tests and offline demos. Integrator-owned.

Everything produced here is synthetic: it proves software behaviour, not real sensor behaviour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .types import RANGE_MAX_M, Pose2D, Scan


@dataclass(frozen=True)
class Room:
    """A set of wall segments, shape (M, 4) rows of x1, y1, x2, y2 in metres."""

    segments: np.ndarray

    @staticmethod
    def box(width_m: float, height_m: float) -> "Room":
        w, h = width_m, height_m
        return Room(np.array([[0, 0, w, 0], [w, 0, w, h], [w, h, 0, h], [0, h, 0, 0]], dtype=np.float64))

    def with_box(self, x1: float, y1: float, x2: float, y2: float) -> "Room":
        extra = np.array([[x1, y1, x2, y1], [x2, y1, x2, y2], [x2, y2, x1, y2], [x1, y2, x1, y1]], dtype=np.float64)
        return Room(np.vstack((self.segments, extra)))

    def with_segment(self, x1: float, y1: float, x2: float, y2: float) -> "Room":
        return Room(np.vstack((self.segments, [[x1, y1, x2, y2]])))


def raycast(room: Room, origin_xy: tuple[float, float], angles_world: np.ndarray) -> np.ndarray:
    """Distance from origin to the nearest wall along each world-frame angle (inf if none)."""
    ox, oy = origin_xy
    dx = np.cos(angles_world)[:, None]
    dy = np.sin(angles_world)[:, None]
    seg = room.segments
    x1, y1, x2, y2 = (seg[:, i][None, :] for i in range(4))
    ex, ey = x2 - x1, y2 - y1
    denom = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    hit = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
    t = np.where(hit, t, np.inf)
    return t.min(axis=1)


def simulate_scan(
    room: Room,
    pose: Pose2D,
    n_beams: int = 450,
    noise_std_m: float = 0.01,
    range_max_m: float = RANGE_MAX_M,
    rng: np.random.Generator | None = None,
    stamp_s: float = 0.0,
    seq: int = 0,
    dropout: float = 0.0,
) -> Scan:
    """Simulate one revolution. Beam i is at robot angle -pi + 2*pi*i/n_beams."""
    rng = rng if rng is not None else np.random.default_rng(0)
    angles = -math.pi + 2.0 * math.pi * np.arange(n_beams) / n_beams
    r = raycast(room, (pose.x, pose.y), angles + pose.theta)
    if noise_std_m > 0:
        r = r + rng.normal(0.0, noise_std_m, size=r.shape)
    r = np.where(r > range_max_m, np.nan, r)
    if dropout > 0:
        r = np.where(rng.random(r.shape) < dropout, np.nan, r)
    q = np.where(np.isfinite(r), 47, 0).astype(np.uint8)
    return Scan(stamp_s=stamp_s, seq=seq, angles_rad=angles, ranges_m=r, quality=q)
