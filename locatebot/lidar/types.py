"""Shared LiDAR types. Integrator-owned: lanes must not change these.

Robot frame: +x forward, +y left, angles counter-clockwise from forward, radians.
Units: metres, seconds (monotonic clock), radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

RANGE_MIN_M = 0.05
RANGE_MAX_M = 12.0


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _frozen(arr, dtype) -> np.ndarray:
    out = np.array(arr, dtype=dtype, copy=True)
    out.setflags(write=False)
    return out


@dataclass(frozen=True, eq=False)
class Scan:
    """One full 360-degree LiDAR revolution in the robot frame.

    ranges_m uses NaN for "no return". Angles need not be sorted or uniform.
    """

    stamp_s: float
    seq: int
    angles_rad: np.ndarray
    ranges_m: np.ndarray
    quality: np.ndarray

    def __post_init__(self) -> None:
        a = _frozen(self.angles_rad, np.float64)
        r = _frozen(self.ranges_m, np.float64)
        q = _frozen(self.quality, np.uint8)
        if not (a.ndim == r.ndim == q.ndim == 1 and len(a) == len(r) == len(q)):
            raise ValueError("angles_rad, ranges_m and quality must be 1-D and equal length")
        if not math.isfinite(self.stamp_s):
            raise ValueError("stamp_s must be finite")
        object.__setattr__(self, "angles_rad", a)
        object.__setattr__(self, "ranges_m", r)
        object.__setattr__(self, "quality", q)

    def __len__(self) -> int:
        return len(self.ranges_m)

    def valid_mask(self, range_min_m: float = RANGE_MIN_M, range_max_m: float = RANGE_MAX_M) -> np.ndarray:
        r = self.ranges_m
        with np.errstate(invalid="ignore"):
            return np.isfinite(r) & (r >= range_min_m) & (r <= range_max_m)

    def points_xy(self, mask: np.ndarray | None = None) -> np.ndarray:
        """(N, 2) points in the robot frame for valid (or masked) returns."""
        m = self.valid_mask() if mask is None else mask
        r = self.ranges_m[m]
        a = self.angles_rad[m]
        return np.column_stack((r * np.cos(a), r * np.sin(a)))


@dataclass(frozen=True)
class Pose2D:
    """Robot pose in the map frame. theta is wrapped to [-pi, pi)."""

    x: float
    y: float
    theta: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "theta", wrap_angle(float(self.theta)))

    def compose(self, other: "Pose2D") -> "Pose2D":
        """self (+) other: apply `other` expressed in self's frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(self.x + c * other.x - s * other.y, self.y + s * other.x + c * other.y, self.theta + other.theta)

    def inverse(self) -> "Pose2D":
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, -self.theta)

    def transform_points(self, pts: np.ndarray) -> np.ndarray:
        """Map (N, 2) points from this pose's local frame into the parent frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        return np.column_stack((c * p[:, 0] - s * p[:, 1] + self.x, s * p[:, 0] + c * p[:, 1] + self.y))
