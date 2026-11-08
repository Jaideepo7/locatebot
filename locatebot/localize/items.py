"""Place detected items on the map.

A detection gives a bearing (robot frame, radians, left positive) and the frame's capture
time. ``localize`` looks along that bearing *from the camera* in the LiDAR scan taken at the
same moment, takes the nearest return as the item's range, and transforms the point into the
map frame with the robot pose at that time. No range, no position: a box is never turned into
a map point by guessing a distance.

Limits: only works for items roughly at LiDAR height; an item on a shelf above the scan plane
gets the range of whatever is below or behind it. Camera position, heading alignment and
the bearing error must be measured on the car (``LocalizeConfig``). Pose comes from LiDAR
odometry, so map positions inherit its drift.
"""
from __future__ import annotations

import bisect
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from locatebot.lidar.types import Pose2D, Scan, wrap_angle


@dataclass(frozen=True)
class LocalizeConfig:
    camera_xy: tuple[float, float] = (0.0, 0.0)  # camera position in the robot frame, metres
    window_rad: float = math.radians(2.0)        # half-width of the ray search
    max_range_m: float = 6.0                     # beyond this the fix is too uncertain to use
    range_sigma_m: float = 0.05                  # LiDAR range noise
    bearing_sigma_rad: float = math.radians(2.0)  # camera bearing error (calibration + box centre)


@dataclass(frozen=True)
class ItemFix:
    x: float
    y: float
    range_m: float
    sigma_m: float   # 1-sigma position uncertainty, metres
    stamp_s: float


def range_along_ray(scan: Scan, bearing_rad: float, camera_xy: tuple[float, float],
                    window_rad: float) -> float | None:
    """Distance from the camera to the nearest valid LiDAR return within ``window_rad`` of the ray."""
    pts = scan.points_xy()
    if len(pts) == 0:
        return None
    rel = pts - np.asarray(camera_xy, dtype=np.float64)
    dist = np.hypot(rel[:, 0], rel[:, 1])
    ang = np.arctan2(rel[:, 1], rel[:, 0])
    dev = np.abs((ang - bearing_rad + math.pi) % (2 * math.pi) - math.pi)
    sel = (dev <= window_rad) & (dist > 0.0)
    if not sel.any():
        return None
    return float(dist[sel].min())


def localize(bearing_rad: float, scan: Scan, pose: Pose2D, config: LocalizeConfig,
             stamp_s: float | None = None) -> ItemFix | None:
    """Item position in the map frame, or None when the LiDAR gives no usable range."""
    r = range_along_ray(scan, bearing_rad, config.camera_xy, config.window_rad)
    if r is None or r > config.max_range_m:
        return None
    cx, cy = config.camera_xy
    local = np.array([[cx + r * math.cos(bearing_rad), cy + r * math.sin(bearing_rad)]])
    x, y = pose.transform_points(local)[0]
    sigma = math.hypot(config.range_sigma_m, r * config.bearing_sigma_rad)
    return ItemFix(float(x), float(y), r, sigma, scan.stamp_s if stamp_s is None else stamp_s)


class PoseHistory:
    """Recent poses by time, to look up where the robot was when a frame was captured."""

    def __init__(self, max_age_s: float = 30.0, max_extrapolation_s: float = 0.1):
        self.max_age_s = max_age_s
        self.max_extrapolation_s = max_extrapolation_s
        self._t: list[float] = []
        self._p: list[Pose2D] = []

    def add(self, stamp_s: float, pose: Pose2D) -> None:
        if not all(math.isfinite(v) for v in (stamp_s, pose.x, pose.y, pose.theta)):
            raise ValueError("pose history entries must be finite")
        if self._t and stamp_s < self._t[-1]:
            raise ValueError("poses must be added in time order")
        self._t.append(stamp_s)
        self._p.append(pose)
        cut = bisect.bisect_left(self._t, stamp_s - self.max_age_s)
        if cut:
            del self._t[:cut], self._p[:cut]

    def at(self, stamp_s: float) -> Pose2D | None:
        if not self._t or not math.isfinite(stamp_s):
            return None
        if stamp_s <= self._t[0]:
            return self._p[0] if self._t[0] - stamp_s <= self.max_extrapolation_s else None
        if stamp_s >= self._t[-1]:
            return self._p[-1] if stamp_s - self._t[-1] <= self.max_extrapolation_s else None
        i = bisect.bisect_right(self._t, stamp_s)
        t0, t1 = self._t[i - 1], self._t[i]
        a, b = self._p[i - 1], self._p[i]
        f = (stamp_s - t0) / (t1 - t0) if t1 > t0 else 0.0
        return Pose2D(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                      a.theta + f * wrap_angle(b.theta - a.theta))


class ScanHistory:
    """Recent scans, to find the one closest to a frame's capture time.

    Keep ``max_dt_s`` small (about half the scan period): the bearing is in the robot frame at
    the frame's capture time but the ray is searched in the scan's frame, so a time gap while
    turning becomes a bearing error (1 rad/s for 0.06 s is about 3.4 degrees).
    """

    def __init__(self, max_dt_s: float = 0.06, maxlen: int = 50):
        self.max_dt_s = max_dt_s
        self._scans: deque[Scan] = deque(maxlen=maxlen)

    def add(self, scan: Scan) -> None:
        self._scans.append(scan)

    def nearest(self, stamp_s: float) -> Scan | None:
        if not self._scans:
            return None
        best = min(self._scans, key=lambda s: abs(s.stamp_s - stamp_s))
        return best if abs(best.stamp_s - stamp_s) <= self.max_dt_s else None


_MIN_SIGMA = 1e-3  # metres; guards the inverse-variance weights


@dataclass
class Track:
    x: float
    y: float
    sigma_m: float
    sightings: int
    last_seen_s: float
    confirmed: bool


class ItemTracker:
    """Merges repeated sightings of each item into tracks (inverse-variance weighted mean).

    A sighting within ``merge_radius_m`` of an existing track of the same item updates the
    nearest one; otherwise it starts a new track (a second instance, or a false detection).
    A track is ``confirmed`` after ``min_sightings`` sightings.
    """

    def __init__(self, merge_radius_m: float = 0.5, min_sightings: int = 2):
        self.merge_radius_m = merge_radius_m
        self.min_sightings = min_sightings
        self._tracks: dict[str, list[Track]] = {}

    def update(self, item: str, fix: ItemFix) -> Track:
        tracks = self._tracks.setdefault(item, [])
        near = [(math.hypot(t.x - fix.x, t.y - fix.y), t) for t in tracks]
        near = [(d, t) for d, t in near if d <= self.merge_radius_m]
        if near:
            _, t = min(near, key=lambda dt: dt[0])
            w1, w2 = 1.0 / max(t.sigma_m, _MIN_SIGMA) ** 2, 1.0 / max(fix.sigma_m, _MIN_SIGMA) ** 2
            t.x = (w1 * t.x + w2 * fix.x) / (w1 + w2)
            t.y = (w1 * t.y + w2 * fix.y) / (w1 + w2)
            t.sigma_m = math.sqrt(1.0 / (w1 + w2))
            t.sightings += 1
            t.last_seen_s = max(t.last_seen_s, fix.stamp_s)
        else:
            t = Track(fix.x, fix.y, fix.sigma_m, 1, fix.stamp_s, False)
            tracks.append(t)
        t.confirmed = t.sightings >= self.min_sightings
        return t

    def tracks(self, item: str) -> list[Track]:
        return list(self._tracks.get(item, []))

    def best(self, item: str) -> Track | None:
        tracks = self._tracks.get(item)
        if not tracks:
            return None
        return max(tracks, key=lambda t: (t.sightings, -t.sigma_m))

    def items(self) -> list[str]:
        return [k for k, v in self._tracks.items() if v]
