"""Pure-pursuit path follower for a car-like (Ackermann) chassis that cannot reverse.

Pose is the rear-axle centre in the map frame. Each ``update``:
1. Projects the pose onto the path (searching forward from the last progress point, so a path
   that loops back on itself is not short-circuited). Cross-track error above
   ``max_cross_track_m`` -> ``NEEDS_REPLAN``.
2. Picks the lookahead point ``L = clip(lookahead_min_m + lookahead_time_s * speed)`` metres
   further along the path (clamped to its end).
3. Curvature ``k = 2 y / d^2`` with the target at (x, y) in the robot frame, d = its distance;
   steering ``atan(wheelbase * k) / max_steer_rad`` clipped to [-1, 1] (positive = left). The
   tightest possible turn is the car's ``min_turn_radius_m``: there is no turning on the spot.
4. Target behind the car (x < 0) and inside the full-lock turning circle on its side (so no
   forward arc reaches it), or behind and needing a loop wider than
   ``max_behind_radius_factor * min_turn_radius_m`` -> ``NEEDS_REPLAN`` (reverse is blocked).
5. Throttle = max_throttle x min(curvature scale, goal-distance scale, clearance scale), floored
   at ``min_throttle_frac`` while following so the car does not stall. Never above
   ``VehicleConfig.max_throttle``, never negative.
6. ``REACHED`` when within ``goal_tolerance_m`` of the end, or when the end is behind the car and
   within ``overshoot_tolerance_m``. ``REACHED`` and ``NEEDS_REPLAN`` always command throttle 0.
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass

import numpy as np

from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig

from .sim import throttle_to_speed


class PursuitStatus(str, enum.Enum):
    FOLLOWING = "following"
    REACHED = "reached"
    NEEDS_REPLAN = "needs_replan"


@dataclass(frozen=True)
class PursuitConfig:
    lookahead_min_m: float = 0.3
    lookahead_max_m: float = 0.8
    lookahead_time_s: float = 1.0          # lookahead grows by speed * this
    goal_tolerance_m: float = 0.15
    overshoot_tolerance_m: float = 0.3
    slowdown_dist_m: float = 0.6           # start slowing this far (along the path) from the end
    min_throttle_frac: float = 0.35        # fraction of max_throttle while following
    curvature_min_frac: float = 0.5        # speed fraction at full-lock curvature
    clearance_slow_m: float = 0.5          # slow down when clearance (if given) is below this
    clearance_stop_m: float = 0.15         # ... reaching min_throttle_frac at this clearance
    max_cross_track_m: float = 0.25
    max_behind_radius_factor: float = 3.0
    search_window_m: float = 1.5           # how far ahead of progress to look for the projection

    def __post_init__(self) -> None:
        if not 0 < self.lookahead_min_m <= self.lookahead_max_m:
            raise ValueError("need 0 < lookahead_min_m <= lookahead_max_m")
        if not 0 <= self.min_throttle_frac <= 1 or not 0 < self.curvature_min_frac <= 1:
            raise ValueError("throttle fractions must be in [0, 1]")
        if self.goal_tolerance_m <= 0 or self.max_cross_track_m <= 0:
            raise ValueError("tolerances must be > 0")


@dataclass(frozen=True)
class PursuitCommand:
    throttle: float                       # [0, max_throttle]
    steering: float                       # [-1, 1], positive = left
    status: PursuitStatus
    target_xy: tuple[float, float] | None = None
    curvature: float = 0.0                # commanded (after steering saturation), 1/m
    cross_track_m: float = 0.0
    remaining_m: float = 0.0              # along the path from the projection to the end
    lookahead_m: float = 0.0
    detail: str = ""


class PurePursuit:
    def __init__(self, vehicle: VehicleConfig | None = None, config: PursuitConfig | None = None) -> None:
        self.vehicle = vehicle or VehicleConfig()
        self.config = config or PursuitConfig()
        self._pts = np.zeros((0, 2))
        self._s = np.zeros(0)
        self._progress = 0.0
        self._last_speed = 0.0

    # --- path ------------------------------------------------------------------------------
    def set_path(self, path) -> None:
        p = np.asarray(path, dtype=np.float64).reshape(-1, 2)
        if len(p) >= 2:
            keep = np.concatenate(([True], np.hypot(*np.diff(p, axis=0).T) > 1e-9))
            p = p[keep]
        self._pts = p
        self._s = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T)))) if len(p) else np.zeros(0)
        self._progress = 0.0

    @property
    def path(self) -> np.ndarray:
        return self._pts

    @property
    def progress_m(self) -> float:
        """Arc length of the last projection onto the path."""
        return self._progress

    def remaining_path(self) -> list[tuple[float, float]]:
        """The path from the current projection to the end (for re-checking against a new map)."""
        if len(self._pts) == 0:
            return []
        i = int(np.searchsorted(self._s, self._progress, side="right"))
        return [self._point_at(self._progress)] + [tuple(map(float, p)) for p in self._pts[i:]]

    @property
    def length_m(self) -> float:
        return float(self._s[-1]) if len(self._s) else 0.0

    def _point_at(self, s: float) -> tuple[float, float]:
        s = min(max(s, 0.0), self.length_m)
        i = int(np.searchsorted(self._s, s, side="right") - 1)
        i = min(max(i, 0), len(self._pts) - 2)
        seg = self._s[i + 1] - self._s[i]
        t = 0.0 if seg <= 0 else (s - self._s[i]) / seg
        a, b = self._pts[i], self._pts[i + 1]
        return float(a[0] + t * (b[0] - a[0])), float(a[1] + t * (b[1] - a[1]))

    def _project(self, x: float, y: float) -> tuple[float, float]:
        """(arc length of the nearest point within the search window, distance to it)."""
        lo = self._progress
        hi = lo + self.config.search_window_m
        n_seg = len(self._pts) - 1
        i0 = min(n_seg - 1, max(0, int(np.searchsorted(self._s, lo, side="right") - 1)))
        i1 = max(i0 + 1, min(n_seg, int(np.searchsorted(self._s, hi, side="right"))))
        a = self._pts[i0:i1]
        b = self._pts[i0 + 1:i1 + 1]
        ab = b - a
        L2 = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12)
        t = np.clip(((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / L2, 0.0, 1.0)
        px = a[:, 0] + t * ab[:, 0]
        py = a[:, 1] + t * ab[:, 1]
        d = np.hypot(px - x, py - y)
        s = self._s[i0:i1] + t * np.sqrt(L2)
        valid = s >= lo - 1e-9
        d = np.where(valid, d, np.inf)
        k = int(np.argmin(d))
        return float(max(s[k], lo)), float(d[k])

    # --- control ---------------------------------------------------------------------------
    def _stop(self, status: PursuitStatus, detail: str, **kw) -> PursuitCommand:
        self._last_speed = 0.0
        return PursuitCommand(0.0, 0.0, status, detail=detail, **kw)

    def update(self, pose: Pose2D, speed_mps: float | None = None,
               clearance_m: float | None = None) -> PursuitCommand:
        """One control step. ``speed_mps``: measured speed (default: last commanded).
        ``clearance_m``: optional free distance around/ahead of the car for speed scaling."""
        cfg, veh = self.config, self.vehicle
        if len(self._pts) == 0:
            return self._stop(PursuitStatus.NEEDS_REPLAN, "no path")
        end = (float(self._pts[-1, 0]), float(self._pts[-1, 1]))
        c, s_ = math.cos(pose.theta), math.sin(pose.theta)

        def to_robot(p):
            dx, dy = p[0] - pose.x, p[1] - pose.y
            return c * dx + s_ * dy, -s_ * dx + c * dy

        d_end = math.hypot(end[0] - pose.x, end[1] - pose.y)
        if d_end <= cfg.goal_tolerance_m:
            return self._stop(PursuitStatus.REACHED, "at goal", target_xy=end)
        if len(self._pts) == 1:
            return self._stop(PursuitStatus.NEEDS_REPLAN, "single-point path is not at the goal")

        s_proj, cross = self._project(pose.x, pose.y)
        self._progress = s_proj
        remaining = self.length_m - s_proj
        ex, _ = to_robot(end)
        if ex < 0 and d_end <= cfg.overshoot_tolerance_m and remaining <= cfg.overshoot_tolerance_m:
            return self._stop(PursuitStatus.REACHED, "overshot goal within tolerance", target_xy=end)
        if cross > cfg.max_cross_track_m:
            return self._stop(PursuitStatus.NEEDS_REPLAN, f"off path by {cross:.2f} m",
                              cross_track_m=cross, remaining_m=remaining)

        v = self._last_speed if speed_mps is None else max(0.0, float(speed_mps))
        L = min(cfg.lookahead_max_m, max(cfg.lookahead_min_m, cfg.lookahead_min_m + cfg.lookahead_time_s * v))
        target = self._point_at(s_proj + L)
        tx, ty = to_robot(target)
        d2 = tx * tx + ty * ty
        if d2 < 1e-12:
            return self._stop(PursuitStatus.REACHED, "on target", target_xy=end)
        k = 2.0 * ty / d2
        rmin = veh.min_turn_radius_m
        kmax = 1.0 / rmin
        if tx < 0:
            radius = math.inf if abs(ty) < 1e-9 else d2 / (2.0 * abs(ty))
            if radius < rmin or radius > cfg.max_behind_radius_factor * rmin:
                return self._stop(PursuitStatus.NEEDS_REPLAN,
                                  f"target behind the car ({tx:.2f}, {ty:.2f}) m: no forward arc",
                                  target_xy=target, cross_track_m=cross, remaining_m=remaining)
        steer = math.atan(veh.wheelbase_m * k) / veh.max_steer_rad
        steer = max(-1.0, min(1.0, steer))
        k_cmd = math.tan(steer * veh.max_steer_rad) / veh.wheelbase_m

        # speed scaling
        curv = 1.0 - (1.0 - cfg.curvature_min_frac) * min(1.0, abs(k) / kmax)
        goal = min(1.0, remaining / cfg.slowdown_dist_m) if cfg.slowdown_dist_m > 0 else 1.0
        clear = 1.0
        if clearance_m is not None and cfg.clearance_slow_m > cfg.clearance_stop_m:
            clear = (clearance_m - cfg.clearance_stop_m) / (cfg.clearance_slow_m - cfg.clearance_stop_m)
            clear = min(1.0, max(0.0, clear))
        frac = max(cfg.min_throttle_frac, min(curv, goal, clear))
        throttle = min(veh.max_throttle, max(0.0, veh.max_throttle * frac))
        self._last_speed = throttle_to_speed(veh, throttle)
        return PursuitCommand(throttle, steer, PursuitStatus.FOLLOWING, target_xy=target, curvature=k_cmd,
                              cross_track_m=cross, remaining_m=remaining, lookahead_m=L)
