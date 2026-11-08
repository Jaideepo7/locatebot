"""People tracking from LiDAR change clusters (M3).

Input per scan: the clusters of NEW points from ``layers.LayeredMap`` (``last_clusters``: map
frame centroid, width) -- things that are not part of the saved room. Candidates:

* **leg pair**: two clusters ``leg_width_m`` wide (default 0.05-0.25 m) whose centres are
  ``leg_gap_m`` apart (0.1-0.5 m) -> one candidate at the midpoint (the gait swings the legs;
  the midpoint stays on the body's path).
* **body**: one cluster ``body_width_m`` wide (0.25-0.6 m): a person close by, two legs merged
  into one cluster, or a LiDAR mounted at hip height.
* **single leg**: an unpaired leg-sized cluster; it only updates an existing track (the other
  leg is occluded), it never starts one.

Geometry is for a LiDAR at ~20-30 cm (shin height). A mount at hip height sees one wider blob:
raise ``leg_width_m``/``body_width_m``. All thresholds are in ``PeopleConfig``.

Tracking: one constant-velocity Kalman filter per track (state x, y, vx, vy, map frame,
white-acceleration process noise), gated nearest-neighbour association (Mahalanobis distance^2 <
``gate_chi2`` and < ``gate_max_m``, best pairs first). A track is born after ``birth_hits``
detections (tentative tracks die after ``tentative_missed_s`` without one) and dies after
``max_missed_s`` without a detection. ``moving`` = speed > ``moving_mps`` (0.15 m/s).

Classification: a confirmed track that has walked (moved > ``walked_m`` from where it was born,
or kept ``moving`` for ``walked_s``) is ``likely_person``; a track that never moved is an object
(a chair's legs look like two leg pairs) unless the camera confirms it. ``wants_camera_check``
offers a likely person without a camera confirmation in the last ``camera_every_s`` (10 s);
``on_camera_fix`` with a vision "person" fix within ``camera_gate_m`` marks the track
``confirmed_by_camera``.

Synthetic tests only (moving boxes as legs): software behaviour, not real people. Known
failure modes: chair legs (rejected only because they never move), pets (small and fast: may
look like one leg; a cat walking is "likely person" until the camera says otherwise), sitting
people (static: object until confirmed), people closer than one cell to a wall (ROOM points,
see layers.py), two people walking side by side < 0.3 m apart (one track), long occlusion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PeopleConfig:
    leg_width_m: tuple[float, float] = (0.05, 0.25)
    leg_gap_m: tuple[float, float] = (0.1, 0.5)
    body_width_m: tuple[float, float] = (0.25, 0.6)
    meas_sigma_m: float = 0.06         # leg-pair midpoint noise (gait)
    body_sigma_m: float = 0.08
    single_leg_sigma_m: float = 0.15   # the other leg is somewhere within a step
    accel_sigma_mps2: float = 1.5      # process noise (people start/stop/turn)
    gate_chi2: float = 9.21            # 99 % for 2 dof
    gate_max_m: float = 1.0
    birth_hits: int = 3
    tentative_missed_s: float = 0.5
    max_missed_s: float = 2.0
    moving_mps: float = 0.15
    walked_m: float = 0.4
    walked_s: float = 1.0
    camera_every_s: float = 10.0
    camera_gate_m: float = 0.7
    max_tracks: int = 20


@dataclass(frozen=True)
class PersonTrack:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    speed_mps: float
    moving: bool
    kind: str                     # last detection: "legs" | "body" | "leg"
    hits: int
    confirmed: bool               # born (>= birth_hits detections)
    likely_person: bool           # confirmed and has walked
    confirmed_by_camera: bool
    first_seen_s: float
    last_seen_s: float
    camera_s: float | None        # last camera confirmation

    @property
    def is_person(self) -> bool:
        return self.confirmed and (self.likely_person or self.confirmed_by_camera)


class _Track:
    def __init__(self, tid: int, xy: np.ndarray, sigma: float, now: float, kind: str) -> None:
        self.id = tid
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        self.P = np.diag([sigma ** 2, sigma ** 2, 1.0, 1.0])
        self.t = now
        self.first_s = now
        self.last_s = now
        self.birth_xy = np.array(xy, dtype=float)
        self.hits = 1
        self.kind = kind
        self.confirmed = False
        self.walked = False
        self.moving_since: float | None = None
        self.camera_s: float | None = None
        self.camera_requested_s: float | None = None
        self.by_camera = False

    def predict(self, now: float, q: float) -> None:
        dt = now - self.t
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + (q * q) * (G @ G.T)
        self.t = now

    def innovation(self, z: np.ndarray, sigma: float) -> tuple[np.ndarray, np.ndarray]:
        y = z - self.x[:2]
        S = self.P[:2, :2] + np.eye(2) * sigma * sigma
        return y, S

    def correct(self, z: np.ndarray, sigma: float) -> None:
        y, S = self.innovation(z, sigma)
        K = self.P[:, :2] @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ np.eye(2, 4)) @ self.P   # H = [I2 0]

    @property
    def speed(self) -> float:
        return float(math.hypot(self.x[2], self.x[3]))


class PeopleTracker:
    def __init__(self, config: PeopleConfig | None = None) -> None:
        self.config = config or PeopleConfig()
        self._tracks: list[_Track] = []
        self._next = 1
        self._now: float | None = None

    # ------------------------------------------------------------------ detection
    def candidates(self, clusters) -> list[tuple[np.ndarray, float, str]]:
        """(xy, sigma, kind) person candidates from (centroid_xy, width_m[, n]) clusters."""
        cfg = self.config
        legs, out = [], []
        for c in clusters:
            xy, w = np.asarray(c[0], dtype=float), float(c[1])
            if cfg.leg_width_m[0] <= w <= cfg.leg_width_m[1]:
                legs.append(xy)
            if cfg.body_width_m[0] <= w <= cfg.body_width_m[1]:
                out.append((xy, cfg.body_sigma_m, "body"))
        pairs = sorted((float(np.hypot(*(legs[i] - legs[j]))), i, j)
                       for i in range(len(legs)) for j in range(i + 1, len(legs)))
        used = set()
        for d, i, j in pairs:
            if i in used or j in used or not cfg.leg_gap_m[0] <= d <= cfg.leg_gap_m[1]:
                continue
            used |= {i, j}
            out.append(((legs[i] + legs[j]) / 2.0, cfg.meas_sigma_m, "legs"))
        for i, xy in enumerate(legs):
            if i not in used:
                out.append((xy, cfg.single_leg_sigma_m, "leg"))
        return out

    # ------------------------------------------------------------------ tracking
    def update(self, now: float, clusters) -> list[PersonTrack]:
        """One scan's NEW-point clusters (map frame). Returns the current tracks."""
        cfg = self.config
        self._now = now
        for t in self._tracks:
            t.predict(now, cfg.accel_sigma_mps2)
        cands = self.candidates(clusters)
        pairs = []
        for ci, (z, sig, _) in enumerate(cands):
            for ti, t in enumerate(self._tracks):
                y, S = t.innovation(z, sig)
                if float(np.hypot(*y)) > cfg.gate_max_m:
                    continue
                d2 = float(y @ np.linalg.solve(S, y))
                if d2 <= cfg.gate_chi2:
                    pairs.append((d2 + (0.5 if cands[ci][2] == "leg" else 0.0), ci, ti))
        pairs.sort()
        used_c, used_t = set(), set()
        for _, ci, ti in pairs:
            if ci in used_c or ti in used_t:
                continue
            # a pair or body detection explains its legs: don't let one leg also claim a track
            used_c.add(ci)
            used_t.add(ti)
            z, sig, kind = cands[ci]
            t = self._tracks[ti]
            t.correct(z, sig)
            t.last_s = now
            t.hits += 1
            t.kind = kind
            if t.hits >= cfg.birth_hits:
                t.confirmed = True
        for ci, (z, sig, kind) in enumerate(cands):
            if ci in used_c or kind == "leg":
                continue
            if any(float(np.hypot(*(z - t.x[:2]))) < 0.3 for t in self._tracks):
                continue  # duplicate of a track that took another detection
            self._tracks.append(_Track(self._next, z, sig, now, kind))
            self._next += 1
        for t in self._tracks:
            if t.speed > cfg.moving_mps:
                t.moving_since = now if t.moving_since is None else t.moving_since
            else:
                t.moving_since = None
            if t.confirmed and (float(np.hypot(*(t.x[:2] - t.birth_xy))) > cfg.walked_m
                                or (t.moving_since is not None and now - t.moving_since >= cfg.walked_s)):
                t.walked = True
        self._tracks = [t for t in self._tracks
                        if now - t.last_s <= (cfg.max_missed_s if t.confirmed else cfg.tentative_missed_s)]
        if len(self._tracks) > cfg.max_tracks:
            self._tracks.sort(key=lambda t: -t.last_s)
            del self._tracks[cfg.max_tracks:]
        return self.tracks()

    def _snap(self, t: _Track) -> PersonTrack:
        return PersonTrack(t.id, float(t.x[0]), float(t.x[1]), float(t.x[2]), float(t.x[3]), t.speed,
                           t.speed > self.config.moving_mps, t.kind, t.hits, t.confirmed,
                           t.confirmed and t.walked, t.by_camera, t.first_s, t.last_s, t.camera_s)

    def tracks(self, people_only: bool = False) -> list[PersonTrack]:
        out = [self._snap(t) for t in self._tracks if t.confirmed]
        return [p for p in out if p.is_person] if people_only else out

    def people(self) -> list[PersonTrack]:
        """Tracks that are people: walked like one, or the camera said so."""
        return self.tracks(people_only=True)

    # ------------------------------------------------------------------ camera
    def wants_camera_check(self, now: float | None = None) -> PersonTrack | None:
        """A likely person without a camera confirmation (or request) in the last
        ``camera_every_s``; the runtime asks vision for "person" only when no search runs."""
        now = self._now if now is None else now
        if now is None:
            return None
        every = self.config.camera_every_s
        best = None
        for t in self._tracks:
            if not (t.confirmed and t.walked):
                continue
            if t.camera_s is not None and now - t.camera_s < every:
                continue
            if t.camera_requested_s is not None and now - t.camera_requested_s < every:
                continue
            if best is None or t.last_s > best.last_s:
                best = t
        return None if best is None else self._snap(best)

    def camera_requested(self, track_id: int, now: float) -> None:
        for t in self._tracks:
            if t.id == track_id:
                t.camera_requested_s = now

    def on_camera_fix(self, xy, stamp_s: float) -> PersonTrack | None:
        """A vision "person" fix (map frame): marks the nearest track within ``camera_gate_m``."""
        best, best_d = None, self.config.camera_gate_m
        for t in self._tracks:
            d = float(np.hypot(t.x[0] - xy[0], t.x[1] - xy[1]))
            if d <= best_d:
                best, best_d = t, d
        if best is None:
            return None
        best.by_camera = True
        best.camera_s = stamp_s
        if best.hits >= self.config.birth_hits:
            best.confirmed = True
        return self._snap(best)
