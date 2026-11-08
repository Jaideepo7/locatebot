"""Live session mapping on the Orin (build step 2: map the room while driving manually).

``LiveMapper`` wraps ``lidar.mapping.Mapper`` for a running robot: one thread feeds scans as
they arrive from the Pi; other threads (vision localization, the skill executor, the agent,
the dashboard) read the pose, a copy of the grid, and pose/scan history at a frame's capture
time. All access goes through one lock. The map frame is the robot pose at the first accepted
scan, so "home" is (0, 0, 0).

``LiveMapper.from_saved(path, guess)`` (M1) instead loads a saved ``.lbmap`` and localizes in it
(``lidar.localize.LocalizingMapper``): the map frame is the saved map's, ``home`` its stored home
pose, and the map is updated slowly. ``stats()`` reports ``mode`` (``mapping``/``localizing``),
``lost``, ``ambiguous`` and ``last_score``.

In that mode the map is layered by default (M2, ``lidar.layers.LayeredMap``): new/moving things
go to a short-lived dynamic layer, ``change_blobs()`` lists them, and ``grid_snapshot()`` returns
the planning grid (room + dynamic + blocking extra layers) so skills plan around them.
"""
from __future__ import annotations

import copy
import threading
from pathlib import Path

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.layers import ChangeBlob, LayeredMap
from locatebot.lidar.localize import LocalizerConfig, LocalizeResult, LocalizingMapper
from locatebot.lidar.mapping import Mapper
from locatebot.lidar.odometry import OdometryResult
from locatebot.lidar.types import Pose2D, Scan
from locatebot.localize.items import PoseHistory, ScanHistory


class LiveMapper:
    def __init__(self, size_m: float = 20.0, resolution_m: float = 0.05,
                 pose_history_s: float = 30.0, scan_match_dt_s: float = 0.06,
                 engine: Mapper | LocalizingMapper | None = None) -> None:
        self._lock = threading.Lock()
        self._mapper = engine if engine is not None else Mapper(size_m=size_m, resolution_m=resolution_m)
        self._poses = PoseHistory(max_age_s=pose_history_s)
        self._scans = ScanHistory(max_dt_s=scan_match_dt_s)
        self._latest: Scan | None = None
        self._last_result: OdometryResult | LocalizeResult | None = None
        self.home = getattr(self._mapper, "home", Pose2D(0.0, 0.0, 0.0))

    @classmethod
    def from_saved(cls, path, guess: Pose2D | None = None, config: LocalizerConfig | None = None,
                   slow_rate: float = 0.2, layered: bool = True, **kw) -> "LiveMapper":
        """Localize in a saved ``.lbmap`` instead of starting a new map. ``guess`` is where the
        car is in that map (dock pose, operator click); None = global search over the map.
        ``home`` = the map's stored home pose (where it was first mapped), else the guess.
        ``layered``: static/dynamic layers and change detection (M2)."""
        engine = LocalizingMapper.from_file(path, guess=guess, config=config, slow_rate=slow_rate,
                                            layered=layered)
        return cls(engine=engine, **kw)

    @property
    def layers(self) -> LayeredMap | None:
        """The layered map (localizing + layered mode) or None. Mutate only under ``locked()``."""
        return getattr(self._mapper, "layers", None)

    def locked(self) -> threading.Lock:
        """The mapper lock, for modules that update an extra layer (``with lm.locked(): ...``)."""
        return self._lock

    def change_blobs(self, min_persistence: int = 3) -> list[ChangeBlob]:
        """Current new/moving things vs the saved room map, seen in >= ``min_persistence`` scans
        (one-scan flickers are hidden: on the real C1 replays they were mixed-pixel edges).
        Empty when not layered."""
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            return lay.blobs(min_persistence=min_persistence) if lay is not None else []

    @property
    def localizing(self) -> bool:
        return isinstance(self._mapper, LocalizingMapper)

    def relocalize(self, guess: Pose2D | None = None) -> None:
        """Localizing mode: drop the current track and re-initialize around ``guess`` (None =
        global search) on the next scan. No-op when building a new map."""
        with self._lock:
            if isinstance(self._mapper, LocalizingMapper):
                self._mapper.localizer.request_init(guess)

    def add_scan(self, scan: Scan) -> OdometryResult | LocalizeResult:
        """Integrate one cleaned-or-raw scan (Pi clock stamps, in time order)."""
        with self._lock:
            result = self._mapper.add_scan(scan)
            self._last_result = result
            self._latest = scan
            self._scans.add(scan)
            if result.accepted:
                try:
                    self._poses.add(scan.stamp_s, result.pose)
                except ValueError:
                    pass  # out-of-order stamp: keep the map, skip the history entry
            return result

    @property
    def pose(self) -> Pose2D:
        with self._lock:
            traj = self._mapper.trajectory
            if traj:
                _, x, y, th = traj[-1]
                return Pose2D(x, y, th)
            return self.home

    @property
    def latest_scan(self) -> Scan | None:
        with self._lock:
            return self._latest

    def pose_at(self, stamp_s: float) -> Pose2D | None:
        with self._lock:
            return self._poses.at(stamp_s)

    def scan_near(self, stamp_s: float) -> Scan | None:
        with self._lock:
            return self._scans.nearest(stamp_s)

    def grid_snapshot(self) -> OccupancyGrid:
        """Independent copy of the grid, safe to plan on while mapping continues. In layered
        mode: the planning grid (room + dynamic things + blocking extra layers)."""
        with self._lock:
            lay = getattr(self._mapper, "layers", None)
            if lay is not None:
                return lay.planning_grid()
            g = copy.copy(self._mapper.grid)
            g.log_odds = self._mapper.grid.log_odds.copy()
            return g

    def stats(self) -> dict:
        with self._lock:
            m = self._mapper
            r = self._last_result
            out = {"scans": m.n_scans, "integrated": m.n_integrated, "rejected": m.n_rejected,
                   "mode": "localizing" if isinstance(m, LocalizingMapper) else "mapping",
                   "lost": bool(getattr(r, "lost", False)), "ambiguous": bool(getattr(r, "ambiguous", False)),
                   "last_score": None if r is None else round(float(r.score), 3)}
            if isinstance(m, LocalizingMapper):
                loc = m.localizer
                out.update({"initialized": loc.initialized, "lost": loc.lost,
                            "ambiguous": loc.ambiguous and not loc.initialized,
                            "source": None if r is None else r.source,
                            "lost_events": loc.n_lost_events, "recoveries": loc.n_recoveries,
                            "layered": m.layers is not None,
                            "changes": len(m.layers.blobs(min_persistence=3)) if m.layers is not None else 0})
            return out

    def save(self, stem: str | Path, meta: dict | None = None) -> dict[str, Path]:
        with self._lock:
            return self._mapper.save(stem, meta=meta)
