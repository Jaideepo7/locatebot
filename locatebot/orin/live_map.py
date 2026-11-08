"""Live session mapping on the Orin (build step 2: map the room while driving manually).

``LiveMapper`` wraps ``lidar.mapping.Mapper`` for a running robot: one thread feeds scans as
they arrive from the Pi; other threads (vision localization, the skill executor, the agent,
the dashboard) read the pose, a copy of the grid, and pose/scan history at a frame's capture
time. All access goes through one lock. The map frame is the robot pose at the first accepted
scan, so "home" is (0, 0, 0).
"""
from __future__ import annotations

import copy
import threading
from pathlib import Path

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.mapping import Mapper
from locatebot.lidar.odometry import OdometryResult
from locatebot.lidar.types import Pose2D, Scan
from locatebot.localize.items import PoseHistory, ScanHistory


class LiveMapper:
    def __init__(self, size_m: float = 20.0, resolution_m: float = 0.05,
                 pose_history_s: float = 30.0, scan_match_dt_s: float = 0.06) -> None:
        self._lock = threading.Lock()
        self._mapper = Mapper(size_m=size_m, resolution_m=resolution_m)
        self._poses = PoseHistory(max_age_s=pose_history_s)
        self._scans = ScanHistory(max_dt_s=scan_match_dt_s)
        self._latest: Scan | None = None
        self.home = Pose2D(0.0, 0.0, 0.0)

    def add_scan(self, scan: Scan) -> OdometryResult:
        """Integrate one cleaned-or-raw scan (Pi clock stamps, in time order)."""
        with self._lock:
            result = self._mapper.add_scan(scan)
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
        """Independent copy of the grid, safe to plan on while mapping continues."""
        with self._lock:
            g = copy.copy(self._mapper.grid)
            g.log_odds = self._mapper.grid.log_odds.copy()
            return g

    def stats(self) -> dict:
        with self._lock:
            m = self._mapper
            return {"scans": m.n_scans, "integrated": m.n_integrated, "rejected": m.n_rejected}

    def save(self, stem: str | Path, meta: dict | None = None) -> dict[str, Path]:
        with self._lock:
            return self._mapper.save(stem, meta=meta)
