"""Exploration-session mapping pipeline: Scan -> clean -> odometry -> occupancy grid -> map files.

Integrator-owned glue over the lane modules. Runs on the Orin; feed it scans forwarded from the Pi.
"""
from __future__ import annotations

import math
import time
from pathlib import Path

from .detect import clean_scan
from .grid import OccupancyGrid
from .mapfile import export_ros_map, save_map
from .odometry import OdometryResult, ScanOdometry
from .scanlog import read_scan_log
from .sim import Room
from .types import RANGE_MAX_M, Pose2D, Scan


class Mapper:
    """Builds one session map. Only scans whose pose was accepted by odometry touch the grid."""

    def __init__(self, size_m: float = 20.0, resolution_m: float = 0.05,
                 odometry: ScanOdometry | None = None, max_range_m: float = RANGE_MAX_M) -> None:
        self.grid = OccupancyGrid.around((0.0, 0.0), size_m, resolution_m)
        self.odometry = odometry if odometry is not None else ScanOdometry()
        self.max_range_m = max_range_m
        self.trajectory: list[tuple[float, float, float, float]] = []  # (stamp, x, y, theta) of integrated scans
        self.n_scans = 0
        self.n_integrated = 0
        self.n_rejected = 0

    def add_scan(self, scan: Scan) -> OdometryResult:
        self.n_scans += 1
        cleaned = clean_scan(scan)
        result = self.odometry.update(cleaned)
        if result.accepted:
            self.grid.integrate(cleaned, result.pose, max_range_m=self.max_range_m)
            p = result.pose
            self.trajectory.append((cleaned.stamp_s, p.x, p.y, p.theta))
            self.n_integrated += 1
        else:
            self.n_rejected += 1
        return result

    def save(self, stem: str | Path, meta: dict | None = None) -> dict[str, Path]:
        """Write <stem>.lbmap (the session map file) plus <stem>.pgm/.yaml for viewing."""
        stem = Path(stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        full_meta = {
            "format": "locatebot-session-map",
            "created_unix_s": time.time(),
            "scans": self.n_scans,
            "integrated": self.n_integrated,
            "rejected": self.n_rejected,
            "trajectory": self.trajectory,
        }
        full_meta.update(meta or {})
        paths = {"lbmap": stem.with_suffix(".lbmap"), "pgm": stem.with_suffix(".pgm"), "yaml": stem.with_suffix(".yaml")}
        save_map(paths["lbmap"], self.grid, full_meta)
        export_ros_map(self.grid, paths["pgm"], paths["yaml"])
        return paths


def build_map_from_log(log_path: str | Path, out_stem: str | Path, size_m: float = 20.0,
                       resolution_m: float = 0.05, meta: dict | None = None) -> dict:
    """Replay a recorded .lbscan file into a session map. Returns a small summary dict."""
    mapper = Mapper(size_m=size_m, resolution_m=resolution_m)
    t0 = time.perf_counter()
    for scan in read_scan_log(log_path):
        mapper.add_scan(scan)
    elapsed = time.perf_counter() - t0
    paths = mapper.save(out_stem, meta={"source_log": str(log_path), **(meta or {})})
    return {
        "scans": mapper.n_scans,
        "integrated": mapper.n_integrated,
        "rejected": mapper.n_rejected,
        "seconds": elapsed,
        "final_pose": [mapper.odometry.pose.x, mapper.odometry.pose.y, mapper.odometry.pose.theta],
        "files": {k: str(v) for k, v in paths.items()},
    }


def demo_room() -> Room:
    """Asymmetric 8 x 6 m synthetic room with furniture-like boxes and a partial wall."""
    return (Room.box(8.0, 6.0)
            .with_box(1.0, 4.2, 2.2, 5.0)
            .with_box(6.6, 0.5, 7.4, 1.6)
            .with_box(3.6, 3.0, 4.0, 3.4)
            .with_segment(6.5, 6.0, 6.5, 4.2))


def demo_trajectory(n: int = 60) -> list[Pose2D]:
    """A smooth loop through demo_room at about 0.08 m per scan (≈0.8 m/s at 10 Hz)."""
    poses = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        x = 4.0 + 2.2 * math.cos(t)
        y = 3.0 + 1.6 * math.sin(t)
        heading = math.atan2(1.6 * math.cos(t), -2.2 * math.sin(t))
        poses.append(Pose2D(x, y, heading))
    return poses
