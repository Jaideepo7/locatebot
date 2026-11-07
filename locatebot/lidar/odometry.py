"""Keyframe-based LiDAR odometry: pose from scan matching alone (the kit has no wheel encoders).

Each scan is matched against a likelihood field built from the last few keyframes'
points (a local submap in the map frame), starting from a constant-velocity
prediction. The map frame is the robot frame of the first scan.

Behaviour is verified on synthetic scans (``locatebot.lidar.sim``) only; real-sensor
performance (motion distortion within a revolution, glass, featureless corridors) is untested.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .matching import LikelihoodField, _subsample, match_scan
from .types import Pose2D, Scan


@dataclass(frozen=True)
class OdometryResult:
    pose: Pose2D
    score: float
    accepted: bool
    is_keyframe: bool


class ScanOdometry:
    def __init__(self, keyframe_dist_m: float = 0.3, keyframe_angle_rad: float = 0.3,
                 submap_keyframes: int = 5, max_points: int = 600, min_score: float = 0.35) -> None:
        if submap_keyframes < 1 or max_points < 1:
            raise ValueError("submap_keyframes and max_points must be >= 1")
        self.keyframe_dist_m = float(keyframe_dist_m)
        self.keyframe_angle_rad = float(keyframe_angle_rad)
        self.max_points = int(max_points)
        self.min_score = float(min_score)
        self._keyframes: deque[tuple[Pose2D, np.ndarray]] = deque(maxlen=int(submap_keyframes))
        self._field: LikelihoodField | None = None
        self._pose = Pose2D(0.0, 0.0, 0.0)
        self._velocity = Pose2D(0.0, 0.0, 0.0)  # per-scan motion in the robot frame
        self._last_accepted: Pose2D | None = None
        self._step = 0
        self._last_accepted_step = 0

    @property
    def pose(self) -> Pose2D:
        return self._pose

    def _add_keyframe(self, pose: Pose2D, local_pts: np.ndarray) -> None:
        self._keyframes.append((pose, pose.transform_points(local_pts)))
        self._field = None  # rebuild lazily on the next match

    def update(self, scan: Scan) -> OdometryResult:
        pts = _subsample(scan.points_xy(), self.max_points)
        self._step += 1

        if self._last_accepted is None:
            if len(pts) == 0:
                return OdometryResult(self._pose, 0.0, False, False)
            self._last_accepted = self._pose
            self._last_accepted_step = self._step
            self._add_keyframe(self._pose, pts)
            return OdometryResult(self._pose, 1.0, True, True)

        predicted = self._pose.compose(self._velocity)
        if self._field is None:
            self._field = LikelihoodField.from_points(np.vstack([p for _, p in self._keyframes]))
        res = match_scan(self._field, pts, predicted, min_score=self.min_score)

        if not res.converged:
            self._pose = predicted
            return OdometryResult(predicted, res.score, False, False)

        pose = res.pose
        # constant velocity from the last two accepted poses, per scan (spread over any rejected gap)
        gap = self._step - self._last_accepted_step
        d = self._last_accepted.inverse().compose(pose)
        self._velocity = Pose2D(d.x / gap, d.y / gap, d.theta / gap)
        self._last_accepted = pose
        self._last_accepted_step = self._step
        self._pose = pose

        kf_pose = self._keyframes[-1][0]
        rel = kf_pose.inverse().compose(pose)
        is_kf = bool(np.hypot(rel.x, rel.y) > self.keyframe_dist_m or abs(rel.theta) > self.keyframe_angle_rad)
        if is_kf:
            self._add_keyframe(pose, pts)
        return OdometryResult(pose, res.score, True, is_kf)
