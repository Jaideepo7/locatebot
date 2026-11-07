"""Tests for the likelihood-field correlative scan matcher.

All scans are synthetic (locatebot.lidar.sim): these prove software behaviour, not
real-sensor behaviour. Rooms carry interior boxes so they are not symmetric.
"""
import math

import numpy as np
import pytest

from locatebot.lidar.matching import LikelihoodField, MatchResult, match_scan
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D


def _room() -> Room:
    return (
        Room.box(7.0, 5.0)
        .with_box(4.6, 3.4, 5.6, 4.2)
        .with_box(1.0, 3.6, 1.6, 4.4)
        .with_segment(6.0, 0.0, 6.0, 1.2)
    )


def _local_points(room: Room, pose: Pose2D, seed: int, noise: float = 0.01, dropout: float = 0.0) -> np.ndarray:
    scan = simulate_scan(room, pose, n_beams=450, noise_std_m=noise, rng=np.random.default_rng(seed), dropout=dropout)
    return scan.points_xy()


def test_field_peaks_on_reference_points_and_is_zero_outside():
    pts = np.array([[0.0, 0.0], [1.0, 0.5], [2.0, -1.0]])
    f = LikelihoodField.from_points(pts, resolution_m=0.05, sigma_m=0.1, margin_m=1.0)
    assert f.score(pts) > 0.9
    # 0.1 m away (one sigma): exp(-0.5) ~= 0.61
    assert f.score(pts + [0.1, 0.0]) == pytest.approx(math.exp(-0.5), abs=0.08)
    # far from any point but inside the raster: ~0
    assert f.score(np.array([[0.0, -0.9]])) < 0.01
    # outside the raster: exactly 0
    assert f.score(np.array([[50.0, 50.0]])) == 0.0
    assert f.score(np.empty((0, 2))) == 0.0


def test_match_recovers_known_offset_from_perturbed_guess():
    room = _room()
    ref_pose = Pose2D(2.5, 2.0, 0.3)
    field = LikelihoodField.from_points(_local_points(room, ref_pose, seed=1))
    truth = Pose2D(0.15, -0.10, math.radians(8.0))  # scan pose in the reference frame
    scan_pts = _local_points(room, ref_pose.compose(truth), seed=2, dropout=0.02)
    initial = Pose2D(0.05, 0.02, math.radians(2.0))

    res = match_scan(field, scan_pts, initial)

    assert isinstance(res, MatchResult)
    err_xy = math.hypot(res.pose.x - truth.x, res.pose.y - truth.y)
    err_th = abs(math.degrees(res.pose.theta - truth.theta))
    print(f"match error: {err_xy * 100:.2f} cm, {err_th:.3f} deg, score {res.score:.3f}")
    assert res.converged
    assert err_xy < 0.02
    assert err_th < 1.0


@pytest.mark.parametrize("seed", [3, 4, 5])
def test_match_accuracy_holds_across_poses(seed):
    room = _room()
    rng = np.random.default_rng(seed)
    ref_pose = Pose2D(rng.uniform(2.0, 4.0), rng.uniform(1.5, 3.0), rng.uniform(-math.pi, math.pi))
    field = LikelihoodField.from_points(_local_points(room, ref_pose, seed=seed + 10))
    truth = Pose2D(rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-0.25, 0.25))
    scan_pts = _local_points(room, ref_pose.compose(truth), seed=seed + 20, dropout=0.02)
    res = match_scan(field, scan_pts, Pose2D(0.0, 0.0, 0.0))
    assert res.converged
    assert math.hypot(res.pose.x - truth.x, res.pose.y - truth.y) < 0.02
    assert abs(math.degrees(res.pose.theta - truth.theta)) < 1.0


def test_different_room_does_not_converge():
    field = LikelihoodField.from_points(_local_points(_room(), Pose2D(2.5, 2.0, 0.0), seed=1))
    other = Room.box(3.0, 9.0).with_box(0.5, 6.0, 1.2, 6.5).with_segment(2.0, 2.0, 3.0, 3.0)
    scan_pts = _local_points(other, Pose2D(1.5, 4.0, 1.0), seed=2)
    res = match_scan(field, scan_pts, Pose2D(0.0, 0.0, 0.0))
    print(f"different-room score {res.score:.3f}")
    assert not res.converged


def test_best_on_window_edge_is_not_converged():
    room = _room()
    ref_pose = Pose2D(2.5, 2.0, 0.0)
    field = LikelihoodField.from_points(_local_points(room, ref_pose, seed=1))
    # true offset 0.6 m: outside a 0.3 m window, so the best is pushed to the edge
    scan_pts = _local_points(room, ref_pose.compose(Pose2D(0.6, 0.0, 0.0)), seed=2)
    res = match_scan(field, scan_pts, Pose2D(0.0, 0.0, 0.0), search_xy_m=0.3)
    assert not res.converged


def test_empty_scan_is_not_converged():
    field = LikelihoodField.from_points(np.array([[0.0, 0.0], [1.0, 1.0]]))
    res = match_scan(field, np.empty((0, 2)), Pose2D(0.1, 0.2, 0.3))
    assert not res.converged
    assert res.pose == Pose2D(0.1, 0.2, 0.3)
