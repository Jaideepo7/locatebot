"""Tests for keyframe-based LiDAR odometry (no wheel encoders).

All scans are synthetic (locatebot.lidar.sim): these prove software behaviour, not
real-sensor behaviour. Rooms carry interior boxes so they are not symmetric.
"""
import math
import time

import numpy as np
import pytest

from locatebot.lidar.odometry import OdometryResult, ScanOdometry
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D


def _room() -> Room:
    return (
        Room.box(7.0, 6.0)
        .with_box(5.0, 4.8, 6.0, 5.5)
        .with_box(0.4, 4.8, 1.0, 5.6)
        .with_segment(6.2, 0.0, 6.2, 1.0)
    )


def _trajectory(n_steps: int = 60) -> list[Pose2D]:
    """Ground-truth poses: 0.08 m per step, turning up to 4 deg per step."""
    turn = np.zeros(n_steps)
    turn[12:34] = math.radians(4.0)    # ~88 deg left
    turn[40:50] = math.radians(-3.0)   # 30 deg right
    poses = [Pose2D(1.5, 1.0, 0.0)]
    for k in range(n_steps):
        poses.append(poses[-1].compose(Pose2D(0.08, 0.0, float(turn[k]))))
    return poses


def _rel(p0: Pose2D, p: Pose2D) -> Pose2D:
    return p0.inverse().compose(p)


def test_trajectory_stays_inside_room_clear_of_obstacles():
    for p in _trajectory():
        assert 0.6 < p.x < 6.0 and 0.6 < p.y < 4.4, p


def test_first_scan_is_origin_keyframe():
    odo = ScanOdometry()
    scan = simulate_scan(_room(), Pose2D(1.5, 1.0, 0.4), rng=np.random.default_rng(0))
    res = odo.update(scan)
    assert isinstance(res, OdometryResult)
    assert res.accepted and res.is_keyframe
    assert res.pose == Pose2D(0.0, 0.0, 0.0)
    assert odo.pose == res.pose


@pytest.mark.parametrize("seed", [42, 7, 2024])
def test_sixty_step_trajectory_drift_is_bounded(seed):
    room = _room()
    gt = _trajectory(60)
    rng = np.random.default_rng(seed)
    odo = ScanOdometry()
    results = []
    for k, p in enumerate(gt):
        scan = simulate_scan(room, p, n_beams=450, noise_std_m=0.01, dropout=0.02, rng=rng, stamp_s=0.1 * k, seq=k)
        results.append(odo.update(scan))
    truth = _rel(gt[0], gt[-1])
    est = odo.pose
    err_xy = math.hypot(est.x - truth.x, est.y - truth.y)
    err_th = abs(math.degrees(est.theta - truth.theta))
    n_acc = sum(r.accepted for r in results)
    n_kf = sum(r.is_keyframe for r in results)
    print(f"60-step drift (seed {seed}): {err_xy * 100:.2f} cm, {err_th:.3f} deg; accepted {n_acc}/{len(results)}, keyframes {n_kf}")
    assert n_acc == len(results)
    assert 5 <= n_kf <= 40
    assert err_xy < 0.10
    assert err_th < 3.0


def test_constant_velocity_straight_run_converges_every_step():
    # 0.1 m/step = 1 m/s at 10 Hz, beyond the default 0.3 m keyframe spacing per 3 steps
    room = _room()
    rng = np.random.default_rng(7)
    odo = ScanOdometry()
    p = Pose2D(1.0, 2.0, 0.0)
    for k in range(40):
        scan = simulate_scan(room, p, noise_std_m=0.01, dropout=0.02, rng=rng, seq=k)
        res = odo.update(scan)
        assert res.accepted, f"step {k} rejected (score {res.score:.3f})"
        p = p.compose(Pose2D(0.1, 0.0, 0.0))
    # ended at x = 1.0 + 3.9 m relative 3.9 m forward
    assert abs(odo.pose.x - 3.9) < 0.08 and abs(odo.pose.y) < 0.08


def test_rejected_scan_keeps_prediction_and_adds_no_keyframe():
    room = _room()
    rng = np.random.default_rng(3)
    odo = ScanOdometry()
    p = Pose2D(1.5, 2.0, 0.0)
    for k in range(3):
        odo.update(simulate_scan(room, p, rng=rng, seq=k))
        p = p.compose(Pose2D(0.05, 0.0, 0.0))
    before = odo.pose
    junk = simulate_scan(Room.box(2.0, 12.0).with_box(0.2, 1.0, 0.6, 1.5), Pose2D(1.0, 6.0, 1.2), rng=rng, seq=99)
    res = odo.update(junk)
    assert not res.accepted and not res.is_keyframe
    # predicted with constant velocity: ~0.05 m further forward
    assert abs(res.pose.x - (before.x + 0.05)) < 0.02
    assert odo.pose == res.pose


def test_update_timing_450_beams():
    room = _room()
    rng = np.random.default_rng(11)
    odo = ScanOdometry()
    gt = _trajectory(20)
    times = []
    for k, p in enumerate(gt):
        scan = simulate_scan(room, p, n_beams=450, noise_std_m=0.01, dropout=0.02, rng=rng, seq=k)
        t0 = time.perf_counter()
        odo.update(scan)
        times.append(time.perf_counter() - t0)
    steady = times[1:]
    print(f"update timing: median {np.median(steady) * 1e3:.1f} ms, max {max(steady) * 1e3:.1f} ms")
    assert np.median(steady) < 0.050
    assert max(steady) < 0.100  # includes keyframe/field rebuilds; generous bound for CI noise
