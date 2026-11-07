"""Tests for locatebot.lidar.detect.

All scans here are synthetic (locatebot.lidar.sim): they prove software behaviour,
not real RPLIDAR C1 behaviour.
"""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from locatebot.lidar.detect import (
    Obstacle,
    clean_scan,
    cluster_scan,
    corridor_clearance,
    forward_coverage,
    sector_clearance,
)
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, Scan


def _with_ranges(scan: Scan, ranges, quality=None) -> Scan:
    q = scan.quality if quality is None else quality
    return Scan(scan.stamp_s, scan.seq, scan.angles_rad, ranges, q)


# ---------------------------------------------------------------- clean_scan

def test_clean_scan_removes_invalid_and_far_spike_keeps_three_point_edge():
    room = Room.box(4.0, 4.0)
    scan = simulate_scan(room, Pose2D(2.0, 2.0, 0.0), n_beams=360, noise_std_m=0.0, stamp_s=1.5, seq=7)
    r = scan.ranges_m.copy()
    q = scan.quality.copy()
    r[10] = np.nan            # no return
    r[20] = 0.01              # below RANGE_MIN_M
    r[30] = 20.0              # above RANGE_MAX_M
    q[40] = 0                 # low quality
    r[100] = r[100] + 1.0     # isolated FAR spike (both neighbours agree)
    r[200:203] = r[200:203] - 1.0  # genuine 3-point object edge
    r[0] = r[0] + 1.0         # far spike on the -pi/+pi seam (neighbours are beams 359 and 1)
    out = clean_scan(_with_ranges(scan, r, q))

    assert out.stamp_s == 1.5 and out.seq == 7
    np.testing.assert_array_equal(out.angles_rad, scan.angles_rad)
    for i in (10, 20, 30, 40, 100, 0):
        assert np.isnan(out.ranges_m[i]), i
    np.testing.assert_allclose(out.ranges_m[200:203], r[200:203])
    untouched = np.ones(len(r), bool)
    untouched[[0, 10, 20, 30, 40, 100]] = False
    np.testing.assert_allclose(out.ranges_m[untouched], r[untouched])


def test_clean_scan_keeps_point_next_to_unknown_neighbour():
    # A point with a NaN neighbour is not provably a spike: keep it (fail-safe).
    a = np.linspace(-0.1, 0.1, 5)
    r = np.array([2.0, np.nan, 1.0, 2.0, 2.0])
    out = clean_scan(Scan(0.0, 0, a, r, np.full(5, 47)))
    assert out.ranges_m[2] == 1.0


def test_clean_scan_handles_unsorted_angles():
    a = np.array([0.02, -0.02, 0.0, 0.04, -0.04])
    r = np.array([2.0, 2.0, 3.0, 2.0, 2.0])   # beam at angle 0 is a far spike
    out = clean_scan(Scan(0.0, 0, a, r, np.full(5, 47)))
    assert np.isnan(out.ranges_m[2])
    assert np.isfinite(out.ranges_m[[0, 1, 3, 4]]).all()


def test_clean_scan_removes_single_far_outlier_between_agreeing_neighbours():
    a = np.linspace(-0.1, 0.1, 5)
    r = np.array([1.0, 1.0, 3.0, 1.0, 1.0])
    out = clean_scan(Scan(0.0, 0, a, r, np.full(5, 47)))
    assert np.isnan(out.ranges_m[2])
    np.testing.assert_allclose(out.ranges_m[[0, 1, 3, 4]], 1.0)


def test_clean_scan_keeps_single_beam_thin_obstacle_closer_than_neighbours():
    # Safety: a thin pole hit by one beam is CLOSER than both neighbours; it must survive.
    wall = Room(np.array([[3.0, -3.0, 3.0, 3.0]]))
    scan = simulate_scan(wall, Pose2D(0.0, 0.0, 0.0), n_beams=450, noise_std_m=0.0)
    r = scan.ranges_m.copy()
    i = int(np.argmin(np.abs(scan.angles_rad)))   # beam at angle 0
    r[i] = 1.0
    pole = _with_ranges(scan, r)
    assert corridor_clearance(pole, half_width_m=0.2) == pytest.approx(1.0, abs=1e-6)
    out = clean_scan(pole)
    assert out.ranges_m[i] == 1.0
    assert corridor_clearance(out, half_width_m=0.2) == pytest.approx(1.0, abs=1e-6)


# ---------------------------------------------------------- sector_clearance

def test_sector_clearance_box_in_front_left_wall_and_empty_sector():
    room = Room.box(6.0, 4.0).with_box(2.0, 1.8, 2.3, 2.2)
    scan = simulate_scan(room, Pose2D(1.0, 2.0, 0.0), noise_std_m=0.0)
    s = sector_clearance(scan, 8)
    assert s.shape == (8,)
    assert s[0] == pytest.approx(1.0, abs=1e-6)
    assert s[2] == pytest.approx(2.0, abs=1e-3)
    assert s[4] == pytest.approx(1.0, abs=1e-3)   # back wall

    open_room = Room(np.array([[2.0, -0.5, 2.0, 0.5]]))  # within sector 0 (+-22.5 deg)
    s2 = sector_clearance(simulate_scan(open_room, Pose2D(0.0, 0.0, 0.0), noise_std_m=0.0), 8)
    assert s2[0] == pytest.approx(2.0, abs=1e-6)
    assert np.isnan(s2[1:]).all()   # unknown, NOT free


def test_sector_clearance_sector_boundaries_and_quality():
    w = 2 * math.pi / 8
    a = np.array([w / 2 - 1e-6, w / 2 + 1e-6, math.pi - 1e-9, -math.pi])
    r = np.array([1.0, 2.0, 3.0, 4.0])
    s = sector_clearance(Scan(0.0, 0, a, r, np.full(4, 47)), 8)
    assert s[0] == 1.0 and s[1] == 2.0 and s[4] == 3.0
    assert np.isnan(s[[2, 3, 5, 6, 7]]).all()


# -------------------------------------------------------- corridor_clearance

def test_corridor_clearance_inside_outside_and_none():
    pose = Pose2D(0.0, 0.0, 0.0)
    inside = Room(np.array([[1.5, -0.1, 1.5, 0.1], [-1.0, -1.0, -1.0, 1.0]]))
    scan = simulate_scan(inside, pose, n_beams=1000, noise_std_m=0.0)
    assert corridor_clearance(scan, 0.2) == pytest.approx(1.5, abs=1e-6)
    assert corridor_clearance(scan, 0.2, max_lookahead_m=1.0) == math.inf

    outside = Room(np.array([[1.5, 0.25, 1.5, 0.5], [-1.0, -1.0, -1.0, 1.0]]))
    assert corridor_clearance(simulate_scan(outside, pose, n_beams=1000, noise_std_m=0.0), 0.2) == math.inf

    behind_only = Room(np.array([[-1.0, -1.0, -1.0, 1.0]]))
    assert corridor_clearance(simulate_scan(behind_only, pose, noise_std_m=0.0), 0.2) == math.inf


# ----------------------------------------------------------- forward_coverage

def test_forward_coverage_full_scan_is_one():
    scan = simulate_scan(Room.box(4.0, 4.0), Pose2D(2.0, 2.0, 0.0), n_beams=450)
    assert forward_coverage(scan) == pytest.approx(1.0)
    assert forward_coverage(scan, fov_rad=2 * math.pi) == pytest.approx(1.0)


def test_forward_coverage_drops_with_forward_dropout():
    room = Room.box(4.0, 4.0)
    scan = simulate_scan(room, Pose2D(2.0, 2.0, 0.0))
    assert forward_coverage(scan) == pytest.approx(1.0)

    r = scan.ranges_m.copy()
    r[np.abs(scan.angles_rad) < 0.5] = np.nan     # 1 rad of the forward half unknown
    dropped = _with_ranges(scan, r)
    cov = forward_coverage(dropped)
    assert (math.pi - 1.0) / math.pi - 0.03 < cov < (math.pi - 1.0) / math.pi + 0.03
    # rear half is untouched, so a rear-only "FOV" check would still be fine
    assert forward_coverage(dropped, fov_rad=2 * math.pi) > cov


def test_forward_coverage_no_beams_in_fov():
    a = np.array([3.0, -3.0])
    assert forward_coverage(Scan(0.0, 0, a, np.array([1.0, 1.0]), np.full(2, 47))) == 0.0


def test_forward_coverage_empty_scan_is_zero():
    e = np.zeros(0)
    assert forward_coverage(Scan(0.0, 0, e, e, np.zeros(0, dtype=int))) == 0.0


def test_forward_coverage_counts_missing_beams_as_uncovered():
    # Safety: beams physically absent around forward (not just invalid) must lower coverage.
    scan = simulate_scan(Room.box(4.0, 4.0), Pose2D(2.0, 2.0, 0.0), n_beams=450, noise_std_m=0.0)
    keep = np.abs(scan.angles_rad) >= 0.6
    holed = Scan(scan.stamp_s, scan.seq, scan.angles_rad[keep], scan.ranges_m[keep], scan.quality[keep])
    assert corridor_clearance(holed, half_width_m=0.2) == math.inf   # "nothing seen" ...
    tol = 2 * math.radians(2.0) / math.pi
    assert forward_coverage(holed) <= (math.pi - 1.2) / math.pi + tol   # ... and coverage says so


# --------------------------------------------------------------- cluster_scan

def test_cluster_two_boxes():
    room = Room.box(6.0, 6.0).with_box(4.0, 2.85, 4.3, 3.15).with_box(2.85, 4.5, 3.15, 4.8)
    # noise-free: the nearest point on a flat face is noise-dominated, so bearings need clean ranges
    scan = simulate_scan(room, Pose2D(3.0, 3.0, 0.0), noise_std_m=0.0)
    obs = cluster_scan(scan)
    noisy = cluster_scan(simulate_scan(room, Pose2D(3.0, 3.0, 0.0), noise_std_m=0.01))
    assert noisy[0].nearest_range_m == pytest.approx(1.0, abs=0.04)
    assert noisy[1].nearest_range_m == pytest.approx(1.5, abs=0.04)
    assert all(isinstance(o, Obstacle) for o in obs)
    assert [o.nearest_range_m for o in obs] == sorted(o.nearest_range_m for o in obs)
    a, b = obs[0], obs[1]
    assert a.nearest_range_m == pytest.approx(1.0, abs=0.02)
    assert a.bearing_rad == pytest.approx(0.0, abs=0.05)
    assert b.nearest_range_m == pytest.approx(1.5, abs=0.02)
    assert b.bearing_rad == pytest.approx(math.pi / 2, abs=0.05)
    assert a.width_m == pytest.approx(0.3, abs=0.05)
    assert a.centroid_xy[0] == pytest.approx(1.0, abs=0.05)
    assert a.n_points >= 3
    assert obs[2].nearest_range_m > 2.5   # everything else is wall


def test_cluster_box_straddling_seam_is_one_cluster():
    room = Room.box(4.0, 4.0).with_box(1.0, 1.8, 1.3, 2.2)
    scan = simulate_scan(room, Pose2D(2.0, 2.0, 0.0), noise_std_m=0.0)
    obs = cluster_scan(scan)
    near = [o for o in obs if o.nearest_range_m < 1.0]
    assert len(near) == 1
    box = near[0]
    assert box.nearest_range_m == pytest.approx(0.7, abs=0.02)
    assert abs(abs(box.bearing_rad) - math.pi) < 0.05
    expected = int(np.sum(simulate_scan(room, Pose2D(2.0, 2.0, 0.0), noise_std_m=0.0).ranges_m < 1.0))
    assert box.n_points == expected
    assert box.width_m == pytest.approx(0.4, abs=0.05)


def test_cluster_empty_room_has_no_small_clusters():
    room = Room.box(4.0, 6.0)
    scan = simulate_scan(room, Pose2D(1.5, 2.5, 0.3))
    r = scan.ranges_m.copy()
    r[100:102] = 1.0          # 2-point blob, below min_points
    obs = cluster_scan(_with_ranges(scan, r))
    assert obs
    assert all(o.n_points >= 3 for o in obs)
    assert all(o.nearest_range_m > 1.2 for o in obs)
    assert any(o.nearest_range_m == pytest.approx(1.0) for o in cluster_scan(_with_ranges(scan, r), min_points=2))


def test_cluster_all_unknown_is_empty():
    a = np.linspace(-math.pi, math.pi, 10, endpoint=False)
    assert cluster_scan(Scan(0.0, 0, a, np.full(10, np.nan), np.zeros(10))) == []


# --------------------------------------------------------------------- timing

def test_timing_1000_beams():
    room = Room.box(5.0, 4.0).with_box(3.0, 1.0, 3.4, 1.5).with_box(1.0, 3.0, 1.3, 3.3)
    scan = simulate_scan(room, Pose2D(2.0, 2.0, 0.4), n_beams=1000, dropout=0.05)
    cleaned = clean_scan(scan)
    funcs = {
        "clean_scan": lambda: clean_scan(scan),
        "sector_clearance": lambda: sector_clearance(cleaned),
        "corridor_clearance": lambda: corridor_clearance(cleaned, 0.15),
        "forward_coverage": lambda: forward_coverage(cleaned),
        "cluster_scan": lambda: cluster_scan(cleaned),
    }
    total = 0.0
    for name, fn in funcs.items():
        fn()
        best = min(_timed(fn) for _ in range(20))
        total += best
        print(f"{name}: {best * 1e3:.3f} ms for 1000 beams")
        assert best < 0.05, name   # generous bound; target is well under 5 ms on the Pi
    print(f"total: {total * 1e3:.3f} ms for 1000 beams")


def _timed(fn) -> float:
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0
