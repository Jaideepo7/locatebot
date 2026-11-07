"""Tests for the log-odds occupancy grid.

All scans here come from locatebot.lidar.sim: they prove software behaviour, not real-sensor behaviour.
"""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from locatebot.lidar.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, Scan

RES = 0.05


def box_grid() -> OccupancyGrid:
    # Offset the origin by half a cell so the walls of Room.box(4, 3) lie on cell centres
    # (no floor() ambiguity for noise-free endpoints exactly on a wall).
    return OccupancyGrid(100, 80, RES, origin_xy=(-0.5 - RES / 2, -0.5 - RES / 2))


def box_scan(n_beams: int = 450, **kw) -> tuple[Scan, Pose2D]:
    pose = Pose2D(2.0, 1.5, 0.3)
    return simulate_scan(Room.box(4, 3), pose, n_beams=n_beams, noise_std_m=0.0, **kw), pose


def test_constants():
    assert (FREE, OCCUPIED, UNKNOWN) == (0, 100, 255)


def test_init_shape_and_dtype():
    g = OccupancyGrid(30, 20, 0.1, origin_xy=(1.0, -2.0))
    assert g.log_odds.shape == (20, 30)
    assert g.log_odds.dtype == np.float32
    assert np.all(g.log_odds == 0)
    assert np.all(g.classify() == UNKNOWN)
    assert np.allclose(g.probability(), 0.5)


def test_around_centres_grid():
    g = OccupancyGrid.around((1.0, 2.0), 4.0, 0.05)
    assert g.log_odds.shape == (80, 80)
    assert g.origin_xy == pytest.approx((-1.0, 0.0))
    assert g.world_to_cell(np.array([[1.0, 2.0]])).tolist() == [[40, 40]]


def test_world_to_cell_floor_and_negative():
    g = OccupancyGrid(10, 10, 0.5, origin_xy=(-1.0, -1.0))
    xy = np.array([[-1.0, -1.0], [-0.99, -0.51], [-0.5, -0.5], [-1.01, -1.2], [0.0, 0.0], [3.99, 3.99]])
    cr = g.world_to_cell(xy)
    assert cr.dtype == np.int64
    assert cr.tolist() == [[0, 0], [0, 0], [1, 1], [-1, -1], [2, 2], [9, 9]]


def test_world_cell_round_trip():
    g = OccupancyGrid(40, 30, 0.05, origin_xy=(-1.234, -0.777))
    cr = np.array([[0, 0], [39, 29], [-3, 5], [12, -7]], dtype=np.int64)
    centres = g.cell_to_world(cr)
    assert centres[0] == pytest.approx([-1.234 + 0.025, -0.777 + 0.025])
    assert np.array_equal(g.world_to_cell(centres), cr)


def test_in_bounds():
    g = OccupancyGrid(5, 3)
    cr = np.array([[0, 0], [4, 2], [5, 0], [0, 3], [-1, 0], [0, -1]])
    assert g.in_bounds(cr).tolist() == [True, True, False, False, False, False]


def test_single_scan_marks_walls_interior_and_unknown_behind():
    g = box_grid()
    scan, pose = box_scan()
    g.integrate(scan, pose)
    cls = g.classify()
    # every endpoint cell is OCCUPIED
    ends = pose.transform_points(scan.points_xy())
    ecr = g.world_to_cell(ends)
    assert np.all(cls[ecr[:, 1], ecr[:, 0]] == OCCUPIED)
    # One scan gives l_free = -0.4 -> p ~= 0.40, which the default free_threshold (0.35) still calls
    # UNKNOWN; check the interior with a looser threshold, and the defaults after a second scan.
    cls_loose = g.classify(free_threshold=0.45)
    centres = g.cell_to_world(np.stack(np.meshgrid(np.arange(100), np.arange(80)), axis=-1).reshape(-1, 2))
    cx = centres[:, 0].reshape(80, 100)
    cy = centres[:, 1].reshape(80, 100)
    interior = (cx > 0.1) & (cx < 3.9) & (cy > 0.1) & (cy < 2.9)
    assert np.all(cls_loose[interior] == FREE)
    behind = (cx < -0.1) | (cx > 4.1) | (cy < -0.1) | (cy > 3.1)
    assert np.all(cls[behind] == UNKNOWN)
    assert np.all(g.log_odds[behind] == 0)
    # wall cells along each wall are mostly OCCUPIED
    on_wall = ((np.abs(cx) < 0.01) | (np.abs(cx - 4) < 0.01)) & (cy > 0.1) & (cy < 2.9)
    assert np.mean(cls[on_wall] == OCCUPIED) > 0.9
    g.integrate(scan, pose)
    assert np.all(g.classify()[interior] == FREE)


def test_nan_beams_change_nothing():
    g = box_grid()
    n = 360
    scan = Scan(0.0, 0, np.linspace(-math.pi, math.pi, n, endpoint=False), np.full(n, np.nan), np.zeros(n))
    g.integrate(scan, Pose2D(2.0, 1.5, 0.0))
    assert np.all(g.log_odds == 0)


def test_nan_beams_leave_their_sector_unknown():
    g = box_grid()
    scan, pose = box_scan()
    r = scan.ranges_m.copy()
    # robot-frame angles in (0, pi/2) carry no return
    r[(scan.angles_rad > 0) & (scan.angles_rad < math.pi / 2)] = np.nan
    blind = Scan(0.0, 0, scan.angles_rad, r, scan.quality)
    g_full = box_grid()
    sub = Scan(0.0, 0, scan.angles_rad[np.isfinite(r)], r[np.isfinite(r)], scan.quality[np.isfinite(r)])
    g.integrate(blind, pose)
    g_full.integrate(sub, pose)
    # NaN beams contribute exactly nothing: same result as dropping them
    assert np.array_equal(g.log_odds, g_full.log_odds)
    # a cell deep inside the blind sector stays unknown
    ang = pose.theta + math.pi / 4
    p = np.array([[pose.x + 1.0 * math.cos(ang), pose.y + 1.0 * math.sin(ang)]])
    c = g.world_to_cell(p)[0]
    assert g.log_odds[c[1], c[0]] == 0


def test_repeated_integration_saturates():
    g = box_grid()
    scan, pose = box_scan()
    for _ in range(30):
        g.integrate(scan, pose)
    assert g.log_odds.max() == pytest.approx(g.l_max)
    assert g.log_odds.min() == pytest.approx(g.l_min)
    assert g.log_odds.max() <= 4.0 and g.log_odds.min() >= -4.0


def test_dedupe_one_update_per_cell_per_scan():
    g = box_grid()
    pose = Pose2D(2.0, 1.5, 0.0)
    # 2000 beams in a tiny fan all hit the same wall cells and cross the same free cells many times
    n = 2000
    ang = np.linspace(-0.001, 0.001, n)
    scan = Scan(0.0, 0, ang, np.full(n, 1.0), np.full(n, 47))
    g.integrate(scan, pose)
    assert g.log_odds.max() == pytest.approx(g.l_occ)
    assert g.log_odds.min() == pytest.approx(g.l_free)
    vals = np.unique(g.log_odds)
    allowed = np.array([0.0, g.l_occ, g.l_free], dtype=np.float32)
    assert np.all(np.isin(vals, allowed))


def test_endpoint_cell_not_freed_and_beyond_max_range_skipped():
    g = box_grid()
    pose = Pose2D(2.0, 1.5, 0.0)
    scan = Scan(0.0, 0, np.array([0.0, math.pi]), np.array([1.0, 1.0]), np.array([47, 47]))
    g.integrate(scan, pose, max_range_m=0.5)
    assert np.all(g.log_odds == 0)
    g.integrate(scan, pose)
    end = g.world_to_cell(np.array([[3.0, 1.5]]))[0]
    assert g.log_odds[end[1], end[0]] == pytest.approx(g.l_occ)
    before = g.world_to_cell(np.array([[2.9, 1.5]]))[0]
    assert g.log_odds[before[1], before[0]] == pytest.approx(g.l_free)
    robot = g.world_to_cell(np.array([[2.0, 1.5]]))[0]
    assert g.log_odds[robot[1], robot[0]] == pytest.approx(g.l_free)


def test_out_of_grid_endpoints_do_not_raise():
    g = OccupancyGrid(20, 20, 0.05, origin_xy=(0.0, 0.0))  # 1 m x 1 m
    scan, _ = box_scan()
    g.integrate(scan, Pose2D(0.5, 0.5, 0.0))  # every endpoint lies outside the grid
    assert np.all(g.log_odds <= 0)
    assert np.any(g.log_odds < 0)
    # robot itself outside the grid too
    g2 = OccupancyGrid(10, 10, 0.05, origin_xy=(100.0, 100.0))
    g2.integrate(scan, Pose2D(2.0, 1.5, 0.0))
    assert np.all(g2.log_odds == 0)


def test_empty_scan_ok():
    g = box_grid()
    scan = Scan(0.0, 0, np.zeros(0), np.zeros(0), np.zeros(0))
    g.integrate(scan, Pose2D(2.0, 1.5, 0.0))
    assert np.all(g.log_odds == 0)


def test_integrate_timing_450_beams_400x400():
    g = OccupancyGrid.around((0.0, 0.0), 20.0, 0.05)
    assert g.log_odds.shape == (400, 400)
    room = Room.box(16.0, 12.0)
    pose = Pose2D(8.0, 6.0, 0.0)
    g = OccupancyGrid.around((8.0, 6.0), 20.0, 0.05)
    scan = simulate_scan(room, pose, n_beams=450, rng=np.random.default_rng(1))
    g.integrate(scan, pose)  # warm-up
    times = []
    for _ in range(20):
        t0 = time.perf_counter()
        g.integrate(scan, pose)
        times.append(time.perf_counter() - t0)
    med_ms = 1000 * float(np.median(times))
    print(f"\nintegrate 450 beams into 400x400: median {med_ms:.2f} ms, max {1000 * max(times):.2f} ms")
    assert med_ms < 100.0  # generous CI bound; target is < 20 ms on a laptop CPU
