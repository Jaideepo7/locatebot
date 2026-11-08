"""Costmap tests (synthetic grids only)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from locatebot.lidar.grid import FREE, OCCUPIED, UNKNOWN
from locatebot.mission.types import VehicleConfig
from locatebot.nav.costmap import CostmapConfig, build_costmap, costmap_from_classes

from .helpers import RES, grid_from_classes, open_grid

VEH = VehicleConfig()


def _point_obstacle_grid(n: int = 61):
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[n // 2, n // 2] = True
    free &= ~occ
    return grid_from_classes(free, occ)


def test_open_grid_all_traversable_zero_cost():
    cm = build_costmap(open_grid(40, 30), VEH, CostmapConfig(inflate_unknown=False))
    assert cm.traversable.shape == (30, 40)
    assert cm.traversable.all()
    assert np.all(cm.cost == 0.0)


def test_border_counts_as_unknown_when_inflating_unknown():
    cm = build_costmap(open_grid(40, 40), VEH)
    r = int(math.ceil((VEH.half_width_m + CostmapConfig().margin_m) / RES))
    assert not cm.traversable[0, 20] and not cm.traversable[20, 0]
    assert cm.traversable[20, 20]
    assert not cm.traversable[20, r - 1]


def test_unknown_is_never_traversable_even_without_unknown_inflation():
    free = np.ones((40, 40), dtype=bool)
    free[:, 20:] = False  # right half unknown
    g = grid_from_classes(free, np.zeros_like(free))
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    assert not cm.traversable[:, 20:].any()
    assert cm.traversable[:, :20].all()


def test_occupied_inflation_radius():
    g = _point_obstacle_grid()
    cfg = CostmapConfig(margin_m=0.05, inflate_unknown=False)
    cm = build_costmap(g, VEH, cfg)
    radius = VEH.half_width_m + cfg.margin_m
    c = 30
    rows, cols = np.mgrid[0:61, 0:61]
    d = np.hypot(rows - c, cols - c) * RES
    assert not cm.traversable[d <= radius - 1e-9].any()
    assert cm.traversable[d > radius + 1e-9].all()
    # lethal = the car body could overlap the obstacle (half a cell of discretisation slack)
    r_lethal = VEH.half_width_m + RES / 2
    assert cm.lethal[d < r_lethal - 1e-9].all()
    assert not cm.lethal[d >= r_lethal + 1e-9].any()


def test_unknown_inflation_configurable():
    free = np.ones((41, 41), dtype=bool)
    free[20, 20] = False  # single unknown cell
    g = grid_from_classes(free, np.zeros_like(free))
    off = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    on = build_costmap(g, VEH, CostmapConfig(inflate_unknown=True, unknown_inflation_m=0.1))
    assert not off.traversable[20, 20] and off.traversable[20, 21]
    assert not on.traversable[20, 21] and not on.traversable[20, 22]
    assert on.traversable[20, 23]


def test_clearance_is_exact_euclidean():
    g = _point_obstacle_grid()
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False, soft_range_m=0.5))
    rows, cols = np.mgrid[0:61, 0:61]
    d = np.hypot(rows - 30, cols - 30) * RES
    cap = cm.clearance_cap_m
    np.testing.assert_allclose(cm.clearance_m, np.minimum(d, cap), atol=1e-9)


def test_soft_cost_decreases_with_distance():
    g = _point_obstacle_grid()
    cfg = CostmapConfig(inflate_unknown=False, soft_range_m=0.4, soft_weight=5.0)
    cm = build_costmap(g, VEH, cfg)
    row = cm.cost[30, 30:]
    trav = cm.traversable[30, 30:]
    vals = row[trav]
    assert np.all(np.diff(vals) <= 1e-12)
    assert vals[0] > 0
    assert vals[-1] == 0.0
    assert np.all(np.isinf(row[~trav]))


def test_costmap_from_classes_and_coordinates():
    classes = np.full((10, 20), FREE, dtype=np.uint8)
    classes[0, 0] = OCCUPIED
    classes[9, 19] = UNKNOWN
    cm = costmap_from_classes(classes, 0.1, (1.0, 2.0), VEH, CostmapConfig(inflate_unknown=False))
    assert cm.world_to_cell((1.05, 2.05)) == (0, 0)
    assert cm.cell_to_world((3, 4)) == pytest.approx((1.35, 2.45))
    assert not cm.is_traversable_xy((1.05, 2.05))
    assert cm.is_traversable_xy((2.0, 2.5))
    assert not cm.is_traversable_xy((-5.0, 0.0))  # out of bounds
