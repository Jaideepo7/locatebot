"""A* planner tests (synthetic grids only)."""
from __future__ import annotations

import math

import numpy as np

from locatebot.mission.types import VehicleConfig
from locatebot.nav.costmap import CostmapConfig, build_costmap
from locatebot.nav.planner import astar, path_length, plan, plan_approach

from .helpers import RES, grid_from_classes, open_grid

VEH = VehicleConfig()


def _wall_with_gap(n: int = 80, gap=(55, 70)):
    """Open n x n grid with a vertical wall at col 40 except rows gap[0]..gap[1]."""
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[:, 40] = True
    occ[gap[0]:gap[1], 40] = False
    free &= ~occ
    return grid_from_classes(free, occ)


def _all_traversable(cm, path, step=RES / 4):
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for t in np.linspace(0, 1, n + 1):
            if not cm.is_traversable_xy((x0 + t * (x1 - x0), y0 + t * (y1 - y0))):
                return False
    return True


def test_astar_optimal_octile_on_open_grid():
    cm = build_costmap(open_grid(50, 50), VEH, CostmapConfig(inflate_unknown=False, soft_weight=0.0))
    res = astar(cm, (5, 5), (35, 15))
    assert res is not None
    cells, cost = res
    assert cells[0] == (5, 5) and cells[-1] == (35, 15)
    dx, dy = 30, 10
    octile = (max(dx, dy) - min(dx, dy) + math.sqrt(2) * min(dx, dy)) * RES
    assert math.isclose(cost, octile, rel_tol=1e-9)


def test_plan_straight_line_and_even_spacing():
    cm = build_costmap(open_grid(60, 60), VEH, CostmapConfig(inflate_unknown=False, soft_weight=0.0))
    path = plan(cm, (0.5, 0.5), (2.5, 1.5), spacing_m=0.1)
    assert path is not None
    assert path[0] == (0.5, 0.5) and path[-1] == (2.5, 1.5)
    assert math.isclose(path_length(path), math.hypot(2.0, 1.0), rel_tol=0.02)
    steps = np.hypot(*np.diff(np.array(path), axis=0).T)
    assert np.all(steps <= 0.1 + 1e-9)
    assert np.all(steps[:-1] >= 0.1 - 1e-6)


def test_plan_goes_through_gap_with_clearance():
    g = _wall_with_gap()
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    path = plan(cm, (1.0, 1.0), (3.0, 1.0))
    assert path is not None
    assert _all_traversable(cm, path)
    ys = [y for x, y in path if abs(x - 40.5 * RES) < 0.05]
    assert ys and all(55 * RES < y < 70 * RES for y in ys)
    assert path_length(path) > 2.0


def test_soft_cost_keeps_path_off_the_wall():
    # Corridor wide enough to hug either side; the soft cost should centre the path.
    n = 80
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[20, :] = True
    occ[44, :] = True  # corridor rows 21..43 (1.15 m)
    free &= ~occ
    g = grid_from_classes(free, occ)
    # Start and goal 0.275 m above the lower wall: without soft cost the path hugs it.
    hard_cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False, soft_weight=0.0))
    soft_cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False, soft_weight=8.0, soft_range_m=0.5))
    hug = plan(hard_cm, (0.5, 1.3), (3.5, 1.3), ensure_exit=False)
    soft = plan(soft_cm, (0.5, 1.3), (3.5, 1.3), ensure_exit=False)
    assert hug is not None and soft is not None

    def mid_clearance(path):
        return min(soft_cm.clearance_at(p) for p in path if 1.5 < p[0] < 2.5)

    assert mid_clearance(soft) > mid_clearance(hug) + 0.1


def test_no_path_through_unknown():
    n = 60
    free = np.ones((n, n), dtype=bool)
    free[:, 28:32] = False  # unknown band splits the map
    g = grid_from_classes(free, np.zeros_like(free))
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    assert plan(cm, (0.5, 1.5), (2.5, 1.5)) is None


def test_goal_inside_obstacle_needs_approach():
    n = 80
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[38:42, 58:62] = True  # a small box centred near (3.0, 2.0)
    free &= ~occ
    g = grid_from_classes(free, occ)
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    goal = (3.0, 2.0)
    assert plan(cm, (0.5, 0.5), goal) is None
    path = plan_approach(cm, (0.5, 0.5), goal, standoff_m=0.5, ensure_exit=False)
    assert path is not None
    end = path[-1]
    assert math.hypot(end[0] - goal[0], end[1] - goal[1]) <= 0.5 + 1e-9
    assert cm.is_traversable_xy(end)
    assert _all_traversable(cm, path)
    # nearest reachable: as close as inflation allows (box edge at 0.1 m + inflation 0.2 m)
    assert math.hypot(end[0] - goal[0], end[1] - goal[1]) < 0.4
    # With the exit check the end may be pulled back, but never beyond the standoff.
    safe = plan_approach(cm, (0.5, 0.5), goal, standoff_m=0.5)
    assert safe is not None
    assert math.hypot(safe[-1][0] - goal[0], safe[-1][1] - goal[1]) <= 0.5 + 1e-9


def test_approach_unreachable_returns_none():
    n = 60
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[:, 30] = True
    free &= ~occ
    g = grid_from_classes(free, occ)
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    assert plan_approach(cm, (0.3, 1.5), (2.5, 1.5), standoff_m=0.4) is None


def test_start_inside_inflation_escapes():
    n = 60
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[:, 10] = True
    free &= ~occ
    g = grid_from_classes(free, occ)
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    start = (11.5 * RES + 0.05, 1.5)  # 0.1 m from the wall: inside inflation
    assert not cm.is_traversable_xy(start)
    path = plan(cm, start, (2.5, 1.5))
    assert path is not None
    assert path[0] == start
    assert _all_traversable(cm, path[2:])
    assert plan(cm, start, (2.5, 1.5), allow_escape=False) is None


def test_start_inside_obstacle_does_not_escape():
    n = 60
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[:, 10] = True
    free &= ~occ
    g = grid_from_classes(free, occ)
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    assert plan(cm, (10.5 * RES, 1.5), (2.5, 1.5)) is None


def test_heading_aware_start_goes_forward_first():
    cm = build_costmap(open_grid(100, 100), VEH, CostmapConfig())
    start, goal = (2.5, 2.5), (1.0, 2.5)  # goal behind a car facing +x
    path = plan(cm, start, goal, start_theta=0.0)
    assert path is not None
    p = np.array(path)
    first = p[min(3, len(p) - 1)] - p[0]
    assert first[0] > 0  # leaves forwards, not backwards
    assert _all_traversable(cm, path)
    assert path[-1] == goal


# --- forward-only car: dead ends, tracking, re-checks ----------------------------------------
from locatebot.lidar.types import Pose2D  # noqa: E402
from locatebot.nav.planner import can_turn_around, check_tracking, has_forward_exit, path_clear  # noqa: E402


def _slot_grid():
    """Open 4 x 4 m area with a 0.6 m wide, 2 m deep dead-end slot going up from y = 2."""
    n = 80
    free = np.ones((n, n), dtype=bool)
    occ = np.zeros_like(free)
    occ[40, :] = True            # wall at y = 2.0 ...
    occ[40, 34:46] = False       # ... with a 0.6 m opening, x in [1.7, 2.3]
    occ[40:, 33] = True          # slot side walls
    occ[40:, 46] = True
    occ[79, 33:47] = True        # slot end
    free &= ~occ
    free[41:, :33] = False       # unknown outside the slot
    free[41:, 47:] = False
    return grid_from_classes(free, occ)


def test_can_turn_around_open_space_but_not_in_a_slot():
    cm = build_costmap(_slot_grid(), VEH, CostmapConfig(inflate_unknown=False))
    assert can_turn_around(cm, (2.0, 1.0), 0.0)
    assert not can_turn_around(cm, (2.0, 2.8), math.pi / 2)  # inside the slot, facing its end
    assert has_forward_exit(cm, (2.0, 2.4), math.pi / 2)     # straight on is free for a while...
    assert not can_turn_around(cm, (2.0, 2.4), math.pi / 2)  # ...but it is still a dead end


def test_plan_refuses_to_park_in_a_dead_end():
    cm = build_costmap(_slot_grid(), VEH, CostmapConfig(inflate_unknown=False))
    goal = (2.0, 3.5)  # 1.5 m into the slot
    assert cm.is_traversable_xy(goal)
    assert plan(cm, (1.0, 1.0), goal, start_theta=0.0, ensure_exit=False) is not None
    assert plan(cm, (1.0, 1.0), goal, start_theta=0.0) is None  # end would be a trap


def test_check_tracking_rejects_corner_too_tight_for_the_car():
    # L-shaped 0.5 m wide corridor: the grid path turns 90 degrees, the car cannot.
    n = 80
    free = np.zeros((n, n), dtype=bool)
    free[10:20, 5:60] = True     # horizontal leg, y in [0.5, 1.0]
    free[10:70, 50:60] = True    # vertical leg, x in [2.5, 3.0]
    occ = np.zeros_like(free)
    g = grid_from_classes(free, occ)
    cm = build_costmap(g, VEH, CostmapConfig(margin_m=0.0, soft_weight=0.0))
    path = plan(cm, (0.6, 0.75), (2.75, 3.0), validate=False, ensure_exit=False)
    assert path is not None
    ok, fail_xy, detail = check_tracking(cm, path, Pose2D(0.6, 0.75, 0.0))
    assert not ok and fail_xy is not None and detail
    assert plan(cm, (0.6, 0.75), (2.75, 3.0), start_theta=0.0, ensure_exit=False) is None


def test_check_tracking_accepts_easy_path():
    cm = build_costmap(open_grid(100, 100), VEH)
    path = plan(cm, (1.0, 1.0), (4.0, 3.0), start_theta=0.0)
    assert path is not None
    assert check_tracking(cm, path, Pose2D(1.0, 1.0, 0.0))[0]


def test_path_clear_detects_new_obstacle():
    g = open_grid(80, 40)
    cm = build_costmap(g, VEH, CostmapConfig(inflate_unknown=False))
    path = plan(cm, (0.5, 1.0), (3.5, 1.0), ensure_exit=False)
    assert path_clear(cm, path)
    g.log_odds[18:22, 40:42] = 3.0  # a box appears on the path
    assert not path_clear(build_costmap(g, VEH, CostmapConfig(inflate_unknown=False)), path)


def test_planned_path_metadata():
    cm = build_costmap(open_grid(100, 100), VEH)
    path = plan(cm, (1.0, 1.0), (4.0, 3.0), start_theta=0.0)
    assert isinstance(path, list)
    assert path.goal_xy == (4.0, 3.0)
    assert path.trimmed_m == 0.0
