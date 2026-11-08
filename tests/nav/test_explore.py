"""Frontier exploration tests (synthetic rooms and scans only)."""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from locatebot.lidar.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from locatebot.lidar.mapping import demo_room
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig
from locatebot.nav.costmap import build_costmap
from locatebot.nav.explore import (
    ExploreConfig,
    FrontierExplorer,
    cluster_frontiers,
    explored_fraction,
    find_frontiers,
    frontier_mask,
)
from locatebot.nav.planner import can_turn_around, path_clear, plan
from locatebot.nav.pursuit import PurePursuit, PursuitStatus
from locatebot.nav.sim import BicycleSim

from .helpers import grid_from_classes, rasterise_room, segment_distance

VEH = VehicleConfig()


def test_frontier_mask_free_next_to_unknown():
    c = np.full((5, 6), UNKNOWN, dtype=np.uint8)
    c[1:4, 1:3] = FREE
    c[2, 3] = OCCUPIED
    m = frontier_mask(c, occ_clearance_cells=0)
    assert m.dtype == bool
    # every FREE cell here touches UNKNOWN (8-neighbourhood) except none -> all frontier
    assert m[1:4, 1:3].all()
    assert not m[c != FREE].any()
    c2 = np.full((5, 5), FREE, dtype=np.uint8)
    assert not frontier_mask(c2).any()  # the grid edge is not a frontier


def test_frontier_ignores_small_holes_in_walls():
    c = np.full((20, 20), FREE, dtype=np.uint8)
    c[10, :] = OCCUPIED
    c[11:, :] = UNKNOWN          # unknown behind the wall
    c[10, 5:7] = UNKNOWN         # 2-cell hole missed at a grazing angle
    c[10, 12:18] = UNKNOWN       # 6-cell opening: a real frontier
    m = frontier_mask(c)
    assert not m[:, 2:9].any()
    assert m[9, 14:16].all()


def test_cluster_frontiers_splits_components():
    m = np.zeros((10, 10), dtype=bool)
    m[1, 1:5] = True
    m[7:9, 7] = True
    clusters = cluster_frontiers(m)
    sizes = sorted(len(c) for c in clusters)
    assert sizes == [2, 4]
    assert all(c.shape[1] == 2 for c in clusters)  # (col, row)


def test_explored_fraction_edges():
    g = OccupancyGrid(40, 40, 0.05)
    assert explored_fraction(g) == 0.0
    g.log_odds[5, 5] = 3.0  # an occupied cell but nothing known free: still no estimate
    assert explored_fraction(g) == 0.0
    room = Room.box(1.5, 1.5)
    g = OccupancyGrid(40, 40, 0.05, origin_xy=(-0.25, -0.25))
    rasterise_room(room, g, (0, 0, 1.5, 1.5))
    assert explored_fraction(g) == 1.0  # closed, fully known room: outside unknown not counted
    # Open one side of the free area to unknown: fraction drops below 1.
    g.log_odds[20:30, 15:25] = 0.0
    f = explored_fraction(g)
    assert 0.0 < f < 1.0


def test_find_frontiers_reachable_goals_and_none_when_closed():
    room = Room.box(4.0, 3.0)
    g = OccupancyGrid(100, 80, 0.05, origin_xy=(-0.5, -0.5))
    rasterise_room(room, g, (0, 0, 4, 3))
    cm = build_costmap(g, VEH)
    assert find_frontiers(cm, (1.0, 1.0)) == []
    assert FrontierExplorer().next_goal(cm, Pose2D(1.0, 1.0, 0.0)) is None
    # Make the right half unknown: one frontier, its goal is reachable and traversable.
    g.log_odds[:, 50:] = 0.0
    cm = build_costmap(g, VEH)
    fr = find_frontiers(cm, (1.0, 1.0))
    assert len(fr) >= 1
    best = fr[0]
    assert cm.is_traversable_xy(best.goal_xy)
    assert best.goal_xy[0] > 1.5
    assert plan(cm, (1.0, 1.0), best.goal_xy) is not None


def test_unreachable_frontier_is_ignored():
    # Two free areas separated by a wall; the far one has a frontier, the near one is closed.
    n = 80
    free = np.zeros((n, n), dtype=bool)
    occ = np.zeros_like(free)
    free[5:35, 5:75] = True
    occ[4, 4:76] = occ[35, 4:76] = occ[4:36, 4] = occ[4:36, 75] = True
    free[45:70, 10:70] = True  # second area, open to unknown on its edges
    free &= ~occ
    g = grid_from_classes(free, occ)
    cm = build_costmap(g, VEH)
    assert find_frontiers(cm, (1.0, 1.0)) == []


def run_exploration(room, start: Pose2D, max_goals: int = 80, rmax: float = 12.0, verbose: bool = False) -> dict:
    """Map ``room`` from scratch (ground-truth poses, simulated scans) while driving frontier
    goals with planner + pure pursuit + bicycle model. Mirrors what a skill executor must do:
    re-check the path ahead against the growing map at 2 Hz and replan when it is blocked or the
    follower asks; stop early only where the car can still turn around; give up on a goal after
    a few failed attempts. ``max_goals`` counts goals considered, including ones refused at once
    (no path from here), not only goals driven to."""
    grid = OccupancyGrid.around((4.0, 3.0), 9.0, 0.05)
    rng = np.random.default_rng(3)
    sim = BicycleSim(VEH, start, dt=0.1)

    def sense():
        # RPLIDAR C1: ~508 beams per revolution, ~370 valid in a cluttered room (~27 % dropout).
        s = simulate_scan(room, sim.pose, n_beams=508, range_max_m=rmax, rng=rng, dropout=0.27)
        grid.integrate(s, sim.pose, max_range_m=rmax)

    for _ in range(3):  # the car waits for a few revolutions before planning (10 Hz LiDAR)
        sense()
    out = {"f0": explored_fraction(grid), "goals": 0, "finished": False, "min_clear": math.inf,
           "lethal": 0, "distance_m": 0.0, "stuck": False}
    explorer = FrontierExplorer(ExploreConfig())
    pp = PurePursuit(VEH)
    for _ in range(max_goals):
        cm = build_costmap(grid, VEH)
        fr = explorer.next_goal(cm, sim.pose)
        if fr is None:
            out["finished"] = True
            break
        out["goals"] += 1
        status, detail = PursuitStatus.NEEDS_REPLAN, "no path"
        for _attempt in range(6):
            cm = build_costmap(grid, VEH)
            path = plan(cm, (sim.pose.x, sim.pose.y), fr.goal_xy, start_theta=sim.pose.theta)
            if path is None:
                status, detail = PursuitStatus.NEEDS_REPLAN, "no path"
                if not can_turn_around(cm, (sim.pose.x, sim.pose.y), sim.pose.theta):
                    out["stuck"] = True
                    out["stuck_at"] = (sim.pose, cm, fr)
                break
            pp.set_path(path)
            if verbose:
                print("   plan", (round(sim.pose.x, 2), round(sim.pose.y, 2), round(sim.pose.theta, 2)), "->",
                      tuple(round(v, 2) for v in path[-1]), path.detail, "trim", round(path.trimmed_m, 2))
            for step in range(400):
                cmd = pp.update(sim.pose, speed_mps=sim.speed_mps)
                status, detail = cmd.status, cmd.detail
                if status is not PursuitStatus.FOLLOWING:
                    break
                sim.step(cmd.throttle, cmd.steering)
                out["min_clear"] = min(out["min_clear"], segment_distance(room, sim.pose.x, sim.pose.y))
                c, r = cm.world_to_cell((sim.pose.x, sim.pose.y))
                out["lethal"] += int(cm.occupied[r, c])
                sense()  # one revolution per 0.1 s step
                if step % 5 == 4:  # the executor re-checks the path ahead at 2 Hz
                    cm = build_costmap(grid, VEH)
                    if not explorer.goal_still_useful(cm, fr) and can_turn_around(
                            cm, (sim.pose.x, sim.pose.y), sim.pose.theta):
                        detail = "goal resolved"  # stop early only where the car can turn round
                        break
                    if not path_clear(cm, pp.remaining_path()):
                        status, detail = PursuitStatus.NEEDS_REPLAN, "path blocked"
                        break
            if verbose:
                print("   ->", status.value, detail, (round(sim.pose.x, 2), round(sim.pose.y, 2), round(sim.pose.theta, 2)))
            if status is not PursuitStatus.NEEDS_REPLAN:
                break
        if out["stuck"]:
            break
        if verbose:
            print(out["goals"], fr.goal_xy, status.value, detail, sim.pose, round(explored_fraction(grid), 3))
        if status is PursuitStatus.REACHED or detail == "goal resolved":
            explorer.mark_visited(fr.goal_xy)
        else:
            explorer.mark_failed(fr.goal_xy, from_xy=(sim.pose.x, sim.pose.y))
    out["f1"] = explored_fraction(grid)
    out["distance_m"] = sim.distance_m
    out["classes"] = grid.classify()
    return out


TWO_ROOMS = (Room.box(8.0, 6.0).with_segment(4.0, 0.0, 4.0, 4.5)   # dividing wall, 1.5 m door at the top
             .with_box(1.5, 3.5, 2.5, 4.5).with_box(5.5, 1.0, 6.5, 2.0))


@pytest.mark.parametrize("room,start", [
    (demo_room(), Pose2D(1.0, 1.0, 0.4)),
    (demo_room(), Pose2D(2.0, 1.5, 0.6)),
    (TWO_ROOMS, Pose2D(1.0, 1.0, 0.4)),
    (TWO_ROOMS, Pose2D(1.5, 2.5, math.pi / 2)),
], ids=["demo-a", "demo-b", "two-rooms-a", "two-rooms-b"])
def test_exploration_closed_loop(room, start):
    """Explore from scratch: the map must grow substantially, the car must never touch an
    obstacle, never get stuck (it cannot reverse) and the run must terminate."""
    t0 = time.monotonic()
    out = run_exploration(room, start)
    assert out["finished"], "exploration did not terminate"
    assert not out["stuck"]
    assert out["f0"] < 0.5
    assert out["f1"] > 0.95
    assert out["goals"] >= 3
    assert out["lethal"] == 0
    assert out["min_clear"] >= VEH.half_width_m
    # Hang guard only: about 3-4 s on an x86 laptop, about 10-11 s on the Orin Nano CPU (2026-11-08).
    assert time.monotonic() - t0 < 60.0


def test_explorer_failed_goal_retried_from_elsewhere():
    ex = FrontierExplorer(retry_distance_m=1.0, max_failures=2)
    ex.mark_failed((5.0, 5.0), from_xy=(0.0, 0.0))
    assert (5.0, 5.0) in ex.excluded(Pose2D(0.5, 0.0, 0.0))      # same place: skip it
    assert (5.0, 5.0) not in ex.excluded(Pose2D(3.0, 0.0, 0.0))  # moved: try again
    ex.mark_failed((5.1, 5.0), from_xy=(3.0, 0.0))
    assert (5.0, 5.0) in ex.excluded(Pose2D(9.0, 9.0, 0.0))      # failed twice: give up
    ex.mark_visited((1.0, 1.0))
    assert (1.0, 1.0) in ex.excluded(Pose2D(9.0, 9.0, 0.0))
