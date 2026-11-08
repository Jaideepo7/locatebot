"""Closed loop: planner + pure pursuit + kinematic bicycle model in a synthetic room.

Synthetic: proves software behaviour, not real-car behaviour or physical safety.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig
from locatebot.nav.costmap import build_costmap
from locatebot.nav.planner import plan, plan_approach
from locatebot.nav.pursuit import PurePursuit, PursuitStatus
from locatebot.nav.sim import BicycleSim

from .helpers import segment_distance

VEH = VehicleConfig()
ROOM = Room.box(6.0, 4.0).with_box(2.5, 1.5, 3.5, 2.5)


def mapped_grid() -> OccupancyGrid:
    """Map built from simulated scans at ground-truth poses (as the mapper would)."""
    g = OccupancyGrid.around((3.0, 2.0), 7.0, 0.05)
    rng = np.random.default_rng(1)
    for x, y in [(1, 1), (5, 1), (5, 3), (1, 3), (3, 0.7), (3, 3.3)]:
        pose = Pose2D(x, y, 0.0)
        for _ in range(3):
            g.integrate(simulate_scan(ROOM, pose, n_beams=450, rng=rng), pose)
    return g


def drive(cm, path, start: Pose2D, max_steps=1500, dt=0.05):
    pp = PurePursuit(VEH)
    pp.set_path(path)
    sim = BicycleSim(VEH, start, dt=dt)
    min_clear = math.inf
    status = PursuitStatus.FOLLOWING
    for _ in range(max_steps):
        cmd = pp.update(sim.pose, speed_mps=sim.speed_mps)
        status = cmd.status
        if status is not PursuitStatus.FOLLOWING:
            break
        sim.step(cmd.throttle, cmd.steering)
        c, r = cm.world_to_cell((sim.pose.x, sim.pose.y))
        assert not cm.lethal[r, c], f"entered lethal cell at {sim.pose}"
        assert not cm.occupied[r, c]
        min_clear = min(min_clear, segment_distance(ROOM, sim.pose.x, sim.pose.y))
    return sim.pose, status, min_clear


@pytest.fixture(scope="module")
def cm():
    return build_costmap(mapped_grid(), VEH)


@pytest.mark.parametrize("start,goal", [
    (Pose2D(1.0, 1.0, 0.0), (5.0, 3.0)),
    (Pose2D(5.0, 1.0, 0.0), (1.0, 3.0)),        # starts facing away from the goal
    (Pose2D(1.0, 3.0, -math.pi / 2), (5.0, 3.0)),
])
def test_reaches_goal_without_touching_obstacles(cm, start, goal):
    path = plan(cm, (start.x, start.y), goal, start_theta=start.theta)
    assert path is not None
    # The end may be pulled back (ensure_exit) so the car can turn around there.
    assert math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) <= path.trimmed_m + 1e-6
    assert path.trimmed_m <= 1.0
    end, status, min_clear = drive(cm, path, start)
    assert status is PursuitStatus.REACHED
    assert math.hypot(end.x - path[-1][0], end.y - path[-1][1]) < 0.3
    assert min_clear >= VEH.half_width_m


def test_approach_item_on_box(cm):
    item = (3.0, 2.0)  # centre of the 1 x 1 m box: not reachable itself
    start = Pose2D(1.0, 1.0, 0.0)
    # 1.2 m from the centre = 0.7 m from the box face: the closest a forward-only car can stop
    # here and still turn around with margin in the 1.5 m gaps (the camera sees it from there).
    assert plan_approach(cm, (start.x, start.y), item, standoff_m=1.2, start_theta=start.theta,
                         ensure_exit=False) is not None
    path = plan_approach(cm, (start.x, start.y), item, standoff_m=1.2, start_theta=start.theta)
    assert path is not None
    end, status, min_clear = drive(cm, path, start)
    assert status is PursuitStatus.REACHED
    assert math.hypot(path[-1][0] - item[0], path[-1][1] - item[1]) <= 1.2 + 1e-9
    assert math.hypot(end.x - item[0], end.y - item[1]) < 1.2 + 0.3
    assert min_clear >= VEH.half_width_m
