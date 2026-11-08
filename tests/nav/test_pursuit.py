"""Pure-pursuit and bicycle-model tests (no hardware)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig
from locatebot.nav.pursuit import PurePursuit, PursuitConfig, PursuitStatus
from locatebot.nav.sim import BicycleSim

VEH = VehicleConfig()


def line(x0, y0, x1, y1, step=0.1):
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def arc(cx, cy, r, a0, a1, step=0.05):
    n = max(2, int(abs(a1 - a0) * r / step))
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def test_straight_path_ahead():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 3, 0))
    cmd = pp.update(Pose2D(0, 0, 0))
    assert cmd.status is PursuitStatus.FOLLOWING
    assert cmd.steering == pytest.approx(0.0, abs=1e-9)
    assert 0 < cmd.throttle <= VEH.max_throttle


@pytest.mark.parametrize("side", [1, -1])
def test_steering_sign_follows_path_side(side):
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 1, 0) + line(1, 0, 1, side * 2)[1:])
    cmd = pp.update(Pose2D(0.8, 0, 0))
    assert math.copysign(1, cmd.steering) == side
    assert abs(cmd.steering) <= 1.0


def test_steering_and_throttle_limits():
    veh = VehicleConfig(max_throttle=0.3)
    pp = PurePursuit(veh)
    pp.set_path(line(0, 0, 0.05, 0.6))  # nearly sideways: needs more than full lock
    cmd = pp.update(Pose2D(0, 0, 0))
    assert cmd.steering == pytest.approx(1.0)
    pp.set_path(line(0, 0, 10, 0))
    for v in (0.0, 0.2, 0.4, 5.0):
        c = pp.update(Pose2D(0, 0, 0), speed_mps=v)
        assert 0.0 <= c.throttle <= 0.3 + 1e-12
        assert -1.0 <= c.steering <= 1.0


def test_reached_at_goal():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 2, 0))
    cmd = pp.update(Pose2D(1.95, 0.02, 0))
    assert cmd.status is PursuitStatus.REACHED
    assert cmd.throttle == 0.0


def test_overshoot_close_to_goal_counts_as_reached():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 2, 0))
    pp.update(Pose2D(1.5, 0, 0))
    cmd = pp.update(Pose2D(2.2, 0.05, 0))
    assert cmd.status is PursuitStatus.REACHED


def test_target_behind_needs_replan():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, -2, 0))
    cmd = pp.update(Pose2D(0, 0, 0))
    assert cmd.status is PursuitStatus.NEEDS_REPLAN
    assert cmd.throttle == 0.0
    assert "behind" in cmd.detail


def test_off_path_needs_replan():
    pp = PurePursuit(VEH, PursuitConfig(max_cross_track_m=0.4))
    pp.set_path(line(0, 0, 3, 0))
    cmd = pp.update(Pose2D(1.0, 0.8, 0))
    assert cmd.status is PursuitStatus.NEEDS_REPLAN
    assert cmd.throttle == 0.0


def test_empty_path_needs_replan():
    pp = PurePursuit(VEH)
    assert pp.update(Pose2D(0, 0, 0)).status is PursuitStatus.NEEDS_REPLAN


def test_slows_near_goal_and_in_curves():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 4, 0))
    far = pp.update(Pose2D(0.5, 0, 0)).throttle
    near = pp.update(Pose2D(3.7, 0, 0)).throttle
    assert near < far
    pp2 = PurePursuit(VEH)
    pp2.set_path(arc(0, 0.6, 0.6, -math.pi / 2, math.pi / 2) + line(0, 1.2, -0.1, 1.2)[1:])
    pp2.update(Pose2D(0, 0, 0))
    curve = pp2.update(Pose2D(0.3, 0.1, 0.6))
    assert curve.throttle < far


def test_slows_with_low_clearance():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 4, 0))
    open_ = pp.update(Pose2D(0.5, 0, 0), clearance_m=5.0).throttle
    tight = pp.update(Pose2D(0.5, 0, 0), clearance_m=0.25).throttle
    assert tight < open_


def test_lookahead_grows_with_speed():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 5, 0))
    slow = pp.update(Pose2D(0, 0, 0), speed_mps=0.0).lookahead_m
    fast = pp.update(Pose2D(0, 0, 0), speed_mps=0.4).lookahead_m
    assert fast > slow


# --- bicycle model ---------------------------------------------------------------------------
def test_bicycle_straight_and_no_reverse():
    sim = BicycleSim(VEH, Pose2D(0, 0, 0), dt=0.1)
    for _ in range(10):
        sim.step(VEH.max_throttle, 0.0)
    assert sim.pose.x == pytest.approx(VEH.max_speed_mps * 1.0, rel=1e-6)
    assert sim.pose.y == pytest.approx(0.0, abs=1e-9)
    p = sim.pose
    sim.step(-1.0, 0.0)
    assert sim.speed_mps == 0.0 and sim.pose == p
    sim.step(5.0, 0.0)
    assert sim.speed_mps == pytest.approx(VEH.max_speed_mps)


def test_bicycle_full_lock_turns_on_min_radius():
    sim = BicycleSim(VEH, Pose2D(0, 0, 0), dt=0.02)
    pts = []
    for _ in range(400):
        sim.step(VEH.max_throttle, 1.0)
        pts.append((sim.pose.x, sim.pose.y))
    p = np.array(pts)
    r = np.hypot(p[:, 0], p[:, 1] - VEH.min_turn_radius_m)  # centre on the left
    np.testing.assert_allclose(r, VEH.min_turn_radius_m, atol=1e-6)


def test_pursuit_drives_bicycle_round_a_curve():
    path = line(0, 0, 1.5, 0) + arc(1.5, 1.0, 1.0, -math.pi / 2, 0)[1:] + line(2.5, 1.0, 2.5, 3.0)[1:]
    pp = PurePursuit(VEH)
    pp.set_path(path)
    sim = BicycleSim(VEH, Pose2D(0, 0, 0), dt=0.05)
    P = np.array(path)
    status = None
    for _ in range(600):
        cmd = pp.update(sim.pose, speed_mps=sim.speed_mps)
        status = cmd.status
        if status is not PursuitStatus.FOLLOWING:
            break
        sim.step(cmd.throttle, cmd.steering)
        assert np.min(np.hypot(P[:, 0] - sim.pose.x, P[:, 1] - sim.pose.y)) < 0.15
    assert status is PursuitStatus.REACHED
    assert math.hypot(sim.pose.x - 2.5, sim.pose.y - 3.0) < 0.2


def test_remaining_path_starts_at_projection():
    pp = PurePursuit(VEH)
    pp.set_path(line(0, 0, 3, 0))
    assert pp.remaining_path()[0] == (0.0, 0.0)
    pp.update(Pose2D(1.25, 0.05, 0))
    rest = pp.remaining_path()
    assert rest[0][0] == pytest.approx(1.25)
    assert rest[-1] == (3.0, 0.0)
    assert pp.progress_m == pytest.approx(1.25)
