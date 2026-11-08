"""Tests for locatebot.safety.supervisor.

All scans are synthetic (locatebot.lidar.sim). They prove the decision logic, not real
RPLIDAR behaviour, real braking distance or physical safety: the stop/slow distances
must still be measured on the car.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, Scan
from locatebot.safety.supervisor import DriveCommand, Reason, SafetyConfig, SafetySupervisor

NOW = 100.0
POSE = Pose2D(1.0, 3.0, 0.0)  # facing +x down a 12 m room
ROOM = Room.box(12.0, 6.0)
CFG = SafetyConfig(half_width_m=0.15, stop_distance_m=0.40, slow_distance_m=1.00, max_speed=0.6,
                   max_steer_rad=math.radians(30), min_coverage=0.9, coverage_fov_rad=math.radians(120),
                   scan_timeout_s=0.5, command_timeout_s=0.3)


def scan(room=ROOM, pose=POSE, stamp=NOW - 0.05, **kw) -> Scan:
    return simulate_scan(room, pose, n_beams=720, noise_std_m=0.0, stamp_s=stamp, **kw)


def cmd(throttle=0.3, steering=0.0, stamp=NOW - 0.05) -> DriveCommand:
    return DriveCommand(throttle=throttle, steering=steering, stamp_s=stamp)


@pytest.fixture
def sup():
    return SafetySupervisor(CFG)


def box_ahead(dist, half=0.1):
    return ROOM.with_box(POSE.x + dist, POSE.y - half, POSE.x + dist + 0.1, POSE.y + half)


# --- normal driving ----------------------------------------------------------

def test_clear_scan_allows_bounded_forward_command(sup):
    d = sup.evaluate(cmd(0.3, 0.2), scan(), NOW)
    assert d.reason is Reason.OK
    assert (d.throttle, d.steering) == (0.3, 0.2)
    assert d.clearance_m == pytest.approx(11.0, abs=0.05)
    assert d.coverage == pytest.approx(1.0)


def test_throttle_above_max_speed_is_clamped(sup):
    d = sup.evaluate(cmd(1.0), scan(), NOW)
    assert d.throttle == pytest.approx(0.6) and d.reason is Reason.OK


def test_obstacle_beside_corridor_does_not_stop_straight_driving(sup):
    room = ROOM.with_box(POSE.x + 0.2, POSE.y + 0.3, POSE.x + 0.5, POSE.y + 0.5)  # 0.3 m to the left
    d = sup.evaluate(cmd(0.3), scan(room), NOW)
    assert d.reason is Reason.OK and d.throttle == 0.3


# --- obstacles ---------------------------------------------------------------

def test_close_obstacle_stops(sup):
    d = sup.evaluate(cmd(0.3, 0.5), scan(box_ahead(0.3)), NOW)
    assert d.reason is Reason.OBSTACLE
    assert (d.throttle, d.steering) == (0.0, 0.0)
    # rotated (steering) corridor reaches the box's near corner a little sooner than 0.3 m
    assert 0.2 < d.clearance_m <= 0.31
    assert sup.evaluate(cmd(0.3, 0.0), scan(box_ahead(0.3)), NOW).clearance_m == pytest.approx(0.3, abs=0.01)


def _ring_with_point_ahead(r_ahead: float) -> Scan:
    angles = -math.pi + 2 * math.pi * np.arange(720) / 720
    ranges = np.full(720, 5.0)
    ranges[360] = r_ahead  # angles[360] == 0.0 exactly
    return Scan(NOW - 0.05, 0, angles, ranges, np.full(720, 47))


def test_obstacle_exactly_at_threshold_stops(sup):
    d = sup.evaluate(cmd(0.3), _ring_with_point_ahead(0.40), NOW)
    assert d.clearance_m == 0.40
    assert d.reason is Reason.OBSTACLE


def test_obstacle_just_past_threshold_does_not_stop(sup):
    d = sup.evaluate(cmd(0.3), _ring_with_point_ahead(0.41), NOW)
    assert d.reason is Reason.SLOWED and d.throttle > 0.0


def test_obstacle_in_slow_zone_scales_speed_down(sup):
    d = sup.evaluate(cmd(0.6), scan(box_ahead(0.70)), NOW)  # halfway between 0.4 and 1.0
    assert d.reason is Reason.SLOWED
    assert d.throttle == pytest.approx(0.3, abs=0.02)


def test_slow_zone_never_raises_requested_speed(sup):
    d = sup.evaluate(cmd(0.1), scan(box_ahead(0.70)), NOW)
    assert d.throttle == pytest.approx(0.1) and d.reason is Reason.OK


def test_turn_does_not_bypass_forward_obstacle(sup):
    # small post forward-left (~35 deg, ~0.4 m): outside the straight corridor
    room = ROOM.with_box(POSE.x + 0.30, POSE.y + 0.20, POSE.x + 0.36, POSE.y + 0.26)
    assert sup.evaluate(cmd(0.3, 0.0), scan(room), NOW).reason is Reason.OK
    d = sup.evaluate(cmd(0.3, 1.0), scan(room), NOW)  # full left turn into it
    assert d.reason is Reason.OBSTACLE and d.throttle == 0.0


def test_zero_throttle_with_obstacle_is_allowed_to_steer(sup):
    d = sup.evaluate(cmd(0.0, -0.4), scan(box_ahead(0.2)), NOW)
    assert d.reason is Reason.OK and (d.throttle, d.steering) == (0.0, -0.4)


# --- unknown space -----------------------------------------------------------

def test_missing_forward_coverage_stops(sup):
    s = scan()
    r = np.array(s.ranges_m)
    r[np.abs(s.angles_rad) < math.radians(20)] = np.nan  # blind wedge straight ahead
    blind = Scan(s.stamp_s, s.seq, s.angles_rad, r, s.quality)
    d = sup.evaluate(cmd(0.3), blind, NOW)
    assert d.reason is Reason.LOW_COVERAGE and d.throttle == 0.0
    assert d.coverage < 0.9


def test_empty_scan_stops(sup):
    s = Scan(NOW - 0.05, 0, [], [], [])
    assert sup.evaluate(cmd(0.3), s, NOW).reason is Reason.LOW_COVERAGE


def test_all_invalid_ranges_stop(sup):
    s = scan()
    bad = Scan(s.stamp_s, s.seq, s.angles_rad, np.full(len(s), np.inf), s.quality)
    assert sup.evaluate(cmd(0.3), bad, NOW).reason is Reason.LOW_COVERAGE


# --- missing / stale / future inputs ----------------------------------------

@pytest.mark.parametrize("command,scan_,reason", [
    (None, "ok", Reason.NO_COMMAND),
    ("ok", None, Reason.NO_SCAN),
])
def test_missing_inputs_stop(sup, command, scan_, reason):
    c = cmd() if command == "ok" else None
    s = scan() if scan_ == "ok" else None
    d = sup.evaluate(c, s, NOW)
    assert d.reason is reason and (d.throttle, d.steering) == (0.0, 0.0)


@pytest.mark.parametrize("scan_stamp,cmd_stamp,reason", [
    (NOW - 0.51, NOW, Reason.SCAN_STALE),
    (NOW + 0.2, NOW, Reason.SCAN_FROM_FUTURE),
    (NOW, NOW - 0.31, Reason.COMMAND_STALE),
    (NOW, NOW + 0.2, Reason.COMMAND_FROM_FUTURE),
])
def test_future_or_expired_inputs_stop(sup, scan_stamp, cmd_stamp, reason):
    d = sup.evaluate(cmd(stamp=cmd_stamp), scan(stamp=scan_stamp), NOW)
    assert d.reason is reason and d.throttle == 0.0


def test_inputs_exactly_at_timeout_are_still_fresh(sup):
    d = sup.evaluate(cmd(stamp=NOW - 0.3), scan(stamp=NOW - 0.5), NOW)
    assert d.reason is Reason.OK


def test_frozen_inputs_expire_as_time_advances(sup):
    s, c = scan(stamp=NOW), cmd(stamp=NOW)
    assert sup.evaluate(c, s, NOW + 0.1).reason is Reason.OK
    assert sup.evaluate(c, s, NOW + 0.35).reason is Reason.COMMAND_STALE


def test_nonfinite_clock_stops(sup):
    assert sup.evaluate(cmd(), scan(), math.nan).reason is Reason.INVALID_TIME


@pytest.mark.parametrize("throttle,steering", [
    (math.nan, 0.0), (0.2, math.inf), (1.5, 0.0), (0.2, -1.01),
])
def test_invalid_command_stops(sup, throttle, steering):
    d = sup.evaluate(cmd(throttle, steering), scan(), NOW)
    assert d.reason is Reason.INVALID_COMMAND and d.throttle == 0.0


def test_reverse_is_rejected(sup):
    d = sup.evaluate(cmd(-0.3), scan(), NOW)
    assert d.reason is Reason.REVERSE_BLOCKED and d.throttle == 0.0


# --- e-stop -----------------------------------------------------------------

def test_estop_latches_until_released(sup):
    sup.engage_estop()
    assert sup.evaluate(cmd(), scan(), NOW).reason is Reason.ESTOP
    assert sup.evaluate(cmd(), scan(), NOW + 0.01).reason is Reason.ESTOP
    sup.release_estop()
    assert sup.evaluate(cmd(), scan(), NOW).reason is Reason.OK


def test_estop_wins_over_every_other_reason(sup):
    sup.engage_estop()
    assert sup.evaluate(None, None, math.nan).reason is Reason.ESTOP


# --- config -----------------------------------------------------------------

@pytest.mark.parametrize("kw", [
    {"stop_distance_m": 1.0, "slow_distance_m": 0.5},
    {"max_speed": 0.0}, {"max_speed": 1.2},
    {"half_width_m": 0.0}, {"min_coverage": 1.5},
    {"scan_timeout_s": 0.0}, {"command_timeout_s": -1.0},
])
def test_invalid_config_rejected(kw):
    with pytest.raises(ValueError):
        SafetyConfig(**kw)


def test_decision_is_deterministic(sup):
    s, c = scan(box_ahead(0.7)), cmd(0.5, 0.3)
    assert sup.evaluate(c, s, NOW) == sup.evaluate(c, s, NOW)


# --- review fixes: blind wedge in the corridor, non-numeric input, steering vs FOV ----------

def _with_blind_wedge(s: Scan, centre_rad: float, half_rad: float) -> Scan:
    r = np.array(s.ranges_m)
    dev = np.abs((s.angles_rad - centre_rad + math.pi) % (2 * math.pi) - math.pi)
    r[dev <= half_rad] = np.nan
    return Scan(s.stamp_s, s.seq, s.angles_rad, r, s.quality)


def test_small_blind_wedge_in_corridor_stops_even_with_high_global_coverage(sup):
    # wall 3 m ahead, 9 beams (+-4.5 deg) return nothing: e.g. a black or glass object
    s = _with_blind_wedge(scan(), 0.0, math.radians(4.5))
    d = sup.evaluate(cmd(0.5), s, NOW)
    assert d.reason is Reason.LOW_COVERAGE and d.throttle == 0.0


def test_blind_wedge_in_steering_direction_stops(sup):
    s = _with_blind_wedge(scan(), math.radians(30), math.radians(3))
    assert sup.evaluate(cmd(0.3, 0.0), s, NOW).reason is Reason.OK  # not in the straight corridor
    assert sup.evaluate(cmd(0.3, 1.0), s, NOW).reason is Reason.LOW_COVERAGE


def test_blind_wedge_with_zero_throttle_still_allows_steering(sup):
    s = _with_blind_wedge(scan(), 0.0, math.radians(4.5))
    assert sup.evaluate(cmd(0.0, 0.5), s, NOW).reason is Reason.OK


@pytest.mark.parametrize("throttle,steering", [(None, 0.0), ("0.3", 0.0), (0.3, None)])
def test_non_numeric_command_is_invalid_not_an_exception(sup, throttle, steering):
    d = sup.evaluate(DriveCommand(throttle, steering, NOW), scan(), NOW)
    assert d.reason is Reason.INVALID_COMMAND and d.throttle == 0.0


def test_steering_corridor_must_stay_inside_coverage_fov():
    with pytest.raises(ValueError):
        SafetyConfig(max_steer_rad=math.radians(55), coverage_fov_rad=math.radians(120))


def test_reason_values_are_stable_and_fault_ultrasonic_appended():
    # Values go over the link (PiStatus.safety_reason); never rename or reorder them.
    assert [r.value for r in Reason] == [
        "ok", "slowed", "estop", "invalid_time", "no_scan", "scan_stale", "scan_from_future",
        "no_command", "command_stale", "command_from_future", "invalid_command", "reverse_blocked",
        "low_coverage", "obstacle", "fault", "ultrasonic"]
    from locatebot.safety.supervisor import SafetyDecision
    assert SafetyDecision(0.0, 0.0, Reason.FAULT).stopped
    assert SafetyDecision(0.0, 0.0, Reason.ULTRASONIC).stopped
