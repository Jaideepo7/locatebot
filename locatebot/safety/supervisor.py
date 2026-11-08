"""Fail-closed LiDAR safety gate for drive commands.

``SafetySupervisor.evaluate(command, scan, now)`` turns a requested drive command into the
command the motors may actually execute, with an explicit reason. It runs on the Pi every
control tick, above manual drive and mission commands (docs/PROJECT.md §3).

Policy (frozen):
- Any stop outputs throttle 0 **and** steering 0 (centred), with a distinct ``Reason``.
- Checks run in this order; the first failure wins: e-stop, clock, scan presence/age,
  command presence/age, command validity, reverse, forward coverage, obstacle distance.
- Unknown is never free: missing or invalid beams lower ``forward_coverage``; below
  ``min_coverage`` the car stops even if nothing was "seen". When moving, the corridor the
  car is about to enter must also be covered (``corridor_min_coverage``, default every 2-degree
  bin within the corridor's angular half-width at ``stop_distance_m``, for each checked
  steering direction), so a small blind wedge ahead (black, glass or absorbing object) can't
  hide behind a high global coverage.
- Obstacles: the forward corridor (``half_width_m`` either side) is checked straight
  ahead and rotated towards the steering direction (0, 1/2, 1 x ``steering * max_steer_rad``)
  as a conservative stand-in for the swept arc, so turning can't dodge the check.
  Clearance <= ``stop_distance_m`` stops; between stop and ``slow_distance_m`` the allowed
  throttle rises linearly from 0 to ``max_speed``. Requested speed is never raised.
- Reverse is always blocked (no rear policy yet).
- Zero throttle with a healthy, covered scan is allowed, so the car can steer in place.

Throttle and steering are normalised to [-1, 1]; positive steering turns left (CCW).

Limits: the distances and timeouts are placeholders until braking distance and sensing
latency are measured on the real car. A 2D scan misses obstacles above or below the LiDAR
plane, glass and drop-offs. A sensor driver that invents fresh timestamps can't be detected
from scan age alone. Scans and commands must be stamped with the Pi's monotonic clock.
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass

from locatebot.lidar.detect import clean_scan, corridor_clearance, forward_coverage
from locatebot.lidar.types import Scan


class Reason(enum.Enum):
    OK = "ok"
    SLOWED = "slowed"                      # obstacle in the slow zone, throttle reduced
    ESTOP = "estop"
    INVALID_TIME = "invalid_time"
    NO_SCAN = "no_scan"
    SCAN_STALE = "scan_stale"
    SCAN_FROM_FUTURE = "scan_from_future"
    NO_COMMAND = "no_command"
    COMMAND_STALE = "command_stale"
    COMMAND_FROM_FUTURE = "command_from_future"
    INVALID_COMMAND = "invalid_command"
    REVERSE_BLOCKED = "reverse_blocked"
    LOW_COVERAGE = "low_coverage"
    OBSTACLE = "obstacle"
    # Not produced by ``evaluate``; shared here so every stop reason lives in one enum.
    FAULT = "fault"                        # Pi control loop caught an exception: motor stopped
    ULTRASONIC = "ultrasonic"              # reserved for ultrasonic fusion (ROADMAP T1.2)


@dataclass(frozen=True)
class DriveCommand:
    throttle: float  # [-1, 1], positive forward
    steering: float  # [-1, 1], positive left
    stamp_s: float   # monotonic time the command was issued (Pi clock)


@dataclass(frozen=True)
class SafetyDecision:
    throttle: float
    steering: float
    reason: Reason
    clearance_m: float | None = None  # min corridor clearance, when computed
    coverage: float | None = None     # forward coverage, when computed

    @property
    def stopped(self) -> bool:
        return self.reason not in (Reason.OK, Reason.SLOWED)


@dataclass(frozen=True)
class SafetyConfig:
    half_width_m: float = 0.15          # half the car width plus margin
    stop_distance_m: float = 0.40
    slow_distance_m: float = 1.00
    max_speed: float = 0.5              # throttle cap, fraction of full
    max_steer_rad: float = math.radians(30)
    min_coverage: float = 0.9
    coverage_fov_rad: float = math.radians(120)
    scan_timeout_s: float = 0.5
    command_timeout_s: float = 0.3
    future_tolerance_s: float = 0.05
    corridor_min_coverage: float = 1.0

    def __post_init__(self) -> None:
        if not 0.0 < self.stop_distance_m < self.slow_distance_m:
            raise ValueError("need 0 < stop_distance_m < slow_distance_m")
        if not 0.0 < self.max_speed <= 1.0:
            raise ValueError("max_speed must be in (0, 1]")
        if self.half_width_m <= 0.0:
            raise ValueError("half_width_m must be > 0")
        if not 0.0 < self.min_coverage <= 1.0:
            raise ValueError("min_coverage must be in (0, 1]")
        if not 0.0 < self.coverage_fov_rad <= 2 * math.pi:
            raise ValueError("coverage_fov_rad must be in (0, 2*pi]")
        if self.scan_timeout_s <= 0.0 or self.command_timeout_s <= 0.0 or self.future_tolerance_s < 0.0:
            raise ValueError("timeouts must be > 0 and future_tolerance_s >= 0")
        if not 0.0 <= self.max_steer_rad < math.pi / 2:
            raise ValueError("max_steer_rad must be in [0, pi/2)")
        if not 0.0 < self.corridor_min_coverage <= 1.0:
            raise ValueError("corridor_min_coverage must be in (0, 1]")
        if self.max_steer_rad + self.corridor_half_angle_rad > self.coverage_fov_rad / 2 + 1e-9:
            raise ValueError("steered corridor would leave coverage_fov_rad: reduce max_steer_rad "
                             "or widen coverage_fov_rad")

    @property
    def corridor_half_angle_rad(self) -> float:
        return math.atan2(self.half_width_m, self.stop_distance_m)


def _stop(reason: Reason, clearance: float | None = None, coverage: float | None = None) -> SafetyDecision:
    return SafetyDecision(0.0, 0.0, reason, clearance, coverage)


def _rotated(scan: Scan, phi: float) -> Scan:
    """The same scan seen from a frame rotated by phi (so a corridor along +x points along phi)."""
    return Scan(scan.stamp_s, scan.seq, scan.angles_rad - phi, scan.ranges_m, scan.quality)


class SafetySupervisor:
    def __init__(self, config: SafetyConfig | None = None):
        self.config = config or SafetyConfig()
        self._estop = False

    def engage_estop(self) -> None:
        self._estop = True

    def release_estop(self) -> None:
        self._estop = False

    @property
    def estop_engaged(self) -> bool:
        return self._estop

    def evaluate(self, command: DriveCommand | None, scan: Scan | None, now: float) -> SafetyDecision:
        c = self.config
        if self._estop:
            return _stop(Reason.ESTOP)
        if not math.isfinite(now):
            return _stop(Reason.INVALID_TIME)

        if scan is None:
            return _stop(Reason.NO_SCAN)
        if now - scan.stamp_s > c.scan_timeout_s:
            return _stop(Reason.SCAN_STALE)
        if scan.stamp_s - now > c.future_tolerance_s:
            return _stop(Reason.SCAN_FROM_FUTURE)

        if command is None:
            return _stop(Reason.NO_COMMAND)
        if not math.isfinite(command.stamp_s):
            return _stop(Reason.INVALID_COMMAND)
        if now - command.stamp_s > c.command_timeout_s:
            return _stop(Reason.COMMAND_STALE)
        if command.stamp_s - now > c.future_tolerance_s:
            return _stop(Reason.COMMAND_FROM_FUTURE)
        t, s = command.throttle, command.steering
        try:
            valid = math.isfinite(t) and math.isfinite(s) and abs(t) <= 1.0 and abs(s) <= 1.0
        except TypeError:  # None, strings, ...: never let a bad value raise past the gate
            valid = False
        if not valid:
            return _stop(Reason.INVALID_COMMAND)
        if t < 0.0:
            return _stop(Reason.REVERSE_BLOCKED)

        cleaned = clean_scan(scan)
        coverage = forward_coverage(cleaned, fov_rad=c.coverage_fov_rad)
        if coverage < c.min_coverage:
            return _stop(Reason.LOW_COVERAGE, coverage=coverage)
        if t == 0.0:
            return SafetyDecision(0.0, s, Reason.OK, None, coverage)

        phi = s * c.max_steer_rad
        steered = [_rotated(cleaned, k * phi) for k in (0.0, 0.5, 1.0)]
        corridor_cov = min(forward_coverage(r, fov_rad=2 * c.corridor_half_angle_rad) for r in steered)
        if corridor_cov < c.corridor_min_coverage:
            return _stop(Reason.LOW_COVERAGE, coverage=min(coverage, corridor_cov))
        clearance = min(corridor_clearance(r, c.half_width_m) for r in steered)
        if clearance <= c.stop_distance_m:
            return _stop(Reason.OBSTACLE, clearance, coverage)

        requested = min(t, c.max_speed)
        frac = (clearance - c.stop_distance_m) / (c.slow_distance_m - c.stop_distance_m)
        allowed = c.max_speed * min(1.0, frac)
        if requested > allowed:
            return SafetyDecision(allowed, s, Reason.SLOWED, clearance, coverage)
        return SafetyDecision(requested, s, Reason.OK, clearance, coverage)
