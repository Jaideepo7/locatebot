"""Kinematic bicycle model of the car, for closing the loop in tests and offline demos.

Synthetic: proves software behaviour, not real-car behaviour (no slip, no motor dynamics unless
``speed_tau_s``/``steer_rate_rad_s`` are set, ideal odometry).

Throttle mapping (assumption, see docs/nav/README.md): speed = throttle / max_throttle *
max_speed_mps, i.e. ``VehicleConfig.max_speed_mps`` is the speed at ``throttle == max_throttle``
(which should equal ``SafetyConfig.max_speed``). Reverse is blocked like on the Pi.
"""
from __future__ import annotations

import math

from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig


def throttle_to_speed(vehicle: VehicleConfig, throttle: float) -> float:
    t = min(max(float(throttle), 0.0), vehicle.max_throttle)
    return t / vehicle.max_throttle * vehicle.max_speed_mps


def speed_to_throttle(vehicle: VehicleConfig, speed_mps: float) -> float:
    v = min(max(float(speed_mps), 0.0), vehicle.max_speed_mps)
    return v / vehicle.max_speed_mps * vehicle.max_throttle


class BicycleSim:
    """Rear-axle-referenced kinematic bicycle. ``pose`` is the rear-axle centre."""

    def __init__(self, vehicle: VehicleConfig, pose: Pose2D, dt: float = 0.05,
                 speed_tau_s: float = 0.0, steer_rate_rad_s: float | None = None) -> None:
        if dt <= 0:
            raise ValueError("dt must be > 0")
        self.vehicle = vehicle
        self.pose = pose
        self.dt = float(dt)
        self.speed_tau_s = float(speed_tau_s)
        self.steer_rate_rad_s = steer_rate_rad_s
        self.speed_mps = 0.0
        self.steer_rad = 0.0
        self.time_s = 0.0
        self.distance_m = 0.0

    def step(self, throttle: float, steering: float) -> Pose2D:
        v = self.vehicle
        target_v = throttle_to_speed(v, throttle)  # negative throttle -> 0 (reverse blocked)
        if self.speed_tau_s > 0:
            a = 1.0 - math.exp(-self.dt / self.speed_tau_s)
            self.speed_mps += a * (target_v - self.speed_mps)
        else:
            self.speed_mps = target_v
        target_d = max(-1.0, min(1.0, float(steering))) * v.max_steer_rad
        if self.steer_rate_rad_s is not None:
            dmax = self.steer_rate_rad_s * self.dt
            self.steer_rad += max(-dmax, min(dmax, target_d - self.steer_rad))
        else:
            self.steer_rad = target_d
        ds = self.speed_mps * self.dt
        if ds > 0:
            p = self.pose
            k = math.tan(self.steer_rad) / v.wheelbase_m
            dth = k * ds
            if abs(dth) < 1e-9:
                x = p.x + ds * math.cos(p.theta)
                y = p.y + ds * math.sin(p.theta)
            else:  # exact arc
                r = 1.0 / k
                x = p.x + r * (math.sin(p.theta + dth) - math.sin(p.theta))
                y = p.y - r * (math.cos(p.theta + dth) - math.cos(p.theta))
            self.pose = Pose2D(x, y, p.theta + dth)
            self.distance_m += ds
        self.time_s += self.dt
        return self.pose
