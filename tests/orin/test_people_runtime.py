"""M3 runtime wiring: people -> world state/summary, low-priority vision checks, ultrasonic
Range -> glass layer, notes. Synthetic scans; a fake vision service (no model)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from locatebot.agent.summary import render_world_state
from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.mapfile import save_map
from locatebot.lidar.mapping import demo_room, demo_trajectory
from locatebot.lidar.sim import simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.link.messages import Range
from locatebot.mission.types import SkillCall, SkillName, VehicleConfig
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import OrinRuntime
from locatebot.orin.skills import SkillExecutor
from locatebot.vision.service import Found, VisionResult

ROBOT = Pose2D(2.6, 1.6, 0.6)


class FakeVision:
    def __init__(self):
        self.items_calls = []

    def set_items(self, items):
        self.items_calls.append(list(items))

    def submit_frame(self, jpeg, capture_s):
        pass

    def status(self):
        return {"alive": True}


@pytest.fixture(scope="module")
def saved(tmp_path_factory):
    rng = np.random.default_rng(1)
    g = OccupancyGrid.around((4.0, 3.0), 10.0)
    for p in demo_trajectory(60):
        g.integrate(simulate_scan(demo_room(), p, n_beams=508, noise_std_m=0.005, rng=rng), p)
    path = tmp_path_factory.mktemp("m") / "room.lbmap"
    save_map(path, g, {"home": [ROBOT.x, ROBOT.y, ROBOT.theta]})
    return path


def legs(room, c, t):
    s = 0.15 * math.sin(2 * math.pi * t)
    for side, sg in ((0.12, 1.0), (-0.12, -1.0)):
        cx, cy = c[0] + s * sg, c[1] + side
        room = room.with_box(cx - 0.06, cy - 0.06, cx + 0.06, cy + 0.06)
    return room


def walk(rt, n, t0=10.0, seed=2):
    rng = np.random.default_rng(seed)
    centre = None
    for i in range(n):
        t = t0 + 0.1 * i
        centre = (3.4 + 0.5 * (t - t0), 1.2)
        rt.on_scan(simulate_scan(legs(demo_room(), centre, t), ROBOT, n_beams=508, noise_std_m=0.005,
                                 rng=rng, stamp_s=t, seq=i))
    return centre, t


def test_people_reach_world_state_and_camera_check_confirms(saved):
    vision = FakeVision()
    rt = OrinRuntime(LiveMapper.from_saved(saved, guess=ROBOT), SkillExecutor(VehicleConfig()), vision=vision)
    assert render_world_state(rt.world_state(now=10.0)).count("people: none") == 1
    centre, t = walk(rt, 25)
    ws = rt.world_state(now=t)
    assert len(ws.people) == 1
    p = ws.people[0]
    assert math.hypot(p.x - centre[0], p.y - centre[1]) < 0.15 and p.moving and not p.confirmed_by_camera
    assert abs(p.speed_mps - 0.5) < 0.15
    rt.step(t)
    assert vision.items_calls[-1] == ["person"]                     # low-priority check, no search running
    bearing = math.atan2(centre[1] - ROBOT.y, centre[0] - ROBOT.x) - ROBOT.theta
    rt.on_vision_result(VisionResult("person", t, 640, 480, [Found("person", (0, 0, 1, 1), bearing)]))
    assert rt.tracker.items() == []                                 # our check: not an item track
    ws = rt.world_state(now=t)
    assert ws.people[0].confirmed_by_camera
    line = [l for l in render_world_state(ws).splitlines() if l.startswith("people:")][0]
    assert line.startswith("people: 1 person ") and "walking (0.5 m/s, camera-confirmed)" in line
    rt.step(t + 6.0)                                                # check window over, confirmed
    assert vision.items_calls[-1] == []


def test_no_person_check_while_a_search_runs(saved):
    vision = FakeVision()
    rt = OrinRuntime(LiveMapper.from_saved(saved, guess=ROBOT), SkillExecutor(VehicleConfig()), vision=vision)
    rt.on_skill(SkillCall(SkillName.SEARCH, {"items": ("marker",)}, source="operator"))
    rt.step(9.9)
    _, t = walk(rt, 25)
    assert rt.mapper.people_wants_camera(t) is not None
    rt.step(t)
    assert all("person" not in c for c in vision.items_calls)
    assert vision.items_calls[-1] == ["marker"]


def test_range_messages_mark_glass_and_add_a_note(saved):
    lm = LiveMapper.from_saved(saved, guess=ROBOT)
    rt = OrinRuntime(lm, SkillExecutor(VehicleConfig()))
    rng = np.random.default_rng(3)
    for i in range(3):
        rt.on_scan(simulate_scan(demo_room(), ROBOT, n_beams=508, rng=rng, stamp_s=20.0 + 0.1 * i, seq=i))
    assert lm.layers is not None and "glass_or_low" not in lm.layers.extra   # no Range yet: no layer
    fov = math.radians(15)
    rt.on_range(Range(20.2, "front_ultrasonic", 0.6, fov))
    rt.on_range(Range(20.2, "front_ultrasonic", 0.6, fov))
    rt.on_range(Range(20.2, "front_ultrasonic", None, fov))         # no echo: ignored
    ws = rt.world_state(now=20.5)
    assert ws.notes == ("glass or low obstacle: 0.6 m ahead (ultrasonic only)",)
    assert "glass or low obstacle: 0.6 m ahead (ultrasonic only)" in render_world_state(ws).splitlines()
    ahead = lm.layers.static.world_to_cell(np.array([[ROBOT.x + 0.6 * math.cos(ROBOT.theta),
                                                      ROBOT.y + 0.6 * math.sin(ROBOT.theta)]]))[0]
    assert lm.grid_snapshot().classify()[ahead[1], ahead[0]] == 100   # planning sees it
    assert rt.world_state(now=40.0).notes == ()


def test_new_map_mode_is_inert():
    lm = LiveMapper(size_m=10.0)
    rng = np.random.default_rng(4)
    lm.add_scan(simulate_scan(demo_room(), ROBOT, rng=rng, stamp_s=1.0))
    assert not lm.on_range(1.0, 0.5, math.radians(15))
    assert not lm.add_wifi(-80.0, 1.0)
    assert lm.people() == [] and lm.notes(1.0) == [] and lm.layer_snapshot("wifi_rssi") is None


def test_wifi_readings_build_the_layer_and_the_page_overlay(saved):
    from locatebot.orin.app import map_png
    lm = LiveMapper.from_saved(saved, guess=ROBOT)
    rt = OrinRuntime(lm, SkillExecutor(VehicleConfig()))
    rng = np.random.default_rng(5)
    rt.on_scan(simulate_scan(demo_room(), ROBOT, n_beams=508, rng=rng, stamp_s=30.0))
    plain = map_png(rt, "wifi")
    assert plain == map_png(rt)                                     # no readings: plain map
    assert lm.add_wifi(-81.0, 30.0)
    assert map_png(rt, "wifi") != plain and map_png(rt, "wifi")[25] == 2   # RGB colour type
    assert any(n.startswith("wifi: weak here (-81 dBm)") for n in rt.world_state(now=30.5).notes)
