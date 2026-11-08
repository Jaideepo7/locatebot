"""Ultrasonic-only hits -> glass_or_low layer (M3). Synthetic rooms only: a 'glass pane' is a
wall the simulated LiDAR does not see; the ultrasonic reading is given directly."""
from __future__ import annotations

import copy
import math

import numpy as np
import pytest

from locatebot.lidar.grid import OCCUPIED, OccupancyGrid
from locatebot.lidar.layers import LayeredMap
from locatebot.lidar.mapping import demo_room, demo_trajectory
from locatebot.lidar.sim import simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.lidar.ultrasonic import LAYER_NAME, UltrasonicConfig, UltrasonicLayer
from locatebot.mission.types import VehicleConfig
from locatebot.nav.costmap import build_costmap

FOV = math.radians(15.0)
ROBOT = Pose2D(4.6, 2.0, 0.0)   # facing +x, open floor ahead in demo_room


@pytest.fixture(scope="module")
def room_map() -> OccupancyGrid:
    rng = np.random.default_rng(1)
    g = OccupancyGrid.around((4.0, 3.0), 10.0)
    for p in demo_trajectory(60):
        g.integrate(simulate_scan(demo_room(), p, n_beams=508, noise_std_m=0.005, rng=rng), p)
    return g


def setup(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    return lm, UltrasonicLayer(lm)


def test_glass_pane_is_marked_and_blocks_planning(room_map):
    lm, us = setup(room_map)
    rng = np.random.default_rng(2)
    scan = simulate_scan(demo_room(), ROBOT, n_beams=508, rng=rng)   # glass: the LiDAR sees through
    assert us.on_range(0.6, FOV, ROBOT, scan, 1.0)
    lay = lm.layer(LAYER_NAME)
    c = lm.static.world_to_cell(np.array([[ROBOT.x + 0.6, ROBOT.y]]))[0]
    assert 0.3 < lay.data[c[1], c[0]] < 0.6                       # low weight: one reading
    assert lm.planning_grid().classify()[c[1], c[0]] != OCCUPIED
    assert us.on_range(0.6, FOV, ROBOT, scan, 1.1)
    pg = lm.planning_grid()
    assert pg.classify()[c[1], c[0]] == OCCUPIED                    # two readings: an obstacle
    cm = build_costmap(pg, VehicleConfig())
    assert not cm.traversable[c[1], c[0]]
    assert lm.static.classify()[c[1], c[0]] != OCCUPIED             # the room map is untouched
    note = us.note(Pose2D(ROBOT.x, ROBOT.y, 0.0), 1.5)
    assert note == "glass or low obstacle: 0.6 m ahead (ultrasonic only)"
    assert us.note(ROBOT, 10.0) is None                             # stale


def test_lidar_sees_the_same_object_nothing_added(room_map):
    lm, us = setup(room_map)
    rng = np.random.default_rng(3)
    room = demo_room().with_box(5.2, 1.9, 5.4, 2.1)                 # a real box 0.6 m ahead
    scan = simulate_scan(room, ROBOT, n_beams=508, rng=rng)
    assert not us.on_range(0.6, FOV, ROBOT, scan, 1.0)
    assert not lm.layer(LAYER_NAME).data.any()
    assert us.stats() == {"readings": 1, "marked": 0, "lidar_saw": 1}


def test_marks_survive_lidar_free_rays_and_none_means_nothing(room_map):
    lm, us = setup(room_map)
    rng = np.random.default_rng(4)
    scan = simulate_scan(demo_room(), ROBOT, n_beams=508, rng=rng)
    us.on_range(0.6, FOV, ROBOT, scan, 0.0)
    before = lm.layer(LAYER_NAME).data.copy()
    for i in range(30):   # LiDAR rays pass straight through the marked cells
        lm.update(simulate_scan(demo_room(), ROBOT, n_beams=508, rng=rng, stamp_s=0.1 * i), ROBOT)
    after = lm.layer(LAYER_NAME).data
    assert after.sum() > 0.99 * before.sum()                        # only the slow 10 min decay
    assert not us.on_range(None, FOV, ROBOT, scan, 5.0)             # no echo: unknown, not free
    assert not us.on_range(3.5, FOV, ROBOT, scan, 5.0)              # beyond max_range_m
    assert np.array_equal(lm.layer(LAYER_NAME).data, after)


def test_mount_offset_and_yaw(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    us = UltrasonicLayer(lm, UltrasonicConfig(mount_xy=(0.15, 0.0), mount_yaw_rad=math.pi / 2))
    rng = np.random.default_rng(5)
    scan = simulate_scan(demo_room(), ROBOT, n_beams=508, rng=rng)
    assert us.on_range(0.5, FOV, ROBOT, scan, 0.0)
    c = lm.static.world_to_cell(np.array([[ROBOT.x + 0.15, ROBOT.y + 0.5]]))[0]
    assert lm.layer(LAYER_NAME).data[c[1], c[0]] > 0
    assert "left" in us.note(ROBOT, 0.5)


def test_no_range_messages_no_layer(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    assert LAYER_NAME not in lm.extra and lm.save_layers() == {}
