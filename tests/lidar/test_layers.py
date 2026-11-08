"""LayeredMap: static/dynamic layers, change detection, promotion, planning grid (M2).

Synthetic rooms only (locatebot.lidar.sim): software behaviour, not real-sensor behaviour.
Poses are ground truth here (the localizer is tested in test_localize.py), except in the
LocalizingMapper integration test.
"""
from __future__ import annotations

import copy
import math
import time

import numpy as np
import pytest

from locatebot.lidar.grid import FREE, OCCUPIED, OccupancyGrid
from locatebot.lidar.layers import NEW, ROOM, UNKNOWN, LayeredMap, LayersConfig
from locatebot.lidar.localize import LocalizingMapper
from locatebot.lidar.mapping import demo_room, demo_trajectory
from locatebot.lidar.sim import simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig
from locatebot.nav.costmap import build_costmap

ROBOT = Pose2D(2.6, 1.6, 0.6)   # looks towards the middle of demo_room


def scan_at(room, pose, rng, stamp=0.0, **kw):
    kw.setdefault("n_beams", 508)
    kw.setdefault("noise_std_m", 0.005)
    return simulate_scan(room, pose, rng=rng, stamp_s=stamp, **kw)


@pytest.fixture(scope="module")
def room_map() -> OccupancyGrid:
    rng = np.random.default_rng(1)
    g = OccupancyGrid.around((4.0, 3.0), 10.0)
    for p in demo_trajectory(60):
        g.integrate(scan_at(demo_room(), p, rng), p)
    return g


def box_cells(g, x1, y1, x2, y2):
    c = g.world_to_cell(np.array([[x1, y1], [x2, y2]]))
    return slice(c[0, 1], c[1, 1] + 1), slice(c[0, 0], c[1, 0] + 1)


def test_room_points_are_room_and_unknown_space_is_not_new(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    rng = np.random.default_rng(2)
    labels = lm.classify_points(scan_at(demo_room(), ROBOT, rng), ROBOT)
    valid = labels >= 0
    assert (labels[valid] == ROOM).mean() > 0.95 and not (labels == NEW).any()
    # a fresh (all-unknown) static map: nothing can be called new
    blank = LayeredMap(OccupancyGrid.around((4.0, 3.0), 10.0))
    lab = blank.classify_points(scan_at(demo_room(), ROBOT, rng), ROBOT)
    assert set(np.unique(lab[lab >= 0])) == {UNKNOWN}


def test_new_box_goes_to_dynamic_only_static_unchanged_after_50_scans(room_map):
    g = copy.deepcopy(room_map)
    lm = LayeredMap(g)
    room = demo_room().with_box(4.6, 2.3, 5.0, 2.7)
    sl = box_cells(g, 4.6, 2.3, 5.0, 2.7)
    static0 = g.log_odds[sl].copy()
    rng = np.random.default_rng(3)
    for i in range(50):
        up = lm.update(scan_at(room, ROBOT, rng, stamp=0.1 * i), ROBOT)
    assert (up.labels == NEW).sum() >= 10
    assert np.array_equal(g.log_odds[sl], static0)            # never burned into the room
    assert (lm.dynamic[sl] >= lm.config.dynamic_occ).any()
    assert len(up.blobs) == 1
    b = up.blobs[0]
    assert math.hypot(b.centroid_xy[0] - 4.7, b.centroid_xy[1] - 2.55) < 0.25
    assert 0.2 < b.width_m < 0.7 and b.persistence == 50 and not b.moving and not b.promoted
    assert (g.classify() == OCCUPIED).sum() >= (room_map.classify() == OCCUPIED).sum() * 0.97


def test_moving_box_is_tracked_and_leaves_no_trail(room_map):
    g = copy.deepcopy(room_map)
    lm = LayeredMap(g)
    occ0 = g.classify() == OCCUPIED
    rng = np.random.default_rng(4)
    ids, moving = set(), []
    for i in range(40):
        x = 4.0 + 0.04 * i          # 0.4 m/s along y = 2.5
        room = demo_room().with_box(x, 2.35, x + 0.3, 2.65)
        up = lm.update(scan_at(room, ROBOT, rng, stamp=0.1 * i), ROBOT)
        ids |= {b.id for b in up.blobs}
        moving.append(any(b.moving for b in up.blobs))
    assert len(ids) == 1                                       # one track the whole way
    assert all(moving[12:])
    path = box_cells(g, 4.0, 2.35, 5.9, 2.65)
    assert not (g.classify()[path] == OCCUPIED).any()
    # nothing new became room anywhere near where it drove (walls far away still get refined)
    near = box_cells(g, 3.0, 1.9, 6.3, 2.9)
    assert not ((g.classify() == OCCUPIED) & ~occ0)[near].any()
    assert up.blobs[0].centroid_xy[0] > 5.4


def test_box_that_stays_is_promoted_into_static(room_map):
    g = copy.deepcopy(room_map)
    lm = LayeredMap(g, LayersConfig(promote_after_s=5.0))
    room = demo_room().with_box(4.6, 2.3, 5.0, 2.7)
    sl = box_cells(g, 4.6, 2.3, 5.0, 2.7)
    rng = np.random.default_rng(5)
    t = 0.0
    for _ in range(45):                                        # 4.5 s: not yet
        lm.update(scan_at(room, ROBOT, rng, stamp=t), ROBOT)
        t += 0.1
    assert not (g.classify()[sl] == OCCUPIED).any()
    promoted = False
    for _ in range(60):                                        # promoted at 5 s, then slow hits
        up = lm.update(scan_at(room, ROBOT, rng, stamp=t), ROBOT)
        promoted |= any(b.promoted for b in lm.blobs(include_promoted=True))
        t += 0.1
    assert promoted
    assert (g.classify()[sl] == OCCUPIED).sum() >= 3          # part of the room now
    assert not up.blobs                                        # no longer reported as a change


def test_person_in_front_of_a_wall_keeps_the_wall_and_the_space_behind(room_map):
    g = copy.deepcopy(room_map)
    lm = LayeredMap(g)
    robot = Pose2D(4.0, 2.0, -math.pi / 2)                     # facing the y = 0 wall, 2 m away
    person = demo_room().with_box(3.85, 0.9, 4.15, 1.1)         # 1 m in front, wall behind it
    wall = box_cells(g, 3.7, -0.05, 4.3, 0.05)
    behind = box_cells(g, 3.85, 0.2, 4.15, 0.8)
    wall0, behind0 = g.log_odds[wall].copy(), g.log_odds[behind].copy()
    rng = np.random.default_rng(6)
    for i in range(30):
        up = lm.update(scan_at(person, robot, rng, stamp=0.1 * i), robot)
    assert np.all(g.log_odds[wall] >= wall0 - 1e-6)            # wall occupancy not reduced
    assert np.array_equal(g.log_odds[behind], behind0)         # nothing behind touched
    assert len(up.blobs) == 1 and up.blobs[0].width_m < 0.5


def test_planning_grid_blocks_the_dynamic_box_until_it_leaves(room_map):
    g = copy.deepcopy(room_map)
    lm = LayeredMap(g)
    room = demo_room().with_box(4.2, 1.2, 4.8, 2.8)            # a wide new thing across the corridor
    rng = np.random.default_rng(7)
    for i in range(10):
        lm.update(scan_at(room, ROBOT, rng, stamp=0.1 * i), ROBOT)
    pg = lm.planning_grid()
    sl = box_cells(g, 4.2, 1.2, 4.8, 2.8)
    assert (pg.classify()[sl] == OCCUPIED).any() and not (g.classify()[sl] == OCCUPIED).any()
    assert (pg.classify()[sl] == OCCUPIED).sum() > (g.classify()[sl] == OCCUPIED).sum()
    cm = build_costmap(pg, VehicleConfig())
    c, r = cm.world_to_cell((4.2, 2.0))
    assert not cm.traversable[r, c]
    # it leaves: rays pass through its cells and the dynamic layer decays
    for i in range(10, 60):
        lm.update(scan_at(demo_room(), ROBOT, rng, stamp=0.1 * i), ROBOT)
    pg = lm.planning_grid()
    assert not (lm.dynamic >= lm.config.dynamic_occ).any()
    assert np.array_equal(pg.log_odds, g.log_odds)
    assert not lm.blobs()


def test_dynamic_layer_decays_with_half_life(room_map):
    lm = LayeredMap(copy.deepcopy(room_map), LayersConfig(dynamic_half_life_s=2.0))
    lm.dynamic[10, 10] = 4.0
    lm.decay(0.0)
    lm.decay(2.0)
    assert lm.dynamic[10, 10] == pytest.approx(2.0)
    lay = lm.register_layer("glass_or_low", params={"half_life_s": 1.0, "blocks_above": 0.5})
    lay.data[5, 5] = 1.0
    lm.decay(3.0)
    assert lay.data[5, 5] == pytest.approx(0.5)
    assert lm.planning_grid().log_odds[5, 5] == lm.static.l_max
    lm.decay(3.5)
    assert lm.planning_grid().log_odds[5, 5] == lm.static.log_odds[5, 5]
    with pytest.raises(ValueError):
        lm.register_layer("dynamic")
    assert lm.register_layer("glass_or_low") is lay and set(lm.save_layers()) == {"glass_or_low"}


def test_localizing_mapper_with_layers_saves_v2_and_reloads(tmp_path, room_map):
    from locatebot.lidar.mapfile import load_map_layers, save_map
    save_map(tmp_path / "r.lbmap", room_map, {"home": [ROBOT.x, ROBOT.y, ROBOT.theta]})
    lm = LocalizingMapper.from_file(tmp_path / "r.lbmap", guess=ROBOT, layered=True)
    assert lm.layers is not None
    lm.layers.register_layer("wifi_rssi", fill=np.nan, params={"units": "dBm"})
    room = demo_room().with_box(4.6, 2.3, 5.0, 2.7)
    rng = np.random.default_rng(8)
    for i in range(10):
        r = lm.add_scan(scan_at(room, ROBOT, rng, stamp=0.1 * i))
        assert r.accepted
    assert len(lm.layers.blobs()) == 1
    sl = box_cells(lm.grid, 4.6, 2.3, 5.0, 2.7)
    assert np.array_equal(lm.grid.log_odds[sl], room_map.log_odds[sl])
    paths = lm.save(tmp_path / "r.v2")
    g, meta, layers = load_map_layers(paths["lbmap"])
    assert set(layers) == {"wifi_rssi"} and np.isnan(layers["wifi_rssi"].data).all()
    lm2 = LocalizingMapper.from_file(paths["lbmap"], guess=ROBOT, layered=True)
    assert set(lm2.layers.extra) == {"wifi_rssi"}
    assert (lm.grid.classify() == FREE).sum() == (g.classify() == FREE).sum()


def test_classification_and_layer_update_are_fast(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    room = demo_room().with_box(4.6, 2.3, 5.0, 2.7)
    rng = np.random.default_rng(9)
    scans = [scan_at(room, ROBOT, rng, stamp=0.1 * i) for i in range(20)]
    times = []
    for s in scans:
        t0 = time.perf_counter()
        lm.update(s, ROBOT)
        times.append(time.perf_counter() - t0)
    # laptop budget 15 ms per 508-point scan; keep CI margin for slower machines (Orin ~3x)
    assert np.median(times[1:]) < 0.045
