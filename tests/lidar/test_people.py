"""PeopleTracker on LayeredMap change clusters (M3).

"People" here are pairs of small moving boxes (legs, with a gait) in locatebot.lidar.sim rooms:
this proves the software, not that real people, pets or chairs behave like this.
"""
from __future__ import annotations

import copy
import math
import time

import numpy as np
import pytest

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.layers import LayeredMap
from locatebot.lidar.mapping import demo_room, demo_trajectory
from locatebot.lidar.people import PeopleConfig, PeopleTracker
from locatebot.lidar.sim import simulate_scan
from locatebot.lidar.types import Pose2D

ROBOT = Pose2D(2.6, 1.6, 0.6)


@pytest.fixture(scope="module")
def room_map() -> OccupancyGrid:
    rng = np.random.default_rng(1)
    g = OccupancyGrid.around((4.0, 3.0), 10.0)
    for p in demo_trajectory(60):
        g.integrate(simulate_scan(demo_room(), p, n_beams=508, noise_std_m=0.005, rng=rng), p)
    return g


def with_legs(room, c, heading, t, leg=0.12, half_gap=0.12, step=0.18, freq=1.0):
    """Two leg boxes beside the body centre c; the gait swings them forward/back in turn."""
    ux, uy = math.cos(heading), math.sin(heading)
    px, py = -uy, ux
    s = step * math.sin(2 * math.pi * freq * t)
    for side, sg in ((half_gap, 1.0), (-half_gap, -1.0)):
        cx, cy = c[0] + px * side + ux * s * sg, c[1] + py * side + uy * s * sg
        room = room.with_box(cx - leg / 2, cy - leg / 2, cx + leg / 2, cy + leg / 2)
    return room


def run(room_map, people_at, seconds, dt=0.1, seed=3, robot=ROBOT, tracker=None):
    """people_at(t) -> list of (centre, heading); yields (t, centres, tracks) per scan."""
    lm = LayeredMap(copy.deepcopy(room_map))
    pt = tracker or PeopleTracker()
    rng = np.random.default_rng(seed)
    out = []
    for i in range(int(round(seconds / dt))):
        t = i * dt
        room = demo_room()
        ppl = people_at(t)
        for c, h in ppl:
            room = with_legs(room, c, h, t)
        s = simulate_scan(room, robot, n_beams=508, noise_std_m=0.005, rng=rng, stamp_s=t)
        lm.update(s, robot)
        out.append((t, [c for c, _ in ppl], pt.update(t, lm.last_clusters)))
    return out, pt, lm


def walker(x0, y0, heading, v):
    return lambda t: [((x0 + v * t * math.cos(heading), y0 + v * t * math.sin(heading)), heading)]


def test_walking_person_one_track_accurate_position_and_speed(room_map):
    out, pt, _ = run(room_map, walker(3.4, 1.2, 0.0, 0.6), 3.6)
    ids = {tr.id for _, _, trs in out for tr in trs}
    assert len(ids) == 1
    errs, speeds = [], []
    for t, cs, trs in out[10:]:
        assert len(trs) == 1
        errs.append(math.hypot(trs[0].x - cs[0][0], trs[0].y - cs[0][1]))
        speeds.append(trs[0].speed_mps)
    assert max(errs) < 0.15, f"max error {max(errs):.3f} m"
    assert abs(np.mean(speeds) - 0.6) < 0.12
    p = pt.people()
    assert len(p) == 1 and p[0].likely_person and p[0].moving and not p[0].confirmed_by_camera


def test_static_chair_is_an_object_not_a_person(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    pt = PeopleTracker()
    rng = np.random.default_rng(4)
    room = demo_room()
    for dx in (0.0, 0.45):
        for dy in (0.0, 0.45):
            x, y = 4.6 + dx, 2.0 + dy
            room = room.with_box(x - 0.03, y - 0.03, x + 0.03, y + 0.03)
    for i in range(50):
        s = simulate_scan(room, ROBOT, n_beams=508, noise_std_m=0.005, rng=rng, stamp_s=0.1 * i)
        lm.update(s, ROBOT)
        pt.update(0.1 * i, lm.last_clusters)
    assert lm.blobs()                       # it is a change...
    assert pt.people() == []                # ...but not a person
    assert all(not t.likely_person for t in pt.tracks())
    assert pt.wants_camera_check(5.0) is None


def test_two_people_crossing_keep_their_identities(room_map):
    # A walks +x along y = 1.4, B walks -y along x = 4.6; their paths cross at (4.6, 1.4) with
    # B 0.8 s ahead of A (closest approach ~0.4 m). Simultaneous arrival at the crossing point
    # (legs merging into one cluster, one occluding the other) is not covered.
    a, b = walker(3.2, 1.4, 0.0, 0.5), walker(4.6, 2.4, -math.pi / 2, 0.5)

    def both(t):
        return a(t) + b(t)

    out, _, _ = run(room_map, both, 4.0, seed=5)

    def ident(i, k):
        t, cs, trs = out[i]
        best = min(trs, key=lambda tr: math.hypot(tr.x - cs[k][0], tr.y - cs[k][1]))
        assert math.hypot(best.x - cs[k][0], best.y - cs[k][1]) < 0.3
        return best.id

    id_a, id_b = ident(8, 0), ident(8, 1)
    assert id_a != id_b
    assert ident(len(out) - 1, 0) == id_a and ident(len(out) - 1, 1) == id_b


def test_candidates_geometry():
    pt = PeopleTracker()
    legs = pt.candidates([((1.0, 0.0), 0.10), ((1.0, 0.3), 0.10)])
    assert len(legs) == 1 and legs[0][2] == "legs" and np.allclose(legs[0][0], (1.0, 0.15))
    body = pt.candidates([((2.0, 0.0), 0.4)])
    assert [c[2] for c in body] == ["body"]
    single = pt.candidates([((2.0, 0.0), 0.1)])
    assert [c[2] for c in single] == ["leg"]
    assert pt.candidates([((2.0, 0.0), 1.2), ((0, 0), 0.01)]) == []   # walls and specks
    far = pt.candidates([((1.0, 0.0), 0.10), ((1.0, 0.9), 0.10)])
    assert [c[2] for c in far] == ["leg", "leg"]                       # too far apart for a pair
    hip = PeopleTracker(PeopleConfig(body_width_m=(0.3, 0.9)))        # LiDAR at hip height
    assert [c[2] for c in hip.candidates([((2.0, 0.0), 0.8)])] == ["body"]


def test_track_birth_after_three_detections_and_death_after_max_missed():
    pt = PeopleTracker(PeopleConfig(max_missed_s=1.0))
    det = [((1.0, 0.0), 0.10), ((1.0, 0.3), 0.10)]
    assert pt.update(0.0, det) == []
    assert pt.update(0.1, det) == []
    tr = pt.update(0.2, det)
    assert len(tr) == 1 and tr[0].hits == 3 and tr[0].confirmed
    assert pt.update(0.4, [((1.0, 0.15), 0.1)])  # one leg only: keeps the track alive
    assert len(pt.update(1.3, [])) == 1
    assert pt.update(1.5, []) == []              # > 1.0 s without a detection
    # a single leg never starts a track
    for k in range(5):
        pt.update(2.0 + 0.1 * k, [((3.0, 0.0), 0.1)])
    assert pt.tracks() == [] and pt.update(2.6, []) == []


def test_camera_check_requested_for_walkers_and_fix_confirms():
    pt = PeopleTracker()
    for k in range(20):  # walking at 0.5 m/s
        x = 1.0 + 0.05 * k
        pt.update(0.1 * k, [((x, 0.0), 0.1), ((x, 0.3), 0.1)])
    want = pt.wants_camera_check(2.0)
    assert want is not None and want.likely_person
    pt.camera_requested(want.id, 2.0)
    assert pt.wants_camera_check(3.0) is None       # asked recently
    assert pt.on_camera_fix((5.0, 5.0), 3.0) is None  # far away: not this track
    got = pt.on_camera_fix((1.95, 0.2), 3.0)
    assert got is not None and got.confirmed_by_camera and got.id == want.id
    for k in range(20, 40):  # stands still now: still a person (it walked, and the camera said so)
        pt.update(0.1 * k, [((1.95, 0.0), 0.1), ((1.95, 0.3), 0.1)])
    assert pt.wants_camera_check(4.0) is None       # confirmed 1 s ago
    p = pt.people()
    assert len(p) == 1 and p[0].confirmed_by_camera and not p[0].moving
    assert pt.wants_camera_check(13.5) is not None  # 10 s later: worth another look


def test_tracker_plus_layers_fast_enough(room_map):
    lm = LayeredMap(copy.deepcopy(room_map))
    pt = PeopleTracker()
    rng = np.random.default_rng(6)
    scans = []
    for i in range(30):
        t = 0.1 * i
        room = with_legs(demo_room(), (3.4 + 0.06 * i, 1.2), 0.0, t)
        scans.append(simulate_scan(room, ROBOT, n_beams=508, noise_std_m=0.005, rng=rng, stamp_s=t))
    times = []
    for s in scans:
        t0 = time.perf_counter()
        lm.update(s, ROBOT)
        pt.update(s.stamp_s, lm.last_clusters)
        times.append(time.perf_counter() - t0)
    # laptop budget 10 ms (measured ~3 ms); margin for slower CI machines / the Orin (~3x)
    assert np.median(times) < 0.03
