"""MapLocalizer / LocalizingMapper / integrate_slow (M1).

All scans come from locatebot.lidar.sim: they prove software behaviour, not real-sensor
behaviour. Real recordings are checked with tools/relocalize.py (see docs/lidar/README.md).
"""
from __future__ import annotations

import copy
import hashlib
import math
import time

import numpy as np
import pytest

from locatebot.lidar.grid import OCCUPIED, OccupancyGrid, integrate_slow
from locatebot.lidar.localize import (LocalizerConfig, LocalizingMapper, MapLocalizer, map_home,
                                      reference_points)
from locatebot.lidar.mapfile import load_map, save_map
from locatebot.lidar.mapping import demo_room, demo_trajectory
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, wrap_angle

OFFSET = (0.5, -0.3, math.radians(25.0))


def scan_at(room, pose, rng, **kw):
    kw.setdefault("n_beams", 508)
    kw.setdefault("noise_std_m", 0.005)
    return simulate_scan(room, pose, rng=rng, **kw)


def build_map(room, poses, center, size, seed=1) -> OccupancyGrid:
    """Map built from ground-truth poses (tests the localizer, not odometry)."""
    rng = np.random.default_rng(seed)
    g = OccupancyGrid.around(center, size)
    for p in poses:
        g.integrate(scan_at(room, p, rng), p)
    return g


def second_loop(n=100) -> list[Pose2D]:
    """A different (clockwise, smaller) loop through demo_room than the mapping run."""
    out = []
    for i in range(n):
        t = -2.0 * math.pi * i / n
        out.append(Pose2D(4.0 + 1.8 * math.cos(t), 3.0 + 1.2 * math.sin(t),
                          math.atan2(-1.2 * math.cos(t), 1.8 * math.sin(t))))
    return out


def err(a: Pose2D, b: Pose2D) -> tuple[float, float]:
    return math.hypot(a.x - b.x, a.y - b.y), abs(math.degrees(wrap_angle(a.theta - b.theta)))


@pytest.fixture(scope="module")
def demo_map() -> OccupancyGrid:
    return build_map(demo_room(), demo_trajectory(60), (4.0, 3.0), 10.0)


def dock_for(truth: Pose2D) -> Pose2D:
    """The pose the car was told it is at (the dock) when it really is OFFSET away from it."""
    return Pose2D(truth.x - OFFSET[0], truth.y - OFFSET[1], truth.theta - OFFSET[2])


# ---------------------------------------------------------------------- golden: integrate unchanged
def test_integrate_is_bit_identical_to_before_m1():
    rng = np.random.default_rng(42)
    g = OccupancyGrid.around((4.0, 3.0), 10.0)
    room = demo_room()
    for i, p in enumerate(demo_trajectory(60)[::7]):
        s = simulate_scan(room, p, n_beams=508, rng=rng, dropout=0.1)
        g.integrate(s, p, max_range_m=(12.0 if i % 2 == 0 else 3.0))
    # sha256 of log_odds produced by OccupancyGrid.integrate before the M1 refactor (main @ ca07a18)
    assert hashlib.sha256(g.log_odds.tobytes()).hexdigest() == \
        "ab77e6fdb95ed32fd0e622e9883bc2517c878364eaaeaea1bcfcc18bff3806db"


def test_integrate_slow_full_rate_equals_integrate_and_damps_confident_cells():
    rng = np.random.default_rng(5)
    room, p = demo_room(), Pose2D(4.0, 2.0, 0.3)
    s = scan_at(room, p, rng, dropout=0.1)
    a, b = OccupancyGrid.around((4.0, 3.0), 10.0), OccupancyGrid.around((4.0, 3.0), 10.0)
    a.integrate(s, p)
    b.integrate_slow(s, p, rate=1.0)
    assert np.array_equal(a.log_odds, b.log_odds)

    # confident cells move at rate x, unknown cells at full increments
    c = OccupancyGrid.around((4.0, 3.0), 10.0)
    c.log_odds[:] = -3.0  # confidently free everywhere
    before = c.log_odds.copy()
    integrate_slow(c, s, p, rate=0.2)
    d = c.log_odds - before
    hit = d > 0
    assert hit.any()
    assert np.allclose(d[hit], 0.2 * c.l_occ, atol=1e-5)
    u = OccupancyGrid.around((4.0, 3.0), 10.0)  # unknown everywhere -> full increments
    integrate_slow(u, s, p, rate=0.2)
    assert np.array_equal(u.log_odds, a.log_odds)


def test_integrate_slow_beam_mask_skips_beams():
    rng = np.random.default_rng(6)
    room, p = demo_room(), Pose2D(4.0, 2.0, 0.3)
    s = scan_at(room, p, rng)
    g = OccupancyGrid.around((4.0, 3.0), 10.0)
    g.integrate_slow(s, p, beam_mask=np.zeros(len(s), bool))
    assert not g.log_odds.any()
    with pytest.raises(ValueError):
        g.integrate_slow(s, p, rate=1.5)


def test_reference_points_sit_on_walls_between_two_cells():
    g = OccupancyGrid(20, 20, 0.05, (0.0, 0.0))
    g.log_odds[:, 9] = 3.0   # a wall at x = 0.5 occupies columns 9 and 10 (centres 0.475, 0.525)
    g.log_odds[:, 10] = 3.0
    pts = reference_points(g, g.probability() > 0.65)
    assert np.allclose(pts[:, 0], 0.5, atol=1e-9)


# ---------------------------------------------------------------------- initialization
def test_restart_offset_from_dock_converges_within_5_scans_then_tracks(demo_map):
    room, path = demo_room(), second_loop(100)
    rng = np.random.default_rng(11)
    loc = MapLocalizer(demo_map, guess=dock_for(path[0]))
    converged_at = None
    errs = []
    for i, truth in enumerate(path):
        r = loc.update(scan_at(room, truth, rng, dropout=0.1))
        e_xy, e_th = err(r.pose, truth)
        if converged_at is None and r.accepted and e_xy < 0.1 and e_th < 3.0:
            converged_at = i
        if converged_at is not None:
            assert r.accepted, f"scan {i}: {r}"
            errs.append(e_xy)
            assert e_th < 3.0
    assert converged_at is not None and converged_at < 5
    assert max(errs) < 0.1
    assert loc.n_lost_events == 0


def test_global_init_finds_pose_in_asymmetric_room(demo_map):
    rng = np.random.default_rng(12)
    loc = MapLocalizer(demo_map)
    for truth in (Pose2D(5.8, 3.0, -1.6), Pose2D(2.5, 1.5, 2.6), Pose2D(5.0, 4.5, 0.4)):
        r = loc.initialize(scan_at(rng=rng, room=demo_room(), pose=truth), None)
        assert r.accepted and not r.ambiguous, r
        e_xy, e_th = err(r.pose, truth)
        assert e_xy < 0.1 and e_th < 3.0


def test_symmetric_room_reports_ambiguous_not_a_confident_wrong_answer():
    box = Room.box(6.0, 6.0)
    poses = [Pose2D(3 + 1.5 * math.cos(2 * math.pi * i / 40), 3 + 1.5 * math.sin(2 * math.pi * i / 40),
                    2 * math.pi * i / 40 + math.pi / 2) for i in range(40)]
    g = build_map(box, poses, (3.0, 3.0), 8.0)
    loc = MapLocalizer(g)
    rng = np.random.default_rng(13)
    r = loc.update(scan_at(box, Pose2D(2.0, 1.5, 0.3), rng))
    assert r.ambiguous and not r.accepted
    assert r.runner_up is not None and r.runner_up_score >= 0.95 * r.score
    assert not loc.initialized and loc.ambiguous
    # the operator's guess resolves it
    loc.request_init(Pose2D(2.2, 1.4, 0.2))
    r = loc.update(scan_at(box, Pose2D(2.0, 1.5, 0.3), rng))
    assert r.accepted and err(r.pose, Pose2D(2.0, 1.5, 0.3))[0] < 0.1


def test_no_scan_points_never_initializes(demo_map):
    loc = MapLocalizer(demo_map)
    empty = simulate_scan(demo_room(), Pose2D(4, 2, 0), range_max_m=0.01)  # all NaN
    r = loc.update(empty)
    assert not r.accepted and not loc.initialized


# ---------------------------------------------------------------------- changes, lost, fallback
def test_moved_furniture_still_converges_and_changes_slowly(demo_map):
    g = copy.deepcopy(demo_map)
    path = second_loop(100)
    room2 = demo_room().with_box(5.0, 2.4, 5.4, 2.8)   # a box that is not in the map
    occ_before = g.classify() == OCCUPIED
    confident = g.log_odds >= 2.0
    lm = LocalizingMapper(g, {}, guess=path[0])
    c = g.world_to_cell(np.array([[5.0, 2.4], [5.4, 2.8]]))
    box = (slice(c[0, 1], c[1, 1] + 1), slice(c[0, 0], c[1, 0] + 1))
    start_lo = g.log_odds[box].copy()
    rng = np.random.default_rng(14)
    for i, truth in enumerate(path[:10]):
        r = lm.add_scan(scan_at(room2, truth, rng))
        assert r.accepted and err(r.pose, truth)[0] < 0.1
    rose = g.log_odds[box] - start_lo
    assert rose.max() > 0.5                                   # the new box starts showing up...
    assert not (g.classify()[box] == OCCUPIED).any()          # ...slowly: not yet a wall
    occ_after = g.classify() == OCCUPIED
    # the old map is not wiped: every confident wall cell stays; only marginal edge cells
    # (p just above the threshold) churn, as they do when re-driving the unchanged room
    assert (occ_after[confident]).all()
    assert (occ_before & occ_after).sum() >= 0.95 * occ_before.sum()
    for truth in path[10:40]:
        lm.add_scan(scan_at(room2, truth, rng))
    assert (g.classify()[box] == OCCUPIED).any()              # a box that stays becomes room


def test_teleport_is_declared_lost_never_a_silent_jump(demo_map):
    room, path = demo_room(), second_loop(100)
    cfg = LocalizerConfig()
    loc = MapLocalizer(demo_map, cfg, guess=path[0])
    rng = np.random.default_rng(15)
    for truth in path[:20]:
        assert loc.update(scan_at(room, truth, rng)).accepted
    good = loc.pose
    lost_at = None
    for k, truth in enumerate(path[50:50 + cfg.max_odometry_updates + 2]):
        r = loc.update(scan_at(room, truth, rng))
        if r.accepted:  # any accepted pose must be right, never a jump to a wrong place
            assert err(r.pose, truth)[0] < 0.1
        if r.lost:
            lost_at = k
            break
        assert err(r.pose, good)[0] < 1.0  # no silent jump: stays near the last good pose
    assert lost_at is not None and lost_at < cfg.max_odometry_updates
    assert loc.lost and loc.n_lost_events == 1
    assert err(loc.pose, good)[0] < 1e-9   # holds the last good pose
    # while lost it keeps saying so, then recovers (global search, unambiguous) at the right pose
    recovered = None
    for k, truth in enumerate(path[60:60 + 3 * cfg.reinit_every]):
        r = loc.update(scan_at(room, truth, rng))
        if r.accepted:
            recovered = (k, r, truth)
            break
        assert r.lost
    assert recovered is not None
    assert err(recovered[1].pose, recovered[2])[0] < 0.1
    assert loc.n_recoveries == 1 and not loc.lost


def test_weak_map_match_falls_back_to_odometry_and_says_so(demo_map):
    room, path = demo_room(), second_loop(100)
    rng = np.random.default_rng(16)
    loc = MapLocalizer(demo_map, guess=path[0])
    for truth in path[:5]:
        assert loc.update(scan_at(room, truth, rng)).source in ("init", "map")
    # one scan of a room crowded with new things the map does not have: map match is weak
    crowd = room
    t = path[5]
    for k in range(10):
        a = 2 * math.pi * k / 10
        cx, cy = t.x + 0.6 * math.cos(a), t.y + 0.6 * math.sin(a)
        crowd = crowd.with_box(cx - 0.12, cy - 0.12, cx + 0.12, cy + 0.12)
    sources = []
    for truth in path[5:8]:
        r = loc.update(scan_at(crowd, truth, rng))
        sources.append(r.source)
        assert not r.lost
    assert "odometry" in sources or "prediction" in sources
    assert "map" not in sources


# ---------------------------------------------------------------------- mapper + file round trip
def test_localizing_mapper_saves_updated_map_with_metadata(tmp_path, demo_map):
    path = second_loop(100)
    save_map(tmp_path / "room.lbmap", demo_map, {"trajectory": [[0.0, 1.0, 2.0, 0.5]], "scans": 60})
    lm = LocalizingMapper.from_file(tmp_path / "room.lbmap", guess=path[0])
    assert lm.home == Pose2D(1.0, 2.0, 0.5)
    rng = np.random.default_rng(17)
    for truth in path[:5]:
        lm.add_scan(scan_at(demo_room(), truth, rng))
    paths = lm.save(tmp_path / "room.v2")
    g, meta = load_map(paths["lbmap"])
    assert meta["sessions"] == 2 and meta["home"] == [1.0, 2.0, 0.5]
    assert meta["trajectory"] == [[0.0, 1.0, 2.0, 0.5]]   # the mapping trajectory is kept
    assert meta["last_session"]["scans"] == 5 and lm.n_integrated == 5
    assert g.log_odds.shape == demo_map.log_odds.shape
    assert map_home({}, None) == Pose2D(0, 0, 0)
    assert map_home({"home": "bad"}, Pose2D(1, 1, 0)) == Pose2D(1, 1, 0)


# ---------------------------------------------------------------------- performance
def test_update_is_fast_enough(demo_map):
    room, path = demo_room(), second_loop(100)
    rng = np.random.default_rng(18)
    loc = MapLocalizer(demo_map, guess=path[0])
    scans = [scan_at(room, p, rng) for p in path[:25]]
    loc.update(scans[0])
    times = []
    for s in scans[1:]:
        t0 = time.perf_counter()
        loc.update(s)
        times.append(time.perf_counter() - t0)
    # laptop budget is 30 ms (measured ~6 ms); the Orin CPU is ~3x slower: keep CI margin
    assert np.median(times) < 0.09


def test_relocalize_tool_replays_a_log(tmp_path, demo_map):
    import importlib.util
    from pathlib import Path

    from locatebot.lidar.scanlog import ScanLogWriter
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location("relocalize", root / "tools" / "relocalize.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    path = second_loop(100)
    save_map(tmp_path / "m.lbmap", demo_map, {"home": [path[0].x, path[0].y, path[0].theta]})
    rng = np.random.default_rng(19)
    with ScanLogWriter(tmp_path / "run.lbscan") as w:
        for i, p in enumerate(path[:15]):
            w.write(scan_at(demo_room(), p, rng, stamp_s=i * 0.1, seq=i))
    s = mod.run(tmp_path / "m.lbmap", tmp_path / "run.lbscan", None, False, 0, out=tmp_path / "m.v2", quiet=True)
    assert s["accepted"] == 15 and s["lost_events"] == 0 and s["score_mean"] > 0.9
    assert (tmp_path / "m.v2.lbmap").exists()
    g = mod.run(tmp_path / "m.lbmap", tmp_path / "run.lbscan", None, True, 0, quiet=True)
    assert g["accepted"] == 15
    assert mod.parse_pose("1,2,90") == Pose2D(1.0, 2.0, math.pi / 2)
