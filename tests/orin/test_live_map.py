"""LiveMapper: thread-safe live mapping on the Orin (synthetic scans only)."""
import math
import threading

import numpy as np

from locatebot.lidar.mapfile import load_map
from locatebot.lidar.mapping import demo_room, demo_trajectory
from locatebot.lidar.sim import simulate_scan
from locatebot.orin.live_map import LiveMapper


def _scans(n=40, seed=3):
    room, poses = demo_room(), demo_trajectory(60)[:n]
    rng = np.random.default_rng(seed)
    return poses, [simulate_scan(room, p, rng=rng, stamp_s=10.0 + i * 0.1, seq=i) for i, p in enumerate(poses)]


def test_pose_tracks_truth_relative_to_start():
    poses, scans = _scans()
    lm = LiveMapper(size_m=16.0)
    for s in scans:
        lm.add_scan(s)
    start, last = poses[0], poses[len(scans) - 1]
    rel = start.inverse().compose(last)
    est = lm.pose
    assert math.hypot(est.x - rel.x, est.y - rel.y) < 0.15
    assert lm.home.x == 0.0 and lm.home.y == 0.0


def test_histories_answer_frame_time_lookups():
    _, scans = _scans(10)
    lm = LiveMapper(size_m=16.0)
    for s in scans:
        lm.add_scan(s)
    t = scans[5].stamp_s + 0.03
    assert lm.pose_at(t) is not None
    assert lm.scan_near(scans[5].stamp_s).seq == 5
    assert lm.latest_scan.seq == 9
    assert lm.pose_at(scans[-1].stamp_s + 5.0) is None  # too far in the future


def test_snapshot_is_a_copy_and_save_writes_files(tmp_path):
    _, scans = _scans(15)
    lm = LiveMapper(size_m=16.0)
    for s in scans:
        lm.add_scan(s)
    snap = lm.grid_snapshot()
    snap.log_odds[:] = 0
    assert np.abs(lm.grid_snapshot().log_odds).sum() > 0
    paths = lm.save(tmp_path / "room.2026.11.08", meta={"session": "t"})
    grid, meta = load_map(paths["lbmap"])
    assert meta["session"] == "t" and meta["scans"] == 15
    assert grid.log_odds.shape == snap.log_odds.shape


def test_concurrent_readers_do_not_break_the_writer():
    _, scans = _scans(30)
    lm = LiveMapper(size_m=16.0)
    errors = []

    def reader():
        try:
            for _ in range(200):
                lm.grid_snapshot()
                lm.pose
                lm.stats()
        except Exception as e:  # pragma: no cover - reported below
            errors.append(e)

    th = threading.Thread(target=reader)
    th.start()
    for s in scans:
        lm.add_scan(s)
    th.join()
    assert not errors
    assert lm.stats()["scans"] == 30


# ---------------------------------------------------------------------- M1: localizing mode
def _saved_map(tmp_path):
    """Map a session with LiveMapper (odometry frame = first pose), save it, return the path."""
    poses, scans = _scans(60, seed=4)
    lm = LiveMapper(size_m=16.0)
    for s in scans:
        lm.add_scan(s)
    return lm.save(tmp_path / "room")["lbmap"], poses[0]


def _in_map_frame(start, pose):
    return start.inverse().compose(pose)


def test_from_saved_localizes_and_reports_mode(tmp_path):
    from locatebot.lidar.types import Pose2D
    path, start = _saved_map(tmp_path)
    room = demo_room()
    truth = demo_trajectory(60)[20]  # restart elsewhere in the room, guessed 0.3 m / 15 deg off
    t_map = _in_map_frame(start, truth)
    guess = Pose2D(t_map.x + 0.3, t_map.y - 0.2, t_map.theta + math.radians(15))
    lm = LiveMapper.from_saved(path, guess=guess)
    assert lm.localizing and lm.home == Pose2D(0.0, 0.0, 0.0)
    st = lm.stats()
    assert st["mode"] == "localizing" and not st["lost"] and st["last_score"] is None
    rng = np.random.default_rng(9)
    for i, p in enumerate(demo_trajectory(60)[20:30]):
        lm.add_scan(simulate_scan(room, p, rng=rng, stamp_s=50.0 + 0.1 * i, seq=i))
    est, t_last = lm.pose, _in_map_frame(start, demo_trajectory(60)[29])
    assert math.hypot(est.x - t_last.x, est.y - t_last.y) < 0.1
    st = lm.stats()
    assert st["mode"] == "localizing" and st["initialized"] and not st["lost"] and not st["ambiguous"]
    assert st["last_score"] > 0.5 and st["integrated"] == 10
    assert lm.pose_at(50.45) is not None
    out = lm.save(tmp_path / "room.v2")
    _, meta = load_map(out["lbmap"])
    assert meta["sessions"] == 2 and meta["last_session"]["mode"] == "localizing"


def test_mapping_mode_stats_keep_old_keys():
    _, scans = _scans(5)
    lm = LiveMapper(size_m=16.0)
    for s in scans:
        lm.add_scan(s)
    st = lm.stats()
    assert {"scans", "integrated", "rejected"} <= set(st)
    assert st["mode"] == "mapping" and st["lost"] is False and not lm.localizing
    lm.relocalize(None)  # no-op when mapping


def test_from_saved_concurrent_readers_and_relocalize(tmp_path):
    path, start = _saved_map(tmp_path)
    lm = LiveMapper.from_saved(path)  # no guess: global search
    room = demo_room()
    rng = np.random.default_rng(10)
    poses = demo_trajectory(60)[30:50]
    scans = [simulate_scan(room, p, rng=rng, stamp_s=80.0 + 0.1 * i, seq=i) for i, p in enumerate(poses)]
    errors = []

    def reader():
        try:
            for _ in range(200):
                lm.grid_snapshot()
                lm.pose
                lm.stats()
                lm.relocalize(None) if _ == 100 else None
        except Exception as e:  # pragma: no cover - reported below
            errors.append(e)

    th = threading.Thread(target=reader)
    th.start()
    for s in scans:
        lm.add_scan(s)
    th.join()
    assert not errors
    st = lm.stats()
    assert st["scans"] == 20 and st["mode"] == "localizing"
    est, t_last = lm.pose, _in_map_frame(start, poses[-1])
    assert math.hypot(est.x - t_last.x, est.y - t_last.y) < 0.1
