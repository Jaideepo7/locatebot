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
