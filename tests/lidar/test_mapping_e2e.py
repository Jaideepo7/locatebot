"""End-to-end: simulated exploration -> odometry -> occupancy grid -> .lbmap/.pgm.

Synthetic data only: proves the pipeline, not real-sensor accuracy.
"""
import math

import numpy as np

from locatebot.lidar.grid import OCCUPIED, OccupancyGrid
from locatebot.lidar.mapfile import load_map
from locatebot.lidar.mapping import Mapper, build_map_from_log, demo_room, demo_trajectory
from locatebot.lidar.scanlog import ScanLogWriter
from locatebot.lidar.sim import simulate_scan
from locatebot.lidar.types import Pose2D


def _seg_distance(points, segments):
    p = points[:, None, :]
    a = segments[None, :, :2]
    b = segments[None, :, 2:]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(-1) / (ab * ab).sum(-1), 0.0, 1.0)
    proj = a + t[..., None] * ab
    return np.linalg.norm(p - proj, axis=-1).min(axis=1)


def _explore(mapper, room, poses, seed=7):
    rng = np.random.default_rng(seed)
    for i, pose in enumerate(poses):
        scan = simulate_scan(room, pose, n_beams=450, noise_std_m=0.01, rng=rng, stamp_s=i * 0.1, seq=i, dropout=0.02)
        mapper.add_scan(scan)


def test_explore_builds_map_aligned_with_true_walls(tmp_path):
    room = demo_room()
    poses = demo_trajectory()
    mapper = Mapper(size_m=16.0)
    _explore(mapper, room, poses)

    # Map frame = robot pose at the first scan. Bring true walls into that frame.
    start_inv = poses[0].inverse()
    segs = room.segments
    a = start_inv.transform_points(segs[:, :2])
    b = start_inv.transform_points(segs[:, 2:])
    truth = np.hstack((a, b))

    occ = mapper.grid.classify() == OCCUPIED
    rows, cols = np.nonzero(occ)
    centres = mapper.grid.cell_to_world(np.column_stack((cols, rows)))
    d = _seg_distance(centres, truth)
    assert len(centres) > 200
    assert np.mean(d < 0.15) > 0.9, f"only {np.mean(d < 0.15):.2%} of occupied cells near a true wall"

    end_truth = start_inv.compose(poses[-1])
    end_est = mapper.odometry.pose
    err = math.hypot(end_est.x - end_truth.x, end_est.y - end_truth.y)
    assert err < 0.15, f"final position error {err:.3f} m"
    assert mapper.n_rejected <= len(poses) // 10


def test_save_writes_lbmap_pgm_yaml_and_round_trips(tmp_path):
    mapper = Mapper(size_m=16.0)
    _explore(mapper, demo_room(), demo_trajectory()[:15])
    paths = mapper.save(tmp_path / "session1", meta={"session": "test"})
    for key in ("lbmap", "pgm", "yaml"):
        assert paths[key].exists()
    grid, meta = load_map(paths["lbmap"])
    assert isinstance(grid, OccupancyGrid)
    assert np.array_equal(grid.log_odds, mapper.grid.log_odds)
    assert meta["session"] == "test"
    assert len(meta["trajectory"]) == mapper.n_integrated


def test_build_map_from_scan_log(tmp_path):
    room = demo_room()
    rng = np.random.default_rng(1)
    log = tmp_path / "run.lbscan"
    with ScanLogWriter(log) as w:
        for i, pose in enumerate(demo_trajectory()[:20]):
            w.write(simulate_scan(room, pose, n_beams=450, noise_std_m=0.01, rng=rng, stamp_s=i * 0.1, seq=i))
    summary = build_map_from_log(log, tmp_path / "out", size_m=16.0)
    assert summary["scans"] == 20
    assert summary["integrated"] >= 18
    assert (tmp_path / "out.lbmap").exists()


def test_save_keeps_dotted_stem_names(tmp_path):
    mapper = Mapper(size_m=16.0)
    _explore(mapper, demo_room(), demo_trajectory()[:3])
    paths = mapper.save(tmp_path / "2026.11.07")
    assert paths["lbmap"].name == "2026.11.07.lbmap"
    assert paths["pgm"].name == "2026.11.07.pgm"


def test_caller_meta_cannot_overwrite_pipeline_fields(tmp_path):
    mapper = Mapper(size_m=16.0)
    _explore(mapper, demo_room(), demo_trajectory()[:3])
    paths = mapper.save(tmp_path / "s", meta={"trajectory": [], "scans": -1, "note": "kept"})
    _, meta = load_map(paths["lbmap"])
    assert len(meta["trajectory"]) == mapper.n_integrated
    assert meta["scans"] == mapper.n_scans
    assert meta["note"] == "kept"


def test_summary_final_pose_is_last_integrated_pose(tmp_path):
    log = tmp_path / "r.lbscan"
    rng = np.random.default_rng(2)
    with ScanLogWriter(log) as w:
        for i, pose in enumerate(demo_trajectory()[:5]):
            w.write(simulate_scan(demo_room(), pose, rng=rng, stamp_s=i * 0.1, seq=i))
    s = build_map_from_log(log, tmp_path / "o", size_m=16.0)
    _, meta = load_map(tmp_path / "o.lbmap")
    assert s["final_pose"] == meta["trajectory"][-1][1:]
