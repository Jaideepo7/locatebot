"""Tests for locatebot.localize.items.

Scans are synthetic (locatebot.lidar.sim); they prove the geometry and bookkeeping, not
real camera/LiDAR alignment, which must be checked on the car.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, Scan
from locatebot.localize.items import (
    ItemTracker,
    LocalizeConfig,
    PoseHistory,
    ScanHistory,
    localize,
    range_along_ray,
)

ROOM = Room.box(8.0, 6.0)


def scan_at(room, pose, stamp=0.0):
    return simulate_scan(room, pose, n_beams=720, noise_std_m=0.0, stamp_s=stamp)


def bearing_to(pose: Pose2D, x: float, y: float, cam_xy=(0.0, 0.0)) -> float:
    """Bearing of a world point seen from the camera, robot frame."""
    local = pose.inverse().transform_points(np.array([[x, y]]))[0]
    return math.atan2(local[1] - cam_xy[1], local[0] - cam_xy[0])


# --- range along a ray -------------------------------------------------------

def test_range_along_ray_hits_nearest_surface():
    room = ROOM.with_box(3.9, 2.9, 4.1, 3.1)
    pose = Pose2D(2.0, 3.0, 0.0)
    r = range_along_ray(scan_at(room, pose), 0.0, (0.0, 0.0), math.radians(2))
    assert r == pytest.approx(1.9, abs=0.01)


def test_range_along_ray_none_when_no_valid_return():
    s = scan_at(ROOM, Pose2D(2.0, 3.0, 0.0))
    r = np.array(s.ranges_m)
    r[np.abs(s.angles_rad) < math.radians(10)] = np.nan
    blind = Scan(s.stamp_s, s.seq, s.angles_rad, r, s.quality)
    assert range_along_ray(blind, 0.0, (0.0, 0.0), math.radians(2)) is None


def test_range_along_ray_accounts_for_camera_offset():
    # camera 0.5 m to the left of the LiDAR, looking straight ahead at a post in line with it
    room = ROOM.with_box(4.9, 3.45, 5.1, 3.55)
    pose = Pose2D(2.0, 3.0, 0.0)
    r = range_along_ray(scan_at(room, pose), 0.0, (0.0, 0.5), math.radians(2))
    assert r == pytest.approx(2.9, abs=0.02)  # from the camera, not the wall behind


# --- localize ---------------------------------------------------------------

def test_localize_places_item_on_map():
    room = ROOM.with_box(3.9, 3.4, 4.1, 3.6)  # item centred at (4, 3.5)
    pose = Pose2D(2.0, 3.0, 0.0)
    fix = localize(bearing_to(pose, 4.0, 3.5), scan_at(room, pose), pose, LocalizeConfig(), stamp_s=5.0)
    assert fix is not None
    assert math.hypot(fix.x - 4.0, fix.y - 3.5) < 0.15
    assert fix.stamp_s == 5.0
    assert fix.range_m == pytest.approx(math.hypot(1.9, 0.475), abs=0.1)
    assert 0.0 < fix.sigma_m < 0.2


def test_localize_uses_robot_heading():
    room = ROOM.with_box(3.9, 3.9, 4.1, 4.1)  # item at (4, 4)
    pose = Pose2D(4.0, 1.0, math.pi / 2)    # facing +y, item straight ahead
    fix = localize(0.0, scan_at(room, pose), pose, LocalizeConfig())
    assert (fix.x, fix.y) == pytest.approx((4.0, 3.9), abs=0.03)


def test_localize_with_camera_offset():
    cfg = LocalizeConfig(camera_xy=(0.1, 0.05))
    room = ROOM.with_box(4.9, 1.9, 5.1, 2.1)  # item at (5, 2)
    pose = Pose2D(2.0, 3.0, 0.0)
    b = bearing_to(pose, 5.0, 2.0, cam_xy=cfg.camera_xy)
    fix = localize(b, scan_at(room, pose), pose, cfg)
    assert math.hypot(fix.x - 5.0, fix.y - 2.0) < 0.15


def test_localize_refuses_without_range():
    pose = Pose2D(2.0, 3.0, 0.0)
    s = scan_at(ROOM, pose)
    empty = Scan(s.stamp_s, s.seq, s.angles_rad, np.full(len(s), np.nan), s.quality)
    assert localize(0.0, empty, pose, LocalizeConfig()) is None


def test_localize_refuses_beyond_max_range():
    pose = Pose2D(0.5, 3.0, 0.0)  # far wall 7.5 m ahead
    assert localize(0.0, scan_at(ROOM, pose), pose, LocalizeConfig(max_range_m=6.0)) is None


def test_uncertainty_grows_with_distance():
    pose = Pose2D(2.0, 3.0, 0.0)
    near = localize(0.0, scan_at(ROOM.with_box(2.9, 2.9, 3.1, 3.1), pose), pose, LocalizeConfig())
    far = localize(0.0, scan_at(ROOM.with_box(5.9, 2.9, 6.1, 3.1), pose), pose, LocalizeConfig())
    assert far.sigma_m > near.sigma_m


# --- pose and scan history ----------------------------------------------------

def test_pose_history_interpolates_position_and_wrapped_heading():
    h = PoseHistory()
    h.add(1.0, Pose2D(0.0, 0.0, math.radians(170)))
    h.add(2.0, Pose2D(2.0, 4.0, math.radians(-170)))
    p = h.at(1.5)
    assert (p.x, p.y) == pytest.approx((1.0, 2.0))
    assert abs(abs(p.theta) - math.pi) < 1e-9  # through 180, not through 0


def test_pose_history_refuses_outside_known_span():
    h = PoseHistory(max_extrapolation_s=0.1)
    h.add(1.0, Pose2D(0, 0, 0))
    h.add(2.0, Pose2D(1, 0, 0))
    assert h.at(2.05) == Pose2D(1, 0, 0)  # small step past the end: hold last pose
    assert h.at(2.5) is None
    assert h.at(0.5) is None
    assert PoseHistory().at(1.0) is None


def test_pose_history_drops_old_entries():
    h = PoseHistory(max_age_s=5.0)
    for t in range(20):
        h.add(float(t), Pose2D(float(t), 0, 0))
    assert h.at(2.0) is None
    assert h.at(18.5).x == pytest.approx(18.5)


def test_pose_history_rejects_out_of_order():
    h = PoseHistory()
    h.add(2.0, Pose2D(0, 0, 0))
    with pytest.raises(ValueError):
        h.add(1.0, Pose2D(0, 0, 0))


def test_scan_history_returns_nearest_within_tolerance():
    h = ScanHistory(max_dt_s=0.15)
    a = scan_at(ROOM, Pose2D(2, 3, 0), stamp=1.0)
    b = scan_at(ROOM, Pose2D(2, 3, 0), stamp=1.1)
    h.add(a)
    h.add(b)
    assert h.nearest(1.04) is a
    assert h.nearest(1.08) is b
    assert h.nearest(1.5) is None


# --- tracker ------------------------------------------------------------------

def test_tracker_merges_nearby_sightings_with_inverse_variance_mean():
    from locatebot.localize.items import ItemFix
    t = ItemTracker(merge_radius_m=0.5, min_sightings=2)
    t.update("bolt", ItemFix(1.0, 1.0, 1.0, 0.1, 0.0))
    t.update("bolt", ItemFix(1.2, 1.0, 1.0, 0.2, 1.0))
    (track,) = t.tracks("bolt")
    # weights 1/0.01 and 1/0.04 -> x = (100*1.0 + 25*1.2) / 125 = 1.04
    assert track.x == pytest.approx(1.04)
    assert track.sightings == 2 and track.confirmed
    assert track.sigma_m == pytest.approx(math.sqrt(1 / 125))
    assert track.last_seen_s == 1.0


def test_tracker_keeps_far_sightings_as_separate_tracks():
    from locatebot.localize.items import ItemFix
    t = ItemTracker(merge_radius_m=0.5, min_sightings=2)
    t.update("remote", ItemFix(1.0, 1.0, 1.0, 0.1, 0.0))
    t.update("remote", ItemFix(3.0, 1.0, 1.0, 0.1, 1.0))
    t.update("remote", ItemFix(3.1, 1.0, 1.0, 0.1, 2.0))
    tracks = t.tracks("remote")
    assert len(tracks) == 2
    best = t.best("remote")
    assert best.sightings == 2 and best.x == pytest.approx(3.05)
    assert not tracks[0].confirmed


def test_tracker_items_are_independent():
    from locatebot.localize.items import ItemFix
    t = ItemTracker()
    t.update("a", ItemFix(1.0, 1.0, 1.0, 0.1, 0.0))
    assert t.tracks("b") == [] and t.best("b") is None
    assert t.items() == ["a"]
