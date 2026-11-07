import math
import numpy as np
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D


def test_box_room_ranges_from_centre():
    room = Room.box(4.0, 2.0)
    s = simulate_scan(room, Pose2D(2.0, 1.0, 0.0), n_beams=4, noise_std_m=0.0)
    # beams at -pi, -pi/2, 0, pi/2
    assert np.allclose(s.ranges_m, [2.0, 1.0, 2.0, 1.0])


def test_heading_rotates_robot_frame():
    room = Room.box(4.0, 2.0)
    s = simulate_scan(room, Pose2D(2.0, 1.0, math.pi / 2), n_beams=4, noise_std_m=0.0)
    assert np.allclose(s.ranges_m, [1.0, 2.0, 1.0, 2.0])


def test_obstacle_box_is_hit_and_far_walls_give_nan_beyond_range():
    room = Room.box(30.0, 30.0).with_box(16.0, 14.5, 17.0, 15.5)
    s = simulate_scan(room, Pose2D(15.0, 15.0, 0.0), n_beams=4, noise_std_m=0.0, range_max_m=12.0)
    assert s.ranges_m[2] == 1.0
    assert np.isnan(s.ranges_m[0])


def test_noise_is_reproducible_with_seed():
    room = Room.box(5.0, 5.0)
    a = simulate_scan(room, Pose2D(2.5, 2.5, 0.0), n_beams=90, noise_std_m=0.01, rng=np.random.default_rng(3))
    b = simulate_scan(room, Pose2D(2.5, 2.5, 0.0), n_beams=90, noise_std_m=0.01, rng=np.random.default_rng(3))
    assert np.array_equal(a.ranges_m, b.ranges_m)
