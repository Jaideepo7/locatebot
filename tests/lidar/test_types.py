import math
import numpy as np
import pytest
from locatebot.lidar.types import Scan, Pose2D, RANGE_MIN_M, RANGE_MAX_M


def make_scan(ranges, stamp=1.0):
    n = len(ranges)
    angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return Scan(stamp_s=stamp, seq=0, angles_rad=angles, ranges_m=np.array(ranges, float),
                quality=np.full(n, 47, np.uint8))


def test_scan_arrays_are_read_only():
    s = make_scan([1.0, 2.0, 3.0, 4.0])
    with pytest.raises(ValueError):
        s.ranges_m[0] = 9.0


def test_scan_rejects_mismatched_lengths():
    with pytest.raises(ValueError):
        Scan(stamp_s=0.0, seq=0, angles_rad=np.zeros(3), ranges_m=np.zeros(2), quality=np.zeros(3, np.uint8))


def test_valid_mask_drops_nan_and_out_of_range():
    s = make_scan([float("nan"), RANGE_MIN_M / 2, 1.0, RANGE_MAX_M + 1])
    assert s.valid_mask().tolist() == [False, False, True, False]


def test_points_xy_forward_is_plus_x_left_is_plus_y():
    s = Scan(stamp_s=0.0, seq=0, angles_rad=np.array([0.0, math.pi / 2]), ranges_m=np.array([2.0, 3.0]),
             quality=np.array([10, 10], np.uint8))
    pts = s.points_xy()
    assert np.allclose(pts, [[2.0, 0.0], [0.0, 3.0]], atol=1e-12)


def test_pose_compose_and_inverse_round_trip():
    a = Pose2D(1.0, 2.0, 0.5)
    b = Pose2D(-0.3, 0.7, -1.2)
    c = a.compose(b).compose(b.inverse())
    assert c.x == pytest.approx(a.x) and c.y == pytest.approx(a.y) and c.theta == pytest.approx(a.theta)


def test_pose_transform_points_rotates_then_translates():
    p = Pose2D(1.0, 0.0, math.pi / 2)
    out = p.transform_points(np.array([[1.0, 0.0]]))
    assert np.allclose(out, [[1.0, 1.0]])


def test_pose_theta_is_wrapped():
    assert Pose2D(0, 0, 3 * math.pi).theta == pytest.approx(math.pi) or Pose2D(0, 0, 3 * math.pi).theta == pytest.approx(-math.pi)
