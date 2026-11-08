"""scanmatch.py - 2D scan matching (ICP) for measuring how far the car turned/moved.

Points are in the LiDAR frame with x forward, y LEFT (counter-clockwise positive). The RPLIDAR
reports clockwise angles, so a reading (a_deg, d_mm) becomes x = d cos a, y = -d sin a (metres).

icp(before, after) finds R(phi), t with  before ~= R(phi) @ after + t, i.e. the car turned
phi (CCW / LEFT positive) and moved t between the two scans.
"""
import math
import numpy as np


def to_xy(points, rmin=0.15, rmax=6.0, max_n=450):
    """[(angle_deg_clockwise, dist_mm), ...] -> (N, 2) array, evenly subsampled."""
    a = np.array([p[0] for p in points], dtype=float)
    d = np.array([p[1] for p in points], dtype=float) / 1000.0
    m = (d > rmin) & (d < rmax)
    a, d = np.radians(a[m]), d[m]
    xy = np.stack([d * np.cos(a), -d * np.sin(a)], axis=1)
    if len(xy) > max_n:
        xy = xy[np.linspace(0, len(xy) - 1, max_n).astype(int)]
    return xy


def _rot(phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s], [s, c]])


def icp(before, after, phi0=0.0, t0=(0.0, 0.0), iters=30):
    """Returns (phi_rad, t, rms_m, inlier_frac)."""
    R, t = _rot(phi0), np.array(t0, dtype=float)
    B2 = (before ** 2).sum(1)
    thr = 0.5
    for _ in range(iters):
        A = after @ R.T + t
        d2 = (A ** 2).sum(1)[:, None] + B2[None, :] - 2 * A @ before.T
        j = np.argmin(d2, axis=1)
        dist = np.sqrt(np.maximum(d2[np.arange(len(A)), j], 0))
        thr = max(0.03, min(thr, 3.0 * np.median(dist)))
        m = dist < thr
        if m.sum() < 30:
            break
        P, Q = after[m], before[j[m]]                    # want Q ~= R P + t
        pc, qc = P.mean(0), Q.mean(0)
        H = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[1] *= -1; Rn = Vt.T @ U.T
        tn = qc - Rn @ pc
        if np.allclose(Rn, R, atol=1e-6) and np.allclose(tn, t, atol=1e-5):
            R, t = Rn, tn; break
        R, t = Rn, tn
    A = after @ R.T + t
    d2 = (A ** 2).sum(1)[:, None] + B2[None, :] - 2 * A @ before.T
    dist = np.sqrt(np.maximum(d2.min(1), 0))
    inl = dist < 0.05
    rms = float(np.sqrt((dist[inl] ** 2).mean())) if inl.any() else 1.0
    return math.atan2(R[1, 0], R[0, 0]), t, rms, float(inl.mean())


def match(before_pts, after_pts, guess_right_deg=0.0, spread_deg=40, step_deg=5):
    """Turn (deg, + = RIGHT/clockwise) and shift (m) between two scans.
    Tries ICP from several starting angles around the guess and keeps the best fit."""
    B, A = to_xy(before_pts), to_xy(after_pts)
    best = None
    inits = {0.0, float(guess_right_deg)}
    inits |= {float(guess_right_deg + k) for k in np.arange(-spread_deg, spread_deg + 1, step_deg)}
    for g in sorted(inits):
        phi, t, rms, frac = icp(B, A, phi0=math.radians(-g))
        score = frac - 2.0 * rms                         # many close points, small error
        if best is None or score > best[0]:
            best = (score, phi, t, rms, frac)
    _, phi, t, rms, frac = best
    return {"deg_right": -math.degrees(phi), "shift_m": [round(float(t[0]), 3), round(float(t[1]), 3)],
            "rms_mm": round(rms * 1000, 1), "inliers": round(frac, 2)}
