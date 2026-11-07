"""Likelihood-field correlative scan matching (numpy only).

A reference point set (one scan, or a submap of keyframes) is rasterised into a
likelihood field ``exp(-d^2 / (2 sigma^2))``, ``d`` = distance to the nearest
reference point. A scan is matched by searching (dx, dy, dtheta) around an
initial pose, coarse-to-fine; every candidate of a level is scored in one
vectorised numpy pass.

Accuracy figures quoted in the tests come from synthetic scans
(``locatebot.lidar.sim``); they prove software behaviour, not real-sensor behaviour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .types import Pose2D

# Kernel radius in sigmas: beyond this the field is treated as exactly 0 (exp(-4.5) ~= 0.011).
_KERNEL_SIGMAS = 3.0
# Coarse-level budget: at most this many scan points are used when scoring the full window.
_COARSE_MAX_POINTS = 200
# Typical lever arm (m) used to pick a coarse angular step that moves points ~one xy step.
_LEVER_ARM_M = 2.5
# Number of distinct coarse peaks refined on the fine field (guards against local maxima).
_N_PEAKS = 3
_FINEST_XY_STEP_M = 0.002
_FINEST_THETA_STEP_RAD = 0.0008
# Placeholder "nearest point" for cells with no reference point in reach (field value 0).
_FAR = 1.0e4


class LikelihoodField:
    """Likelihood raster; cell (i, j) is centred at (x0 + j*res, y0 + i*res).

    Besides ``values`` (the field sampled at cell centres) each cell remembers its nearest
    reference point. Queries look up the cell containing the point and evaluate
    ``exp(-d^2 / (2 sigma^2))`` with the *exact* distance to that cell's nearest reference
    point. Interpolating the raster instead (bilinear) pulls axis-aligned walls towards
    cell centres ("pixel locking"); on the 60-step test that bias accumulated to ~5 cm of
    drift at 5 cm cells, versus well under 1 cm with the exact lookup.
    """

    def __init__(self, values: np.ndarray, nearest_xy: np.ndarray, x0: float, y0: float,
                 resolution_m: float, sigma_m: float, ref_points: np.ndarray, margin_m: float) -> None:
        self.values = values
        self.nearest_xy = nearest_xy  # (H*W, 2) float32; far away for cells with no point in reach
        self.x0 = float(x0)
        self.y0 = float(y0)
        self.resolution_m = float(resolution_m)
        self.sigma_m = float(sigma_m)
        self._ref_points = ref_points
        self._margin_m = float(margin_m)
        self._coarse: LikelihoodField | None = None

    @classmethod
    def from_points(cls, points_xy: np.ndarray, resolution_m: float = 0.05, sigma_m: float = 0.10,
                    margin_m: float = 1.0) -> "LikelihoodField":
        """Build the field by splatting an exact-distance kernel (radius 3 sigma) around each point.

        Reference points are first merged per raster cell (mean position), so each kernel
        offset touches every target cell at most once and a plain vectorised ``minimum``
        works (no ``ufunc.at``). Beyond the kernel radius the field is exactly 0.
        """
        if resolution_m <= 0 or sigma_m <= 0:
            raise ValueError("resolution_m and sigma_m must be positive")
        pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        res = float(resolution_m)
        if len(pts) == 0:
            return cls(np.zeros((1, 1), np.float32), np.full((1, 2), _FAR, np.float32),
                       0.0, 0.0, res, sigma_m, pts, margin_m)
        # Pad by at least the kernel radius + 3 cells so the raster border is exactly 0;
        # the matcher relies on this to clip out-of-raster lookups instead of masking them.
        pad = max(float(margin_m), _KERNEL_SIGMAS * sigma_m + 3.0 * res)
        lo = pts.min(axis=0) - pad
        hi = pts.max(axis=0) + pad
        x0, y0 = float(lo[0]), float(lo[1])
        w = int(math.ceil((hi[0] - x0) / res)) + 1
        h = int(math.ceil((hi[1] - y0) / res)) + 1

        # Merge points per cell.
        ix = np.rint((pts[:, 0] - x0) / res).astype(np.int64)
        iy = np.rint((pts[:, 1] - y0) / res).astype(np.int64)
        uniq, inv = np.unique(iy * w + ix, return_inverse=True)
        cnt = np.bincount(inv).astype(np.float64)
        rep = np.column_stack((np.bincount(inv, pts[:, 0]) / cnt, np.bincount(inv, pts[:, 1]) / cnt))
        cy, cx = np.divmod(uniq, w)

        kr = int(math.ceil(_KERNEL_SIGMAS * sigma_m / res))
        r2max = (_KERNEL_SIGMAS * sigma_m) ** 2
        d2 = np.full(h * w, np.inf)
        near = np.full(h * w, -1, dtype=np.int64)
        idx = np.arange(len(rep))
        for oy in range(-kr, kr + 1):
            ty = cy + oy
            for ox in range(-kr, kr + 1):
                tx = cx + ox
                ok = (tx >= 0) & (tx < w) & (ty >= 0) & (ty < h)
                txo, tyo = tx[ok], ty[ok]
                dd = (x0 + txo * res - rep[ok, 0]) ** 2 + (y0 + tyo * res - rep[ok, 1]) ** 2
                f = tyo * w + txo
                better = dd < d2[f]
                f = f[better]
                d2[f] = dd[better]
                near[f] = idx[ok][better]
        reach = (near >= 0) & (d2 <= r2max)
        d2 = np.where(reach, d2, np.inf)
        values = np.exp(-d2 / (2.0 * sigma_m * sigma_m)).astype(np.float32).reshape(h, w)
        nearest = np.full((h * w, 2), _FAR, dtype=np.float32)
        nearest[reach] = rep[near[reach]]
        return cls(values, nearest, x0, y0, res, sigma_m, rep, margin_m)

    @property
    def coarse(self) -> "LikelihoodField":
        """A 2x-coarser, 2x-wider field over the same points (cached), used for the global search."""
        if self._coarse is None:
            self._coarse = LikelihoodField.from_points(
                self._ref_points, 2.0 * self.resolution_m, 2.0 * self.sigma_m, self._margin_m)
        return self._coarse

    def _cells(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        h, w = self.values.shape
        jx = np.clip(np.rint((x - self.x0) / self.resolution_m), 0, w - 1).astype(np.int64)
        jy = np.clip(np.rint((y - self.y0) / self.resolution_m), 0, h - 1).astype(np.int64)
        return jy * w + jx

    def sample(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Field values at arbitrary-shaped coordinate arrays (0 outside the raster)."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        nb = self.nearest_xy[self._cells(x, y)]
        d2 = (x - nb[..., 0]) ** 2 + (y - nb[..., 1]) ** 2
        # cells off the raster or out of kernel reach hold a far-away point -> exactly 0
        return np.exp(-d2 / (2.0 * self.sigma_m ** 2))

    def score(self, points_xy: np.ndarray) -> float:
        """Mean field value at the points (0 for points outside the raster; 0.0 if empty)."""
        p = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if len(p) == 0:
            return 0.0
        return float(self.sample(p[:, 0], p[:, 1]).mean())


@dataclass(frozen=True)
class MatchResult:
    pose: Pose2D
    score: float
    converged: bool


def _subsample(pts: np.ndarray, n_max: int) -> np.ndarray:
    if len(pts) <= n_max:
        return pts
    idx = np.linspace(0, len(pts) - 1, n_max).round().astype(np.int64)
    return pts[idx]


def _score_grid(field: LikelihoodField, pts: np.ndarray, xs: np.ndarray, ys: np.ndarray,
                thetas: np.ndarray, exact: bool = True) -> np.ndarray:
    """Scores of every (theta, x, y) candidate pose in one vectorised pass; shape (T, X, Y).

    Per-axis work is done on (T, X, N) and (T, Y, N) arrays and only combined by
    broadcasting, so the (T, X, Y, N) work is one gather (+ the exact-distance kernel).
    ``exact=False`` returns the cell-centre raster value (cheaper; used for the coarse level).
    """
    c, s = np.cos(thetas)[:, None], np.sin(thetas)[:, None]
    rx = c * pts[None, :, 0] - s * pts[None, :, 1]  # (T, N)
    ry = s * pts[None, :, 0] + c * pts[None, :, 1]
    h, w = field.values.shape
    inv_res = 1.0 / field.resolution_m
    wx = (rx[:, None, :] + xs[None, :, None]).astype(np.float32)  # (T, X, N) world x
    wy = (ry[:, None, :] + ys[None, :, None]).astype(np.float32)  # (T, Y, N) world y
    # Clipping is exact: the raster border holds no reference point in reach (see from_points).
    jx = np.clip(np.rint((wx - np.float32(field.x0)) * np.float32(inv_res)), 0, w - 1).astype(np.int32)
    jy = np.clip(np.rint((wy - np.float32(field.y0)) * np.float32(inv_res)), 0, h - 1).astype(np.int32)
    base = jy[:, None, :, :] * w + jx[:, :, None, :]  # (T, X, Y, N)
    if not exact:
        return field.values.ravel()[base].mean(axis=-1)
    nb = field.nearest_xy[base]  # (T, X, Y, N, 2)
    dx = wx[:, :, None, :] - nb[..., 0]
    dy = wy[:, None, :, :] - nb[..., 1]
    d2 = dx * dx + dy * dy
    val = np.exp(d2 * np.float32(-0.5 / field.sigma_m ** 2))
    return val.mean(axis=-1)


def match_scan(field: LikelihoodField, scan_points_xy: np.ndarray, initial: Pose2D,
               search_xy_m: float = 0.3, search_theta_rad: float = 0.35,
               min_score: float = 0.35) -> MatchResult:
    """Pose of the scan (robot) in the field's frame, searched around ``initial``.

    Level 0 scores the whole window on the coarse field with a subsample of points;
    the best few distinct peaks are then refined on the fine field with all points by
    repeated 5x5x5 grids of halving step, clipped to the window. ``converged`` requires
    score >= ``min_score`` and a best pose not pinned to the window edge.
    """
    pts = np.asarray(scan_points_xy, dtype=np.float64).reshape(-1, 2)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    if len(pts) == 0:
        return MatchResult(initial, 0.0, False)

    # ---- level 0: full window, coarse field, vectorised over all candidates
    cf = field.coarse
    step_xy = min(cf.resolution_m * 0.5, search_xy_m) if search_xy_m > 0 else 0.0
    step_th = min(step_xy / _LEVER_ARM_M, search_theta_rad) if search_theta_rad > 0 else 0.0
    def axis(half: float, step: float) -> np.ndarray:
        if half <= 0 or step <= 0:
            return np.zeros(1)
        n = int(math.ceil(half / step))
        return np.linspace(-half, half, 2 * n + 1)
    oxs, oys, ots = axis(search_xy_m, step_xy), axis(search_xy_m, step_xy), axis(search_theta_rad, step_th)
    coarse = _score_grid(cf, _subsample(pts, _COARSE_MAX_POINTS),
                         initial.x + oxs, initial.y + oys, initial.theta + ots, exact=False)

    # pick distinct peaks (non-maximum suppression within 2 steps)
    order = np.argsort(coarse, axis=None)[::-1][:64]
    peaks: list[tuple[int, int, int]] = []
    for flat in order:
        t, i, j = np.unravel_index(flat, coarse.shape)
        if all(abs(t - pt) > 2 or abs(i - pi) > 2 or abs(j - pj) > 2 for pt, pi, pj in peaks):
            peaks.append((int(t), int(i), int(j)))
            if len(peaks) == _N_PEAKS:
                break

    # ---- refinement on the fine field with all points: one 5x5x5 level per peak, then
    # keep halving the step around the best peak only.
    k = np.arange(-2, 3)

    def refine(dx: float, dy: float, dt: float, sxy: float, sth: float):
        cx = np.unique(np.clip(dx + k * sxy, -search_xy_m, search_xy_m))
        cy = np.unique(np.clip(dy + k * sxy, -search_xy_m, search_xy_m))
        ct = np.unique(np.clip(dt + k * sth, -search_theta_rad, search_theta_rad))
        g = _score_grid(field, pts, initial.x + cx, initial.y + cy, initial.theta + ct)
        a, b, c = np.unravel_index(int(np.argmax(g)), g.shape)
        return float(g[a, b, c]), cx[b], cy[c], ct[a]

    sxy, sth = step_xy * 0.5, step_th * 0.5
    best = max((refine(oxs[i], oys[j], ots[t], sxy, sth) for t, i, j in peaks), key=lambda r: r[0])
    while sxy > _FINEST_XY_STEP_M or sth > _FINEST_THETA_STEP_RAD:
        sxy, sth = max(sxy * 0.5, _FINEST_XY_STEP_M), max(sth * 0.5, _FINEST_THETA_STEP_RAD)
        best = refine(best[1], best[2], best[3], sxy, sth)

    sc, dx, dy, dt = best
    tol_xy, tol_th = 0.5 * _FINEST_XY_STEP_M, 0.5 * _FINEST_THETA_STEP_RAD
    on_edge = ((search_xy_m > 0 and (abs(dx) >= search_xy_m - tol_xy or abs(dy) >= search_xy_m - tol_xy))
               or (search_theta_rad > 0 and abs(dt) >= search_theta_rad - tol_th))
    pose = Pose2D(initial.x + dx, initial.y + dy, initial.theta + dt)
    return MatchResult(pose, sc, bool(sc >= min_score and not on_edge))
