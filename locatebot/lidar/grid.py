"""Log-odds occupancy grid with vectorised ray integration.

Grid layout: ``log_odds[row, col]`` with row = y, col = x; ``origin_xy`` is the world position of
cell (0, 0)'s lower-left corner. Tests use synthetic scans from ``locatebot.lidar.sim``: they prove
software behaviour, not real-sensor behaviour.
"""
from __future__ import annotations

import math

import numpy as np

from .types import RANGE_MAX_M, RANGE_MIN_M, Pose2D, Scan

FREE, OCCUPIED, UNKNOWN = 0, 100, 255


class OccupancyGrid:
    def __init__(
        self,
        width_cells: int,
        height_cells: int,
        resolution_m: float = 0.05,
        origin_xy: tuple[float, float] = (0.0, 0.0),
        l_occ: float = 0.85,
        l_free: float = -0.4,
        l_min: float = -4.0,
        l_max: float = 4.0,
    ) -> None:
        if width_cells <= 0 or height_cells <= 0:
            raise ValueError("grid dimensions must be positive")
        if not resolution_m > 0:
            raise ValueError("resolution_m must be positive")
        self.width = int(width_cells)
        self.height = int(height_cells)
        self.resolution_m = float(resolution_m)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    @classmethod
    def around(cls, center_xy: tuple[float, float], size_m: float, resolution_m: float = 0.05) -> "OccupancyGrid":
        """Square grid of side >= size_m centred on center_xy."""
        n = int(math.ceil(size_m / resolution_m - 1e-9))
        half = n * resolution_m / 2.0
        return cls(n, n, resolution_m, (center_xy[0] - half, center_xy[1] - half))

    # --- coordinates -------------------------------------------------------------------------
    def world_to_cell(self, xy: np.ndarray) -> np.ndarray:
        """(N, 2) world xy -> (N, 2) int64 (col, row), floor division (no bounds check)."""
        p = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        o = np.asarray(self.origin_xy)
        return np.floor((p - o) / self.resolution_m).astype(np.int64)

    def cell_to_world(self, cr: np.ndarray) -> np.ndarray:
        """(N, 2) (col, row) -> (N, 2) world xy of cell centres."""
        c = np.asarray(cr, dtype=np.float64).reshape(-1, 2)
        return np.asarray(self.origin_xy) + (c + 0.5) * self.resolution_m

    def in_bounds(self, cr: np.ndarray) -> np.ndarray:
        c = np.asarray(cr).reshape(-1, 2)
        return (c[:, 0] >= 0) & (c[:, 0] < self.width) & (c[:, 1] >= 0) & (c[:, 1] < self.height)

    def _flat(self, cr: np.ndarray) -> np.ndarray:
        """Flattened indices of the in-bounds cells of cr."""
        cr = cr[self.in_bounds(cr)]
        return cr[:, 1] * self.width + cr[:, 0]

    # --- updates -----------------------------------------------------------------------------
    def integrate(self, scan: Scan, pose: Pose2D, max_range_m: float = RANGE_MAX_M) -> None:
        """Integrate one scan taken at ``pose`` (robot pose in the map frame).

        Valid returns (finite, within [RANGE_MIN_M, max_range_m]) mark their endpoint cell with
        l_occ and every cell sampled along the ray before it with l_free. Each cell gets at most one
        update of each kind per scan; a cell hit by any endpoint in this scan is not also freed.
        NaN / out-of-range beams change nothing. Cells outside the grid are ignored.
        """
        mask = scan.valid_mask(RANGE_MIN_M, max_range_m)
        if not np.any(mask):
            return
        r = scan.ranges_m[mask]
        a = scan.angles_rad[mask] + pose.theta
        dx, dy = np.cos(a), np.sin(a)
        ends = np.column_stack((pose.x + r * dx, pose.y + r * dy))

        # Sample every ray at resolution/2 steps for t in [0, r): (N, K) samples, masked per beam.
        step = self.resolution_m / 2.0
        k = int(math.ceil(float(r.max()) / step))
        t = np.arange(k, dtype=np.float64) * step
        take = t[None, :] < r[:, None]
        tt = np.broadcast_to(t, take.shape)[take]
        rows = np.nonzero(take)[0]
        samples = np.column_stack((pose.x + tt * dx[rows], pose.y + tt * dy[rows]))

        size = self.width * self.height
        occ = np.zeros(size, dtype=bool)
        occ[self._flat(self.world_to_cell(ends))] = True
        free = np.zeros(size, dtype=bool)
        free[self._flat(self.world_to_cell(samples))] = True  # boolean scatter == dedupe
        free &= ~occ

        lo = self.log_odds.reshape(-1)  # view
        lo[free] += np.float32(self.l_free)
        lo[occ] += np.float32(self.l_occ)
        touched = free | occ
        lo[touched] = np.clip(lo[touched], self.l_min, self.l_max)

    # --- queries -----------------------------------------------------------------------------
    def probability(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def classify(self, occ_threshold: float = 0.65, free_threshold: float = 0.35) -> np.ndarray:
        """uint8 map: OCCUPIED where p > occ_threshold, FREE where p < free_threshold, else UNKNOWN."""
        p = self.probability()
        out = np.full(p.shape, UNKNOWN, dtype=np.uint8)
        out[p < free_threshold] = FREE
        out[p > occ_threshold] = OCCUPIED
        return out
