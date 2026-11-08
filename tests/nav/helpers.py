"""Shared helpers for the nav tests. Everything here is synthetic (no hardware)."""
from __future__ import annotations

import numpy as np

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.sim import Room

RES = 0.05
FREE_LO = -3.0
OCC_LO = 3.0


def grid_from_classes(free: np.ndarray, occ: np.ndarray, res: float = RES,
                      origin_xy: tuple[float, float] = (0.0, 0.0)) -> OccupancyGrid:
    """OccupancyGrid whose classify() gives FREE where ``free``, OCCUPIED where ``occ``, else UNKNOWN."""
    h, w = free.shape
    g = OccupancyGrid(w, h, res, origin_xy)
    g.log_odds[free] = FREE_LO
    g.log_odds[occ] = OCC_LO
    return g


def open_grid(w: int = 60, h: int = 60, res: float = RES) -> OccupancyGrid:
    """All-FREE grid with no obstacles."""
    free = np.ones((h, w), dtype=bool)
    return grid_from_classes(free, np.zeros_like(free), res)


def segment_distance(room: Room, x: float, y: float) -> float:
    """Exact distance from (x, y) to the nearest wall segment of ``room``."""
    s = room.segments
    p = np.array([x, y])
    a, b = s[:, :2], s[:, 2:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.min(np.linalg.norm(proj - p, axis=1)))


def rasterise_room(room: Room, grid: OccupancyGrid, inside: tuple[float, float, float, float]) -> OccupancyGrid:
    """Ground-truth map: cells within half a cell of a wall OCCUPIED, cells inside ``inside`` FREE."""
    h, w = grid.height, grid.width
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    xy = grid.cell_to_world(np.column_stack((cols.ravel(), rows.ravel())))
    x0, y0, x1, y1 = inside
    s = room.segments
    a, b = s[:, :2], s[:, 2:]
    ab = b - a
    d = np.full(len(xy), np.inf)
    for i in range(len(s)):
        t = np.clip(((xy - a[i]) @ ab[i]) / max(ab[i] @ ab[i], 1e-12), 0.0, 1.0)
        d = np.minimum(d, np.linalg.norm(a[i] + t[:, None] * ab[i] - xy, axis=1))
    occ = (d <= grid.resolution_m * 0.75).reshape(h, w)
    inb = ((xy[:, 0] > x0) & (xy[:, 0] < x1) & (xy[:, 1] > y0) & (xy[:, 1] < y1)).reshape(h, w)
    grid.log_odds[:] = 0.0
    grid.log_odds[inb] = FREE_LO
    grid.log_odds[occ] = OCC_LO
    return grid
