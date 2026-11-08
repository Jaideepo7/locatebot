"""Planning costmap built from an occupancy grid.

Rules:
- Only FREE cells can be traversable. UNKNOWN is never free (NaN range = unknown).
- OCCUPIED cells are inflated by ``half_width_m + margin_m``; UNKNOWN cells (and, when
  ``inflate_unknown`` is set, everything outside the grid) are inflated by ``unknown_inflation_m``
  (default: the same radius). A cell is traversable when the car's centre may stand there.
- ``lethal`` marks cells where the car body (radius ``half_width_m`` plus half a cell of
  discretisation slack) could overlap an occupied/unknown cell. ``traversable`` is stricter (it includes the margin).
- A soft cost in [0, soft_weight] grows quadratically as the clearance beyond the inflation
  radius drops below ``soft_range_m``. The planner multiplies step length by ``1 + cost``.

Clearances are exact Euclidean distances between cell centres (capped at
``clearance_cap_m``), computed with a separable distance transform (numpy only).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from locatebot.lidar.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from locatebot.mission.types import VehicleConfig


@dataclass(frozen=True)
class CostmapConfig:
    margin_m: float = 0.05             # extra clearance beyond half_width_m around obstacles
    inflate_unknown: bool = True       # treat UNKNOWN (and off-grid) as obstacles for inflation
    unknown_inflation_m: float | None = None  # None -> half_width_m + margin_m
    soft_range_m: float = 0.35         # soft cost fades out this far beyond the inflation radius
    soft_weight: float = 4.0           # soft cost at the inflation boundary
    occ_threshold: float = 0.65        # passed to OccupancyGrid.classify
    free_threshold: float = 0.35

    def __post_init__(self) -> None:
        if self.margin_m < 0 or self.soft_range_m <= 0 or self.soft_weight < 0:
            raise ValueError("margin_m and soft_weight must be >= 0, soft_range_m > 0")
        if self.unknown_inflation_m is not None and self.unknown_inflation_m < 0:
            raise ValueError("unknown_inflation_m must be >= 0")


@dataclass(frozen=True, eq=False)
class Costmap:
    """Arrays are indexed ``[row, col]`` (row = y, col = x), like ``OccupancyGrid``."""

    classes: np.ndarray        # uint8 FREE / OCCUPIED / UNKNOWN
    traversable: np.ndarray    # bool: the car's centre may be here
    lethal: np.ndarray         # bool: the car body would overlap an obstacle/unknown cell
    clearance_m: np.ndarray    # float: distance to the nearest blocking cell, capped
    occ_clearance_m: np.ndarray  # float: distance to the nearest OCCUPIED cell, capped
    cost: np.ndarray           # float: soft cost >= 0, inf where not traversable
    resolution_m: float
    origin_xy: tuple[float, float]
    vehicle: VehicleConfig
    config: CostmapConfig
    clearance_cap_m: float
    inflation_m: float = field(default=0.0)

    @property
    def width(self) -> int:
        return self.classes.shape[1]

    @property
    def height(self) -> int:
        return self.classes.shape[0]

    @property
    def occupied(self) -> np.ndarray:
        return self.classes == OCCUPIED

    @property
    def free(self) -> np.ndarray:
        return self.classes == FREE

    def world_to_cell(self, xy) -> tuple[int, int]:
        """World (x, y) -> (col, row), floor division, no bounds check."""
        return (int(math.floor((xy[0] - self.origin_xy[0]) / self.resolution_m)),
                int(math.floor((xy[1] - self.origin_xy[1]) / self.resolution_m)))

    def cell_to_world(self, cr) -> tuple[float, float]:
        """(col, row) -> world (x, y) of the cell centre."""
        return (self.origin_xy[0] + (cr[0] + 0.5) * self.resolution_m,
                self.origin_xy[1] + (cr[1] + 0.5) * self.resolution_m)

    def in_bounds(self, cr) -> bool:
        return 0 <= cr[0] < self.width and 0 <= cr[1] < self.height

    def is_traversable_xy(self, xy) -> bool:
        c, r = self.world_to_cell(xy)
        return self.in_bounds((c, r)) and bool(self.traversable[r, c])

    def clearance_at(self, xy) -> float:
        """Clearance (m) at a world point; 0 outside the grid."""
        c, r = self.world_to_cell(xy)
        return float(self.clearance_m[r, c]) if self.in_bounds((c, r)) else 0.0


def distance_transform(blocked: np.ndarray, cap_cells: int) -> np.ndarray:
    """Exact Euclidean distance (in cells) from each cell to the nearest ``blocked`` cell.

    Values above ``cap_cells`` are clipped to ``cap_cells``. Separable: a 1-D vertical pass
    (exact distance to the nearest blocked cell in the same column), then a minimum over
    horizontal offsets |dx| <= cap_cells of sqrt(dx^2 + g^2).
    """
    h, w = blocked.shape
    big = cap_cells + 1
    g = np.full((h, w), big, dtype=np.int64)
    run = np.full(w, big, dtype=np.int64)
    for r in range(h):
        run = np.where(blocked[r], 0, np.minimum(run + 1, big))
        g[r] = run
    run = np.full(w, big, dtype=np.int64)
    for r in range(h - 1, -1, -1):
        run = np.where(blocked[r], 0, np.minimum(run + 1, big))
        g[r] = np.minimum(g[r], run)
    g2 = g.astype(np.float64) ** 2
    pad = np.full((h, w + 2 * cap_cells), float(big) ** 2)
    pad[:, cap_cells:cap_cells + w] = g2
    best = g2.copy()
    for dx in range(1, cap_cells + 1):
        dd = float(dx * dx)
        best = np.minimum(best, pad[:, cap_cells + dx:cap_cells + dx + w] + dd)
        best = np.minimum(best, pad[:, cap_cells - dx:cap_cells - dx + w] + dd)
    return np.minimum(np.sqrt(best), float(cap_cells))


def costmap_from_classes(classes: np.ndarray, resolution_m: float, origin_xy: tuple[float, float],
                         vehicle: VehicleConfig | None = None, config: CostmapConfig | None = None) -> Costmap:
    """Build a costmap from a ``classify()``-style uint8 array."""
    vehicle = vehicle or VehicleConfig()
    config = config or CostmapConfig()
    classes = np.array(classes, dtype=np.uint8, copy=True)
    res = float(resolution_m)
    occ = classes == OCCUPIED
    unk = classes == UNKNOWN
    free = classes == FREE
    r_occ = vehicle.half_width_m + config.margin_m
    r_unk = r_occ if config.unknown_inflation_m is None else config.unknown_inflation_m
    cap_m = max(r_occ, r_unk) + config.soft_range_m
    cap = int(math.ceil(cap_m / res)) + 1

    def dist_m(blocked: np.ndarray, border: bool) -> np.ndarray:
        if border:  # off-grid counts as blocked: pad by one cell
            p = np.pad(blocked, 1, constant_values=True)
            return distance_transform(p, cap)[1:-1, 1:-1] * res
        return distance_transform(blocked, cap) * res

    d_occ = dist_m(occ, False)
    if config.inflate_unknown:
        d_unk = dist_m(unk, True)
    else:
        d_unk = np.full(classes.shape, cap * res)
    eps = 1e-9
    traversable = free & (d_occ > r_occ + eps) & (d_unk > r_unk + eps)
    # Half a cell of slack: distances are between cell centres, the car and the obstacle can be
    # anywhere inside their cells.
    r_lethal = vehicle.half_width_m + 0.5 * res
    lethal = occ | (d_occ < r_lethal - eps)
    if config.inflate_unknown:
        lethal |= unk | (d_unk < min(r_lethal, r_unk) - eps)
    # Soft cost measured from whichever inflation boundary is nearer.
    beyond = np.minimum(d_occ - r_occ, d_unk - r_unk)
    frac = np.clip((config.soft_range_m - beyond) / config.soft_range_m, 0.0, 1.0)
    cost = config.soft_weight * frac ** 2
    cost = np.where(traversable, cost, np.inf)
    clearance = np.minimum(d_occ, d_unk) if config.inflate_unknown else d_occ
    clearance = np.minimum(clearance, cap * res)
    return Costmap(classes=classes, traversable=traversable, lethal=lethal, clearance_m=clearance,
                   occ_clearance_m=np.minimum(d_occ, cap * res),
                   cost=cost, resolution_m=res, origin_xy=(float(origin_xy[0]), float(origin_xy[1])),
                   vehicle=vehicle, config=config, clearance_cap_m=cap * res, inflation_m=r_occ)


def build_costmap(grid: OccupancyGrid, vehicle: VehicleConfig | None = None,
                  config: CostmapConfig | None = None) -> Costmap:
    """Build a costmap from an ``OccupancyGrid`` (classified with ``config`` thresholds)."""
    config = config or CostmapConfig()
    classes = grid.classify(config.occ_threshold, config.free_threshold)
    return costmap_from_classes(classes, grid.resolution_m, grid.origin_xy, vehicle, config)
