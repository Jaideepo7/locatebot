"""Frontier exploration on a ``Costmap``.

- A frontier cell is a FREE cell with an UNKNOWN cell in its 8-neighbourhood (the grid edge is
  not unknown) and no OCCUPIED cell within 2 cells (filters holes in walls). Frontier cells are clustered by 8-connectivity; small clusters are ignored.
- Each cluster gets a goal: the traversable cell reachable from the car (8-connected BFS over the
  costmap) that is within ``reach_radius_m`` of a cluster cell and nearest the cluster centroid.
  Unknown space is inflated in the costmap, so the goal sits a little short of the frontier; the
  LiDAR sees past it. Clusters without such a cell are unreachable and skipped.
- Score = ``info_weight * frontier length (m) - distance_weight * path distance (m)
  - heading_weight * |bearing| * min_turn_radius_m``. Goals closer than ``min_goal_dist_m``
  are skipped (the car already looked from there), as are goals near ones already visited or
  failed (``FrontierExplorer``). ``next_goal`` returns None when nothing is left: exploration is
  done (or everything left is unreachable).

``explored_fraction``: ``known / (known + U)`` where known = FREE + OCCUPIED cells and U = the
UNKNOWN cells 4-connected (through UNKNOWN cells) to some frontier cell. Unknown space sealed off
by walls (outside the room, inside furniture) does not count, so a closed, fully mapped room is
1.0 and a map without FREE cells is 0.0. It is an estimate: while walls are still open, U includes unknown
space outside the room, so the value is low early and rises sharply as the room closes.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from locatebot.lidar.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from locatebot.lidar.types import Pose2D

from .costmap import Costmap
from .planner import reachable_steps, start_cell


@dataclass(frozen=True)
class ExploreConfig:
    min_frontier_cells: int = 6
    reach_radius_m: float = 0.6        # goal may sit this far from the frontier cluster
    min_goal_dist_m: float = 0.5       # skip goals this close to the car
    info_weight: float = 1.0           # per metre of frontier
    distance_weight: float = 1.0       # per metre of path
    heading_weight: float = 0.5        # per radian of bearing, scaled by min_turn_radius_m
    blacklist_radius_m: float = 0.4    # goals near visited/failed ones are skipped
    max_samples: int = 16              # cluster cells used to look for a goal


@dataclass(frozen=True)
class Frontier:
    goal_xy: tuple[float, float]
    centroid_xy: tuple[float, float]
    size_cells: int
    distance_m: float
    score: float
    cells: np.ndarray = field(repr=False, compare=False)   # (N, 2) (col, row)


def _classes(grid_or_classes) -> np.ndarray:
    if isinstance(grid_or_classes, OccupancyGrid):
        return grid_or_classes.classify()
    if isinstance(grid_or_classes, Costmap):
        return grid_or_classes.classes
    return np.asarray(grid_or_classes, dtype=np.uint8)


def _shift_any(mask: np.ndarray, eight: bool) -> np.ndarray:
    """Cells with a True neighbour (4- or 8-neighbourhood), no wrap-around."""
    p = np.pad(mask, 1, constant_values=False)
    h, w = mask.shape
    out = p[0:h, 1:w + 1] | p[2:h + 2, 1:w + 1] | p[1:h + 1, 0:w] | p[1:h + 1, 2:w + 2]
    if eight:
        out = out | p[0:h, 0:w] | p[0:h, 2:w + 2] | p[2:h + 2, 0:w] | p[2:h + 2, 2:w + 2]
    return out


def frontier_mask(grid_or_classes, occ_clearance_cells: int = 2) -> np.ndarray:
    """bool [row, col]: FREE cells with an UNKNOWN 8-neighbour and no OCCUPIED cell within
    ``occ_clearance_cells`` (Chebyshev). The clearance rule drops the false frontiers in front of
    small holes in a wall (cells missed at grazing angles): a hole of up to
    ``2 * occ_clearance_cells`` cells is too narrow for the car anyway."""
    c = _classes(grid_or_classes)
    m = (c == FREE) & _shift_any(c == UNKNOWN, eight=True)
    near_occ = c == OCCUPIED
    for _ in range(max(0, int(occ_clearance_cells))):
        near_occ = near_occ | _shift_any(near_occ, eight=True)
    return m & ~near_occ


def cluster_frontiers(mask: np.ndarray) -> list[np.ndarray]:
    """8-connected components of a frontier mask, each as an (N, 2) int array of (col, row)."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    rows, cols = np.nonzero(mask)
    m = mask.tolist()
    sv = seen.tolist()
    out = []
    for r0, c0 in zip(rows.tolist(), cols.tolist()):
        if sv[r0][c0]:
            continue
        sv[r0][c0] = True
        q = deque([(r0, c0)])
        comp = []
        while q:
            r, c = q.popleft()
            comp.append((c, r))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < h and 0 <= cc < w and m[rr][cc] and not sv[rr][cc]:
                        sv[rr][cc] = True
                        q.append((rr, cc))
        out.append(np.array(comp, dtype=np.int64))
    return out


def explored_fraction(grid_or_classes) -> float:
    """known / (known + unknown cells connected to a frontier); see the module docstring."""
    c = _classes(grid_or_classes)
    known = int(np.count_nonzero((c == FREE) | (c == OCCUPIED)))
    if known == 0 or not np.any(c == FREE):
        return 0.0  # nothing known to be free yet: no estimate of the reachable area
    unk = c == UNKNOWN
    front = frontier_mask(c)
    reach = unk & _shift_any(front, eight=True)
    while True:
        grown = reach | (unk & _shift_any(reach, eight=False))
        if np.array_equal(grown, reach):
            break
        reach = grown
    u = int(np.count_nonzero(reach))
    return known / (known + u)


def find_frontiers(costmap: Costmap, start_xy, config: ExploreConfig | None = None,
                   start_theta: float | None = None, exclude=()) -> list[Frontier]:
    """Reachable frontier clusters with goals, best score first. ``exclude``: goal points to
    avoid (within ``blacklist_radius_m``)."""
    cfg = config or ExploreConfig()
    sc = start_cell(costmap, start_xy)
    if sc is None:
        return []
    steps = reachable_steps(costmap, sc)
    reach = steps >= 0
    res = costmap.resolution_m
    k = int(math.ceil(cfg.reach_radius_m / res))
    ox, oy = costmap.origin_xy
    rmin = costmap.vehicle.min_turn_radius_m
    excl = np.asarray(list(exclude), dtype=np.float64).reshape(-1, 2)
    out = []
    for cells in cluster_frontiers(frontier_mask(costmap.classes)):
        if len(cells) < cfg.min_frontier_cells:
            continue
        cxy = np.array([ox, oy]) + (cells + 0.5) * res
        centroid = cxy.mean(axis=0)
        idx = np.linspace(0, len(cells) - 1, min(cfg.max_samples, len(cells))).round().astype(int)
        samples = cxy[idx]
        c0, r0 = cells.min(axis=0) - k
        c1, r1 = cells.max(axis=0) + k + 1
        c0, r0 = max(0, c0), max(0, r0)
        c1, r1 = min(costmap.width, c1), min(costmap.height, r1)
        rr, cc = np.nonzero(reach[r0:r1, c0:c1])
        if len(rr) == 0:
            continue
        rr += r0
        cc += c0
        cand = np.array([ox, oy]) + (np.column_stack((cc, rr)) + 0.5) * res
        dmin = np.min(np.hypot(cand[:, None, 0] - samples[None, :, 0], cand[:, None, 1] - samples[None, :, 1]), axis=1)
        ok = dmin <= cfg.reach_radius_m
        if len(excl):
            dex = np.min(np.hypot(cand[:, None, 0] - excl[None, :, 0], cand[:, None, 1] - excl[None, :, 1]), axis=1)
            ok &= dex > cfg.blacklist_radius_m
        if not np.any(ok):
            continue
        dc = np.hypot(cand[:, 0] - centroid[0], cand[:, 1] - centroid[1])
        dc = np.where(ok, dc, np.inf)
        i = int(np.argmin(dc))
        goal = (float(cand[i, 0]), float(cand[i, 1]))
        euclid = math.hypot(goal[0] - start_xy[0], goal[1] - start_xy[1])
        dist = max(euclid, steps[rr[i], cc[i]] * res)
        if dist < cfg.min_goal_dist_m:
            continue
        score = cfg.info_weight * len(cells) * res - cfg.distance_weight * dist
        if start_theta is not None:
            bearing = math.atan2(goal[1] - start_xy[1], goal[0] - start_xy[0]) - start_theta
            bearing = abs((bearing + math.pi) % (2 * math.pi) - math.pi)
            score -= cfg.heading_weight * bearing * rmin
        out.append(Frontier(goal, (float(centroid[0]), float(centroid[1])), len(cells), dist, score, cells))
    out.sort(key=lambda f: -f.score)
    return out


class FrontierExplorer:
    """Stateful goal picker: remembers visited and failed goals so it cannot loop forever.

    A visited goal is excluded for good. A failed goal is excluded while the car is within
    ``retry_distance_m`` of where it failed (a forward-only car often cannot plan from one pose
    but can from another), and for good once it has failed ``max_failures`` times."""

    def __init__(self, config: ExploreConfig | None = None, retry_distance_m: float = 1.0,
                 max_failures: int = 2) -> None:
        self.config = config or ExploreConfig()
        self.retry_distance_m = retry_distance_m
        self.max_failures = max_failures
        self.visited: list[tuple[float, float]] = []
        self.failed: list[tuple[tuple[float, float], tuple[float, float] | None]] = []

    def excluded(self, pose: Pose2D) -> list[tuple[float, float]]:
        out = list(self.visited)
        rad = self.config.blacklist_radius_m
        for goal, where in self.failed:
            n = sum(1 for g, _ in self.failed if math.hypot(g[0] - goal[0], g[1] - goal[1]) <= rad)
            near = where is None or math.hypot(pose.x - where[0], pose.y - where[1]) <= self.retry_distance_m
            if n >= self.max_failures or near:
                out.append(goal)
        return out

    def next_goal(self, costmap: Costmap, pose: Pose2D) -> Frontier | None:
        fr = find_frontiers(costmap, (pose.x, pose.y), self.config, start_theta=pose.theta,
                            exclude=self.excluded(pose))
        return fr[0] if fr else None

    def mark_visited(self, goal_xy) -> None:
        self.visited.append((float(goal_xy[0]), float(goal_xy[1])))

    def mark_failed(self, goal_xy, from_xy=None) -> None:
        """Record a failed goal; ``from_xy`` = where the car was (None: exclude for good)."""
        where = None if from_xy is None else (float(from_xy[0]), float(from_xy[1]))
        self.failed.append(((float(goal_xy[0]), float(goal_xy[1])), where))

    def goal_still_useful(self, costmap: Costmap, frontier: Frontier) -> bool:
        """False once the goal is blocked or no frontier cell remains near it."""
        if not costmap.is_traversable_xy(frontier.goal_xy):
            return False
        res = costmap.resolution_m
        rad = self.config.reach_radius_m + res
        k = int(math.ceil(rad / res))
        gc, gr = costmap.world_to_cell(frontier.goal_xy)
        r0, r1 = max(0, gr - k), min(costmap.height, gr + k + 1)
        c0, c1 = max(0, gc - k), min(costmap.width, gc + k + 1)
        m = frontier_mask(costmap.classes[max(0, r0 - 1):r1 + 1, max(0, c0 - 1):c1 + 1])
        rr, cc = np.nonzero(m)
        if len(rr) == 0:
            return False
        rr += max(0, r0 - 1)
        cc += max(0, c0 - 1)
        x = costmap.origin_xy[0] + (cc + 0.5) * res
        y = costmap.origin_xy[1] + (rr + 0.5) * res
        return bool(np.any(np.hypot(x - frontier.goal_xy[0], y - frontier.goal_xy[1]) <= rad))
