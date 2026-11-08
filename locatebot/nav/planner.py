"""A* path planning on a ``Costmap``.

- 8-connected grid A*, octile heuristic (admissible: every step costs ``length * (1 + cost)``
  with ``cost >= 0``). Diagonal moves need both orthogonal neighbours traversable.
- ``plan`` returns a world-frame polyline from ``start_xy`` to ``goal_xy``, shortcut with
  cost-aware line-of-sight checks and resampled to even spacing, or ``None``.
- Start escape: if the start cell is not traversable but not occupied/unknown (the car is parked
  inside the inflation band), the path first goes straight to the nearest traversable cell within
  ``escape_radius_m`` whose straight segment crosses no OCCUPIED or UNKNOWN cell.
- Heading-aware start (``start_theta``): a car cannot turn on the spot or reverse. If the grid
  path's first metre or so is not ``start_feasible`` (a point behind the car or inside a
  full-lock turning circle), a small Hybrid-A*-style search over forward constant-curvature
  motions (radius >= 1.05 x min turning radius, collision-checked on the costmap, guided by an
  exact grid cost-to-go) finds a manoeuvre after which the grid route is feasible, e.g. a loop
  round a piece of furniture. If none exists within ``max_expansions``, ``plan`` returns None:
  the car is stuck without reversing (check ``has_forward_exit``).
- Tracking check (``validate``): with a heading, the path is test-driven with pure pursuit on
  the bicycle model; tight corners the car cannot turn (e.g. a 90-degree bend at the end of a
  narrow slot) are blocked and the route replanned.
- Exit check (``ensure_exit``): the end is pulled back (up to ``max_trim_m``) so the car never
  stops where it cannot turn around (``can_turn_around``: a dead-end slot, facing a wall);
  ``plan`` returns None if no such end exists.
- ``plan_approach`` plans to the reachable traversable cell nearest to a goal that may itself be
  inside an obstacle (items sit on or against things), within ``standoff_m`` of it.
"""
from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import replace

import numpy as np

from locatebot.lidar.types import Pose2D

from .costmap import Costmap

SQRT2 = math.sqrt(2.0)
_NBRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]

Path = list[tuple[float, float]]


# --- grid search -----------------------------------------------------------------------------
def astar(costmap: Costmap, start_cell: tuple[int, int], goal_cell: tuple[int, int]
          ) -> tuple[list[tuple[int, int]], float] | None:
    """A* between two traversable cells ``(col, row)``. Returns (cells, cost in metres) or None."""
    w, h = costmap.width, costmap.height
    if not (costmap.in_bounds(start_cell) and costmap.in_bounds(goal_cell)):
        return None
    trav = costmap.traversable
    if not trav[start_cell[1], start_cell[0]] or not trav[goal_cell[1], goal_cell[0]]:
        return None
    res = costmap.resolution_m
    cost = (1.0 + costmap.cost).ravel().tolist()   # inf where blocked
    tr = trav.ravel().tolist()
    s = start_cell[1] * w + start_cell[0]
    g = goal_cell[1] * w + goal_cell[0]
    gc, gr = goal_cell
    best = {s: 0.0}
    parent = {s: -1}
    closed = set()
    heap = [(0.0, 0.0, s)]
    while heap:
        _, d, u = heapq.heappop(heap)
        if u in closed:
            continue
        if u == g:
            break
        closed.add(u)
        uc, ur = u % w, u // w
        for dc, dr, step in _NBRS:
            c, r = uc + dc, ur + dr
            if c < 0 or r < 0 or c >= w or r >= h:
                continue
            v = r * w + c
            if not tr[v] or v in closed:
                continue
            if dc and dr and not (tr[ur * w + c] and tr[r * w + uc]):
                continue
            nd = d + step * res * cost[v]
            if nd < best.get(v, math.inf):
                best[v] = nd
                parent[v] = u
                ax, ay = abs(c - gc), abs(r - gr)
                hh = (max(ax, ay) + (SQRT2 - 1.0) * min(ax, ay)) * res
                heapq.heappush(heap, (nd + hh, nd, v))
    if g not in parent:
        return None
    cells = []
    v = g
    while v != -1:
        cells.append((v % w, v // w))
        v = parent[v]
    cells.reverse()
    return cells, best[g]


def reachable_mask(costmap: Costmap, start_cell: tuple[int, int]) -> np.ndarray:
    """Traversable cells 8-connected to ``start_cell`` (same diagonal rule as ``astar``)."""
    return reachable_steps(costmap, start_cell) >= 0


def reachable_steps(costmap: Costmap, start_cell: tuple[int, int]) -> np.ndarray:
    """BFS step count (8-connected) from ``start_cell`` over traversable cells; -1 = unreachable."""
    w, h = costmap.width, costmap.height
    out = np.full(w * h, -1, dtype=np.int64)
    if not costmap.in_bounds(start_cell) or not costmap.traversable[start_cell[1], start_cell[0]]:
        return out.reshape(h, w)
    tr = costmap.traversable.ravel().tolist()
    dist = out.tolist()
    s = start_cell[1] * w + start_cell[0]
    dist[s] = 0
    q = deque([s])
    while q:
        u = q.popleft()
        uc, ur = u % w, u // w
        du = dist[u] + 1
        for dc, dr, _ in _NBRS:
            c, r = uc + dc, ur + dr
            if c < 0 or r < 0 or c >= w or r >= h:
                continue
            v = r * w + c
            if dist[v] >= 0 or not tr[v]:
                continue
            if dc and dr and not (tr[ur * w + c] and tr[r * w + uc]):
                continue
            dist[v] = du
            q.append(v)
    return np.array(dist, dtype=np.int64).reshape(h, w)


# --- geometry helpers ------------------------------------------------------------------------
def path_length(path) -> float:
    p = np.asarray(path, dtype=np.float64)
    if len(p) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(p, axis=0).T)))


def _segment_cells(costmap: Costmap, a, b) -> tuple[np.ndarray, np.ndarray, float]:
    """Rows, cols sampled every resolution/3 along a->b, and the sample spacing."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(length / (costmap.resolution_m / 3.0))))
    t = np.linspace(0.0, 1.0, n + 1)
    x = a[0] + t * (b[0] - a[0])
    y = a[1] + t * (b[1] - a[1])
    cols = np.floor((x - costmap.origin_xy[0]) / costmap.resolution_m).astype(np.int64)
    rows = np.floor((y - costmap.origin_xy[1]) / costmap.resolution_m).astype(np.int64)
    return rows, cols, length / n


def segment_cost(costmap: Costmap, a, b) -> float:
    """Cost of driving straight a->b (same units as ``astar``); inf if any sample is blocked."""
    rows, cols, ds = _segment_cells(costmap, a, b)
    if rows.min() < 0 or cols.min() < 0 or rows.max() >= costmap.height or cols.max() >= costmap.width:
        return math.inf
    c = costmap.cost[rows, cols]
    if not np.all(np.isfinite(c)):
        return math.inf
    return float(np.sum(1.0 + c[1:]) * ds)


def line_of_sight(costmap: Costmap, a, b) -> bool:
    return math.isfinite(segment_cost(costmap, a, b))


def shortcut(costmap: Costmap, pts: Path, tolerance: float = 1.02) -> Path:
    """Greedy shortcutting: from each anchor, extend while the straight segment is collision free
    and costs no more than ``tolerance`` x the polyline it replaces (keeps soft-cost clearance)."""
    if len(pts) <= 2:
        return list(pts)
    seg = [segment_cost(costmap, pts[k], pts[k + 1]) for k in range(len(pts) - 1)]
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = i + 1
        acc = seg[i]
        while j + 1 < len(pts):
            acc_next = acc + seg[j]
            direct = segment_cost(costmap, pts[i], pts[j + 1])
            if not math.isfinite(direct) or direct > tolerance * acc_next + 1e-9:
                break
            acc = acc_next
            j += 1
        out.append(pts[j])
        i = j
    return out


def resample(path: Path, spacing_m: float) -> Path:
    """Points every ``spacing_m`` along the polyline; the exact end point is kept."""
    if len(path) < 2:
        return list(path)
    out = [tuple(map(float, path[0]))]
    carry = 0.0  # distance already travelled since the last output point
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        if L <= 0:
            continue
        s = spacing_m - carry
        while s <= L + 1e-12:
            out.append((x0 + (x1 - x0) * s / L, y0 + (y1 - y0) * s / L))
            s += spacing_m
        carry = L - (s - spacing_m)
    end = (float(path[-1][0]), float(path[-1][1]))
    if math.hypot(out[-1][0] - end[0], out[-1][1] - end[1]) > 1e-6:
        out.append(end)
    else:
        out[-1] = end
    return out


# --- start handling --------------------------------------------------------------------------
def _escape_cell(costmap: Costmap, start_xy, radius_m: float) -> tuple[int, int] | None:
    """Nearest traversable cell within radius whose straight segment from start crosses no
    OCCUPIED/UNKNOWN cell. None if the start itself is in an OCCUPIED/UNKNOWN cell."""
    sc, sr = costmap.world_to_cell(start_xy)
    if not costmap.in_bounds((sc, sr)) or not costmap.free[sr, sc]:
        return None
    k = int(math.ceil(radius_m / costmap.resolution_m))
    r0, r1 = max(0, sr - k), min(costmap.height, sr + k + 1)
    c0, c1 = max(0, sc - k), min(costmap.width, sc + k + 1)
    rows, cols = np.nonzero(costmap.traversable[r0:r1, c0:c1])
    if len(rows) == 0:
        return None
    rows += r0
    cols += c0
    xy = np.asarray(costmap.origin_xy) + (np.column_stack((cols, rows)) + 0.5) * costmap.resolution_m
    d = np.hypot(xy[:, 0] - start_xy[0], xy[:, 1] - start_xy[1])
    free = costmap.free
    for i in np.argsort(d):
        if d[i] > radius_m:
            break
        rr, cc, _ = _segment_cells(costmap, start_xy, xy[i])
        if np.all(free[rr, cc]):
            return int(cols[i]), int(rows[i])
    return None


def start_cell(costmap: Costmap, start_xy, allow_escape: bool = True,
               escape_radius_m: float = 0.5) -> tuple[int, int] | None:
    """The cell planning starts from: the start's own cell if traversable, else (with
    ``allow_escape``) the escape cell, else None."""
    st = _start_cell(costmap, start_xy, allow_escape, escape_radius_m)
    return None if st is None else st[0]


def _start_cell(costmap: Costmap, start_xy, allow_escape: bool, escape_radius_m: float):
    """(cell, prefix points) for a start position, or None."""
    sc = costmap.world_to_cell(start_xy)
    if costmap.in_bounds(sc) and costmap.traversable[sc[1], sc[0]]:
        return sc, [tuple(map(float, start_xy))]
    if not allow_escape:
        return None
    esc = _escape_cell(costmap, start_xy, escape_radius_m)
    if esc is None:
        return None
    return esc, [tuple(map(float, start_xy))]


def _arc(start_xy, theta: float, radius: float, side: int, angle: float, step: float) -> Path:
    """Forward arc from a pose; side +1 = left, -1 = right."""
    n = max(2, int(math.ceil(radius * angle / step)))
    cx = start_xy[0] - side * radius * math.sin(theta)
    cy = start_xy[1] + side * radius * math.cos(theta)
    out = []
    for i in range(n + 1):
        a = side * angle * i / n
        h = theta + a
        out.append((cx + side * radius * math.sin(h), cy - side * radius * math.cos(h)))
    return out


def start_feasible(path: Path, theta: float, min_turn_radius_m: float, horizon_factor: float = 3.0,
                   slack: float = 0.8) -> bool:
    """True if a forward-only car at ``path[0]`` facing ``theta`` can follow the path's first
    ``horizon_factor * min_turn_radius_m`` metres: no point there lies behind the car, and no
    point beyond one turning radius lies inside either full-lock turning circle (shrunk by
    ``slack``) - those need reversing or a loop. Closer points are skipped: pure pursuit
    converges onto them with some lag."""
    x0, y0 = path[0]
    c, s = math.cos(theta), math.sin(theta)
    R = min_turn_radius_m
    acc = 0.0
    for a, b in zip(path[:-1], path[1:]):
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        if acc > horizon_factor * R:
            break
        dx, dy = b[0] - x0, b[1] - y0
        x, y = c * dx + s * dy, -s * dx + c * dy
        if x * x + y * y < 0.05 ** 2:
            continue
        if x < 0:
            return False
        if acc >= R and x * x + (abs(y) - R) ** 2 < (slack * R) ** 2:
            return False
    return True


def _lethal_xy(costmap: Costmap, xy) -> bool:
    c, r = costmap.world_to_cell(xy)
    return not costmap.in_bounds((c, r)) or bool(costmap.lethal[r, c])


def _finish(costmap: Costmap, prefix: Path, cells, end_xy, spacing_m: float, smooth: bool) -> Path:
    pts = [costmap.cell_to_world(c) for c in cells]
    pts[-1] = tuple(map(float, end_xy))
    if len(pts) > 1 and costmap.is_traversable_xy(prefix[0]) and costmap.world_to_cell(prefix[0]) == tuple(cells[0]):
        pts[0] = tuple(map(float, prefix[0]))
    if smooth:
        pts = shortcut(costmap, pts)
    dedup = list(prefix[:1])
    for p in list(prefix[1:]) + pts:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-9:
            dedup.append(p)
    return resample(dedup, spacing_m)


# --- heading-aware start: forward-only lattice search ----------------------------------------
class _CostToGo:
    """Lazy (resumable) Dijkstra from the goal over traversable cells: exact cost-to-go in the
    same units as ``astar``, expanded only as far as queries need."""

    def __init__(self, costmap: Costmap, goal_cell: tuple[int, int]) -> None:
        self.w, self.h = costmap.width, costmap.height
        self.res = costmap.resolution_m
        self.cost = (1.0 + costmap.cost).ravel().tolist()
        self.tr = costmap.traversable.ravel().tolist()
        g = goal_cell[1] * self.w + goal_cell[0]
        self.goal = g
        self.dist: dict[int, float] = {}
        self.best = {g: 0.0}
        self.parent = {g: -1}
        self.heap = [(0.0, g)]

    def value(self, idx: int) -> float:
        if not self.tr[idx]:
            return math.inf
        w, h, res, cost, tr = self.w, self.h, self.res, self.cost, self.tr
        dist, best, parent, heap = self.dist, self.best, self.parent, self.heap
        while idx not in dist:
            if not heap:
                return math.inf
            d, u = heapq.heappop(heap)
            if u in dist:
                continue
            dist[u] = d
            uc, ur = u % w, u // w
            for dc, dr, step in _NBRS:
                c, r = uc + dc, ur + dr
                if c < 0 or r < 0 or c >= w or r >= h:
                    continue
                v = r * w + c
                if not tr[v] or v in dist:
                    continue
                if dc and dr and not (tr[ur * w + c] and tr[r * w + uc]):
                    continue
                nd = d + step * res * cost[u]   # moving v -> u costs entering u
                if nd < best.get(v, math.inf):
                    best[v] = nd
                    parent[v] = u
                    heapq.heappush(heap, (nd, v))
        return dist[idx]

    def descend(self, idx: int, max_cells: int | None = None) -> list[tuple[int, int]]:
        """Cells (col, row) from ``idx`` to the goal along the optimal tree (call ``value`` first)."""
        out = []
        v = idx
        while v != -1 and (max_cells is None or len(out) < max_cells):
            out.append((v % self.w, v // self.w))
            v = self.parent.get(v, -1)
        return out


def _primitive(x: float, y: float, th: float, k: float, length: float, step: float) -> list[tuple[float, float, float]]:
    """Poses every ``step`` along a constant-curvature forward motion (exact arcs)."""
    n = max(1, int(math.ceil(length / step)))
    out = []
    for i in range(1, n + 1):
        s = length * i / n
        if abs(k) < 1e-9:
            out.append((x + s * math.cos(th), y + s * math.sin(th), th))
        else:
            t = th + k * s
            out.append((x + (math.sin(t) - math.sin(th)) / k, y - (math.cos(t) - math.cos(th)) / k, t))
    return out


_LATTICE_RADIUS_FACTOR = 1.05   # lattice turns at this x min turning radius (leaves steering margin)


def _lattice(costmap: Costmap, start_xy, theta: float, h_fn, goal_fn, max_expansions: int,
             escape_m: float, strict: bool = True):
    """Best-first search over forward constant-curvature motions (radius >= 1.05 x min turning
    radius, 0.3 m long). ``h_fn(x, y, th)`` -> heuristic (inf prunes), ``goal_fn(x, y, th)`` ->
    result or None. With ``strict``, primitive samples must be traversable, except in the first
    ``escape_m`` of travel (the car may start inside the inflation band); otherwise (and in the
    escape band) non-lethal - the body clear of obstacles, no margin - is enough.
    Returns (poses from the start, goal_fn result) or None."""
    rmin = costmap.vehicle.min_turn_radius_m
    kmax = 1.0 / (_LATTICE_RADIUS_FACTOR * rmin)
    curvatures = (kmax, 0.5 * kmax, 0.0, -0.5 * kmax, -kmax)
    length = 0.3
    res = costmap.resolution_m
    trav, lethal, cost = costmap.traversable, costmap.lethal, costmap.cost
    n_bins = 16
    bin_w = 2 * math.pi / n_bins
    pos_bin = 2.0 * res
    soft = costmap.config.soft_weight

    def key(x, y, th):
        return (int(math.floor(x / pos_bin)), int(math.floor(y / pos_bin)),
                int(math.floor(((th % (2 * math.pi)) + bin_w / 2) / bin_w)) % n_bins)

    # node: (x, y, th, g, travelled, parent, poses-from-parent)
    nodes = [(start_xy[0], start_xy[1], theta, 0.0, 0.0, -1, [])]
    h0 = h_fn(start_xy[0], start_xy[1], theta)
    if not math.isfinite(h0):
        return None
    heap = [(h0, 0)]
    closed = set()
    expansions = 0
    while heap and expansions < max_expansions:
        _, ni = heapq.heappop(heap)
        x, y, th, g, travelled, _, _ = nodes[ni]
        kk = key(x, y, th)
        if kk in closed:
            continue
        closed.add(kk)
        expansions += 1
        if ni != 0:
            found = goal_fn(x, y, th)
            if found is not None:
                prefix = []
                j = ni
                while j > 0:
                    prefix = nodes[j][6] + prefix
                    j = nodes[j][5]
                return [(start_xy[0], start_xy[1], theta)] + prefix, found
        for k in curvatures:
            poses = _primitive(x, y, th, k, length, res)
            ok = True
            step_cost = 0.0
            t = travelled
            for (px, py, _) in poses:
                t += length / len(poses)
                c, r = costmap.world_to_cell((px, py))
                if not costmap.in_bounds((c, r)) or lethal[r, c]:
                    ok = False
                    break
                if trav[r, c]:
                    step_cost += 1.0 + cost[r, c]
                elif t <= escape_m or not strict:
                    step_cost += 1.0 + soft
                else:
                    ok = False
                    break
            if not ok:
                continue
            ex, ey, eth = poses[-1]
            if key(ex, ey, eth) in closed:
                continue
            hh = h_fn(ex, ey, eth)
            if not math.isfinite(hh):
                continue
            ng = g + step_cost * (length / len(poses)) + (0.05 if k else 0.0)
            nodes.append((ex, ey, eth, ng, t, ni, poses))
            heapq.heappush(heap, (ng + hh, len(nodes) - 1))
    return None


def _lattice_start(costmap: Costmap, start_xy, theta: float, goal_cell: tuple[int, int],
                   max_expansions: int, escape_m: float):
    """Forward manoeuvre after which the grid route to ``goal_cell`` is ``start_feasible``.
    Returns (poses, cost-to-go object, cell index where the grid route takes over) or None."""
    rmin = costmap.vehicle.min_turn_radius_m
    res = costmap.resolution_m
    w = costmap.width
    trav = costmap.traversable
    ctg = _CostToGo(costmap, goal_cell)
    gx, gy = costmap.cell_to_world(goal_cell)
    probe_cells = int(math.ceil(3.0 * rmin / res)) + 4

    def h_fn(x, y, th):
        c, r = costmap.world_to_cell((x, y))
        if costmap.in_bounds((c, r)) and trav[r, c]:
            return ctg.value(r * w + c)
        return math.hypot(x - gx, y - gy)  # still inside the escape band

    def goal_fn(x, y, th):
        c, r = costmap.world_to_cell((x, y))
        if not (costmap.in_bounds((c, r)) and trav[r, c]):
            return None
        idx = r * w + c
        if not math.isfinite(ctg.value(idx)):
            return None
        cells = ctg.descend(idx, probe_cells)
        pts = [(x, y)] + [costmap.cell_to_world(cr) for cr in cells[1:]]
        if math.hypot(x - gx, y - gy) <= 2 * res or start_feasible(pts, th, rmin):
            return idx
        return None

    found = _lattice(costmap, start_xy, theta, h_fn, goal_fn, max_expansions, escape_m)
    if found is None:  # tight spot: allow the body-clear (no margin) manoeuvres too
        found = _lattice(costmap, start_xy, theta, h_fn, goal_fn, max_expansions, escape_m, strict=False)
    if found is None:
        return None
    poses, idx = found
    return poses, ctg, idx


def can_turn_around(costmap: Costmap, xy, theta: float, turn_deg: float = 150.0,
                    max_expansions: int = 800, strict: bool = True) -> bool:
    """True if a forward-only car at ``xy`` facing ``theta`` can reverse its heading (turn by at
    least ``turn_deg``) with forward motions on traversable cells (``strict``, full inflation
    margin: room for tracking error and for obstacles still unmapped) or, with ``strict=False``,
    cells that merely keep the body clear (non-lethal). False in dead ends such as a narrow
    slot: the car could drive in but never out without reversing."""
    kmax = 1.0 / (_LATTICE_RADIUS_FACTOR * costmap.vehicle.min_turn_radius_m)
    need = math.radians(turn_deg)

    def turned(th):
        return abs((th - theta + math.pi) % (2 * math.pi) - math.pi)

    def h_fn(x, y, th):
        return max(0.0, need - turned(th)) / kmax

    def goal_fn(x, y, th):
        return True if turned(th) >= need else None

    # The cumulative turn is tracked through the wrapped heading: a 150-degree turn is reached
    # before wrap-around ambiguity (180 degrees) matters.
    return _lattice(costmap, xy, theta, h_fn, goal_fn, max_expansions, escape_m=0.0, strict=strict) is not None


# --- results and exits -----------------------------------------------------------------------
class PlannedPath(list):
    """A list of (x, y) world points plus planning metadata.

    ``trimmed_m`` > 0 when the end was pulled back so the car does not stop where it cannot turn
    around (``can_turn_around``)."""

    def __init__(self, pts, *, goal_xy=None, trimmed_m: float = 0.0, detail: str = ""):
        super().__init__(pts)
        self.goal_xy = None if goal_xy is None else (float(goal_xy[0]), float(goal_xy[1]))
        self.trimmed_m = float(trimmed_m)
        self.detail = detail


def _occ_lethal_xy(costmap: Costmap, xy, slack_m: float) -> bool:
    c, r = costmap.world_to_cell(xy)
    if not costmap.in_bounds((c, r)):
        return True
    return bool(costmap.occ_clearance_m[r, c] < costmap.vehicle.half_width_m + slack_m)


def has_forward_exit(costmap: Costmap, xy, theta: float, straight_m: float = 1.0,
                     arc_deg: float = 150.0) -> bool:
    """True if a car at ``xy`` facing ``theta`` can drive away forwards: a straight run of
    ``straight_m`` or a ``arc_deg`` turn at 1.15 x min turning radius (either side) keeps the
    body clear of OCCUPIED cells (UNKNOWN is ignored: the LiDAR will have seen it by then)."""
    slack = costmap.resolution_m / 2.0
    step = costmap.resolution_m
    n = max(2, int(math.ceil(straight_m / step)))
    c, s = math.cos(theta), math.sin(theta)
    if not any(_occ_lethal_xy(costmap, (xy[0] + c * straight_m * i / n, xy[1] + s * straight_m * i / n), slack)
               for i in range(1, n + 1)):
        return True
    radius = 1.15 * costmap.vehicle.min_turn_radius_m
    for side in (1, -1):
        pts = _arc(xy, theta, radius, side, math.radians(arc_deg), step)
        if not any(_occ_lethal_xy(costmap, p, slack) for p in pts[1:]):
            return True
    return False


_EXIT_HEADING_SLACK_RAD = 0.3


def _trim_for_exit(costmap: Costmap, path: Path, max_trim_m: float, step_m: float = 0.15
                   ) -> tuple[Path, float] | None:
    """Pull the end back (at most ``max_trim_m``, in ``step_m`` steps) to the last point where
    the car, arriving along the path, can still turn around. The follower stops a little early
    and a little off line, so the check must also pass with the heading off by
    +-``_EXIT_HEADING_SLACK_RAD``. None if there is no such point."""
    if len(path) < 2:
        return path, 0.0
    back = 0.0
    since = math.inf
    for i in range(len(path) - 1, 0, -1):
        a, b = path[i - 1], path[i]
        if since >= step_m:
            since = 0.0
            h = math.atan2(b[1] - a[1], b[0] - a[0])
            if all(can_turn_around(costmap, b, h + dh)
                   for dh in (0.0, -_EXIT_HEADING_SLACK_RAD, _EXIT_HEADING_SLACK_RAD)):
                return path[:i + 1], back
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        back += seg
        since += seg
        if back > max_trim_m:
            break
    return None


# --- public API ------------------------------------------------------------------------------
def path_clear(costmap: Costmap, pts, near_m: float = 0.3) -> bool:
    """Re-check a path (e.g. ``PurePursuit.remaining_path()``) against a newer costmap: points
    within ``near_m`` (along the path) of its start must be non-lethal, later points traversable.
    False means stop and replan: the map changed under the path."""
    acc = 0.0
    prev = None
    for p in pts:
        if prev is not None:
            acc += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
        c, r = costmap.world_to_cell(p)
        if not costmap.in_bounds((c, r)) or costmap.lethal[r, c]:
            return False
        if acc > near_m and not costmap.traversable[r, c]:
            return False
    return True


def check_tracking(costmap: Costmap, path, start_pose: Pose2D, pursuit_config=None,
                   dt: float = 0.1) -> tuple[bool, tuple[float, float] | None, str]:
    """Drive ``path`` with pure pursuit on the ideal bicycle model and check every pose stays
    out of ``lethal`` cells. Returns (ok, failure point on the path or None, detail).

    Synthetic and optimistic (no lag, no slip): it catches corners the car cannot turn, not
    real-world tracking error."""
    from .pursuit import PurePursuit, PursuitStatus   # local import: pursuit is independent of planner
    from .sim import BicycleSim
    veh = costmap.vehicle
    pp = PurePursuit(veh, pursuit_config)
    pp.set_path(path)
    sim = BicycleSim(veh, start_pose, dt=dt)
    v_min = veh.max_speed_mps * pp.config.min_throttle_frac
    max_steps = int(3.0 * path_length(path) / max(v_min * dt, 1e-6)) + 50
    pts = np.asarray(path, dtype=np.float64)

    def on_path(p):
        i = int(np.argmin(np.hypot(pts[:, 0] - p.x, pts[:, 1] - p.y)))
        return float(pts[i, 0]), float(pts[i, 1])

    for _ in range(max_steps):
        cmd = pp.update(sim.pose, speed_mps=sim.speed_mps)
        if cmd.status is PursuitStatus.REACHED:
            return True, None, "reached"
        if cmd.status is PursuitStatus.NEEDS_REPLAN:
            return False, on_path(sim.pose), cmd.detail
        sim.step(cmd.throttle, cmd.steering)
        c, r = costmap.world_to_cell((sim.pose.x, sim.pose.y))
        if not costmap.in_bounds((c, r)) or costmap.lethal[r, c]:
            return False, on_path(sim.pose), "body would touch an obstacle"
    return False, on_path(sim.pose), "did not reach the end"


def _block_disk(costmap: Costmap, xy, radius_m: float, keep: list) -> Costmap:
    """Copy of ``costmap`` with traversable cells within ``radius_m`` of ``xy`` blocked, except
    cells within ``radius_m`` of any point in ``keep`` (start, goal)."""
    res = costmap.resolution_m
    rows, cols = np.mgrid[0:costmap.height, 0:costmap.width]
    x = costmap.origin_xy[0] + (cols + 0.5) * res
    y = costmap.origin_xy[1] + (rows + 0.5) * res
    block = np.hypot(x - xy[0], y - xy[1]) <= radius_m
    for k in keep:
        block &= np.hypot(x - k[0], y - k[1]) > radius_m
    trav = costmap.traversable & ~block
    return replace(costmap, traversable=trav, cost=np.where(trav, costmap.cost, np.inf))


def plan(costmap: Costmap, start_xy, goal_xy, *, start_theta: float | None = None,
         allow_escape: bool = True, escape_radius_m: float = 0.5, smooth: bool = True,
         spacing_m: float = 0.1, ensure_exit: bool = True, max_trim_m: float = 1.0,
         validate: bool = True, max_repairs: int = 3) -> PlannedPath | None:
    """Plan from ``start_xy`` to ``goal_xy`` (world frame). ``goal_xy`` must be traversable
    (use ``plan_approach`` for goals on/against obstacles). Returns None if unreachable.

    With ``start_theta`` the plan respects the car's kinematics (see the module docstring); it
    returns None if no forward manoeuvre out of the start pose is collision free. With
    ``validate`` (needs ``start_theta``) the path is test-driven with ``check_tracking``; where
    the simulated car fails (a corner too tight to turn), a disc of the map is blocked and the
    route replanned, up to ``max_repairs`` times; None if every attempt fails.

    With ``ensure_exit`` the end is pulled back up to ``max_trim_m`` if the car could not turn
    around there without reversing; None if no such end point exists."""
    gc = costmap.world_to_cell(goal_xy)
    if not costmap.in_bounds(gc) or not costmap.traversable[gc[1], gc[0]]:
        return None
    work = costmap
    for _attempt in range(max_repairs + 1):
        path = _plan_to_cell(work, start_xy, gc, goal_xy, start_theta, allow_escape, escape_radius_m,
                             smooth, spacing_m, ensure_exit, max_trim_m)
        if path is None or start_theta is None or not validate:
            return path
        ok, fail_xy, detail = check_tracking(costmap, path, Pose2D(start_xy[0], start_xy[1], start_theta))
        if ok:
            return path
        if fail_xy is None or math.hypot(fail_xy[0] - start_xy[0], fail_xy[1] - start_xy[1]) < 0.3:
            return None   # fails right at the start: nothing to route around
        work = _block_disk(work, fail_xy, 0.35, [start_xy, goal_xy])
    return None


def _plan_to_cell(costmap, start_xy, gc, end_xy, start_theta, allow_escape, escape_radius_m,
                  smooth, spacing_m, ensure_exit, max_trim_m, max_expansions: int = 6000) -> PlannedPath | None:
    st = _start_cell(costmap, start_xy, allow_escape, escape_radius_m)
    if st is None:
        return None
    sc, prefix = st
    res = astar(costmap, sc, gc)
    if res is None:
        return None
    path = _finish(costmap, prefix, res[0], end_xy, spacing_m, smooth)

    def done(pts, detail=""):
        trimmed = 0.0
        if ensure_exit:
            t = _trim_for_exit(costmap, pts, max_trim_m)
            if t is None:
                return None   # every possible end point is a dead end for a forward-only car
            pts, trimmed = t
        return PlannedPath(pts, goal_xy=end_xy, trimmed_m=trimmed, detail=detail)

    if start_theta is None or len(path) < 2 or path_length(path) < 1e-6:
        return done(path)
    if start_feasible(path, start_theta, costmap.vehicle.min_turn_radius_m):
        return done(path)
    found = _lattice_start(costmap, start_xy, start_theta, gc, max_expansions,
                           escape_m=escape_radius_m if allow_escape else 0.0)
    if found is None:
        return None
    poses, ctg, idx = found
    head = [(p[0], p[1]) for p in poses]
    tail = [costmap.cell_to_world(c) for c in ctg.descend(idx)]
    tail[0] = head[-1]
    tail[-1] = (float(end_xy[0]), float(end_xy[1]))
    if smooth:
        tail = shortcut(costmap, tail)
    pts = head + [p for p in tail[1:]]
    return done(resample(pts, spacing_m), "forward manoeuvre before the grid route")


def plan_approach(costmap: Costmap, start_xy, goal_xy, standoff_m: float, *,
                  start_theta: float | None = None, allow_escape: bool = True,
                  escape_radius_m: float = 0.5, smooth: bool = True, spacing_m: float = 0.1,
                  ensure_exit: bool = True, max_trim_m: float = 0.5,
                  max_candidates: int = 10) -> PlannedPath | None:
    """Plan to the reachable traversable cell closest to ``goal_xy`` within ``standoff_m`` of it
    (trying up to ``max_candidates`` cells, nearest first, at least 0.3 m apart, until the planned
    end - after the exit check - is still within ``standoff_m``). None if none works.
    ``PlannedPath.goal_xy`` is the chosen cell."""
    if standoff_m < 0:
        raise ValueError("standoff_m must be >= 0")
    kw = dict(start_theta=start_theta, allow_escape=allow_escape, escape_radius_m=escape_radius_m,
              smooth=smooth, spacing_m=spacing_m, ensure_exit=ensure_exit, max_trim_m=max_trim_m)
    gc = costmap.world_to_cell(goal_xy)
    if costmap.in_bounds(gc) and costmap.traversable[gc[1], gc[0]]:
        return plan(costmap, start_xy, goal_xy, **kw)
    st = _start_cell(costmap, start_xy, allow_escape, escape_radius_m)
    if st is None:
        return None
    reach = reachable_mask(costmap, st[0])
    k = int(math.ceil(standoff_m / costmap.resolution_m)) + 1
    r0, r1 = max(0, gc[1] - k), min(costmap.height, gc[1] + k + 1)
    c0, c1 = max(0, gc[0] - k), min(costmap.width, gc[0] + k + 1)
    if r0 >= r1 or c0 >= c1:
        return None
    rows, cols = np.nonzero(reach[r0:r1, c0:c1])
    if len(rows) == 0:
        return None
    rows += r0
    cols += c0
    xy = np.asarray(costmap.origin_xy) + (np.column_stack((cols, rows)) + 0.5) * costmap.resolution_m
    d = np.hypot(xy[:, 0] - goal_xy[0], xy[:, 1] - goal_xy[1])
    order = [int(i) for i in np.argsort(d) if d[i] <= standoff_m]
    tried: list[tuple[float, float]] = []
    for i in order:
        target = (float(xy[i, 0]), float(xy[i, 1]))
        if any(math.hypot(target[0] - t[0], target[1] - t[1]) < 0.3 for t in tried):
            continue
        tried.append(target)
        if len(tried) > max_candidates:
            break
        path = plan(costmap, start_xy, target, **kw)
        # The exit check may pull the end back; it must still be within the standoff.
        if path is not None and math.hypot(path[-1][0] - goal_xy[0], path[-1][1] - goal_xy[1]) <= standoff_m:
            return path
    return None
