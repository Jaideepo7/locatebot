"""Static vs dynamic map layers and per-scan change detection (M2).

``LayeredMap`` holds, on one cell raster:

* ``static``: the room (the M1 slowly updated ``OccupancyGrid``, log-odds).
* ``dynamic``: short-lived positive evidence for things that are new or moving; it decays toward
  unknown (0) with a half-life (default 3 s) and is cleared by LiDAR rays passing through.
* ``extra``: named float layers registered by other modules (M3: ``glass_or_low``,
  ``wifi_rssi``), stored as ``mapfile.MapLayer`` with their update parameters in ``params``.
  A layer whose params hold ``half_life_s`` decays toward 0 on every update; one holding
  ``blocks_above`` blocks planning where its value is >= that.

Change detection (``classify_points``), per valid beam endpoint, with the localized pose:

* ``ROOM``: a static OCCUPIED cell within one cell of the endpoint (pose noise ~ 1 cell).
* ``NEW``: the endpoint cell *and its 8 neighbours* are confidently FREE in the static map
  (log-odds < ``free_lo``), and >= ``ray_free_frac`` of the ray samples in the last
  ``ray_check_m`` before it (skipping ``ray_skip_m``) are confidently free too: the robot has
  seen that space empty before, and is looking at it through known free space.
* ``UNKNOWN``: anything else (unknown cells, the edge of the map, near walls).

Update rules (``update``): NEW endpoints go only into ``dynamic``; ``static`` gets M1's slow
update from every ray, but NEW endpoints add no occupancy and their cells are not freed (a
person in a doorway neither adds a wall nor erases the free space behind them: their rays stop
at the person). NEW points are clustered (``detect.cluster_scan``, robot frame, then map frame)
into ``ChangeBlob`` tracks (nearest-neighbour association). A blob that stays within
``still_m`` for ``promote_after_s`` (default 120 s) is *promoted*: its points are integrated into
``static`` as ordinary slow hits from then on (a moved chair becomes part of the room). A blob is
dropped after its spot was seen free in ``blob_free_views`` scans, or unseen for
``blob_max_unseen_s``. ``planning_grid()`` = static with dynamic and blocking extra layers as
obstacles (cell-wise max), so ``nav.build_costmap`` plans around them unchanged.

Tests use synthetic rooms (``locatebot.lidar.sim``): software behaviour only. Known gaps: glass and
mirrors (no or wrong returns), thin chair legs (1-2 beams; ``cluster_min_points`` 2), things within a
cell of a wall (classified ROOM), and space the static map never saw (UNKNOWN, not NEW).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .detect import cluster_scan
from .grid import OccupancyGrid
from .mapfile import MapLayer
from .types import RANGE_MAX_M, Pose2D, Scan

INVALID, ROOM, NEW, UNKNOWN = -1, 0, 1, 2

DYNAMIC_LAYER = "dynamic"


def _logit(p: float) -> float:
    return math.log(p / (1.0 - p))


@dataclass(frozen=True)
class LayersConfig:
    occ_threshold: float = 0.65        # static p above this = OCCUPIED (room)
    free_lo: float = -1.5              # static log-odds below this = confidently FREE (p < 0.18)
    ray_check_m: float = 1.0           # how much of the ray before a NEW endpoint must be free
    ray_skip_m: float = 0.1            # ... ignoring the last bit (the object's own cells)
    ray_step_m: float = 0.1
    ray_free_frac: float = 0.8
    slow_rate: float = 0.2             # M1 slow update for confident static cells
    confident_abs: float = 1.5
    dynamic_half_life_s: float = 3.0
    dynamic_hit: float = 1.5           # log-odds added to a dynamic cell per NEW endpoint
    dynamic_clear: float = 1.0         # removed from a dynamic cell per ray passing through it
    dynamic_max: float = 4.0
    dynamic_occ: float = 0.62          # dynamic log-odds at/above this blocks planning (p ~ 0.65)
    cluster_break_m: float = 0.15
    cluster_min_points: int = 2        # thin chair legs return 1-3 beams
    blob_match_m: float = 0.5          # association gate between scans
    blob_free_views: int = 3           # drop a blob after its spot was seen free this often
    blob_max_unseen_s: float = 300.0   # ... or after this long without a detection
    max_blobs: int = 50
    moving_window_s: float = 1.0       # displacement over this window decides "moving"
    moving_m: float = 0.1
    still_m: float = 0.15              # a blob that stays within this is "still"
    promote_after_s: float = 120.0     # still this long -> part of the room
    max_range_m: float = RANGE_MAX_M

    def __post_init__(self) -> None:
        if self.dynamic_half_life_s <= 0 or self.promote_after_s < 0:
            raise ValueError("dynamic_half_life_s must be > 0 and promote_after_s >= 0")


@dataclass(frozen=True)
class ChangeBlob:
    """A cluster of NEW points tracked across scans (map frame)."""

    id: int
    centroid_xy: tuple[float, float]
    width_m: float
    n_points: int
    first_seen_s: float
    last_seen_s: float
    persistence: int                   # number of scans it was detected in
    moving: bool = False               # moved > moving_m within the last moving_window_s
    promoted: bool = False             # still long enough: being integrated into the room


@dataclass
class _Blob:
    id: int
    xy: np.ndarray
    width: float
    n: int
    first_s: float
    last_s: float
    seen: int = 1
    anchor: np.ndarray | None = None
    still_since_s: float = 0.0
    promoted: bool = False
    free_views: int = 0
    history: deque = field(default_factory=lambda: deque(maxlen=64))

    def snapshot(self, moving: bool) -> ChangeBlob:
        return ChangeBlob(self.id, (float(self.xy[0]), float(self.xy[1])), float(self.width), int(self.n),
                          self.first_s, self.last_s, self.seen, moving, self.promoted)


@dataclass(frozen=True, eq=False)
class LayerUpdate:
    labels: np.ndarray                 # per beam: INVALID / ROOM / NEW / UNKNOWN (after promotion)
    blobs: tuple[ChangeBlob, ...]      # current non-promoted blobs


class LayeredMap:
    def __init__(self, static: OccupancyGrid, config: LayersConfig | None = None,
                 extra: dict[str, MapLayer] | None = None) -> None:
        self.static = static
        self.config = config or LayersConfig()
        self.dynamic = np.zeros_like(static.log_odds, dtype=np.float32)
        self.extra: dict[str, MapLayer] = {}
        for name, lay in (extra or {}).items():
            if lay.data.shape != static.log_odds.shape:
                raise ValueError(f"layer {name!r} shape {lay.data.shape} != grid {static.log_odds.shape}")
            self.extra[name] = lay
        self._occ_lo = _logit(self.config.occ_threshold)
        self._blobs: list[_Blob] = []
        self._next_id = 1
        self._t: float | None = None
        # this scan's NEW-point clusters (map-frame centroid, width_m, n_points), promoted blobs
        # left out: the people tracker's input (M3)
        self.last_clusters: list[tuple[np.ndarray, float, int]] = []

    # ------------------------------------------------------------------ extra layers
    def register_layer(self, name: str, fill: float = 0.0, dtype=np.float32, scale: float = 1.0,
                       offset: float = 0.0, params: dict | None = None) -> MapLayer:
        """Add (or return the existing) named layer. ``params``: JSON-able update rule/metadata;
        ``half_life_s`` -> decays toward 0 on every update; ``blocks_above`` -> planning obstacle."""
        if name in ("static", DYNAMIC_LAYER):
            raise ValueError(f"layer name {name!r} is reserved")
        if name in self.extra:
            if params:
                self.extra[name].params.update(params)
            return self.extra[name]
        lay = MapLayer(np.full(self.static.log_odds.shape, fill, dtype=dtype), scale, offset, dict(params or {}))
        self.extra[name] = lay
        return lay

    def layer(self, name: str) -> MapLayer:
        return self.extra[name]

    def save_layers(self) -> dict[str, MapLayer]:
        """Layers worth keeping across sessions (``save_map(..., layers=...)``): the extra ones.
        ``dynamic`` is short-lived by design and not saved."""
        return dict(self.extra)

    # ------------------------------------------------------------------ change detection
    def _lookup(self, arr: np.ndarray, cr: np.ndarray, fill) -> np.ndarray:
        g = self.static
        ok = (cr[..., 0] >= 0) & (cr[..., 0] < g.width) & (cr[..., 1] >= 0) & (cr[..., 1] < g.height)
        c = np.clip(cr[..., 0], 0, g.width - 1)
        r = np.clip(cr[..., 1], 0, g.height - 1)
        return np.where(ok, arr[r, c], fill)

    def classify_points(self, scan: Scan, pose: Pose2D) -> np.ndarray:
        """Per-beam labels ``INVALID / ROOM / NEW / UNKNOWN`` against the static layer."""
        cfg = self.config
        labels = np.full(len(scan), INVALID, dtype=np.int8)
        valid = scan.valid_mask(range_max_m=cfg.max_range_m)
        if not valid.any():
            return labels
        r = scan.ranges_m[valid]
        a = scan.angles_rad[valid] + pose.theta
        dx, dy = np.cos(a), np.sin(a)
        ends = np.column_stack((pose.x + r * dx, pose.y + r * dy))
        g = self.static
        cr = g.world_to_cell(ends)
        lo = g.log_odds
        nb = np.array([(i, j) for j in (-1, 0, 1) for i in (-1, 0, 1)], dtype=np.int64)
        ncr = cr[:, None, :] + nb[None, :, :]                               # (N, 9, 2)
        nlo = self._lookup(lo, ncr, np.float32(0.0))                        # off-grid = unknown
        room = (nlo > self._occ_lo).any(axis=1)
        end_free = (nlo < cfg.free_lo).all(axis=1)
        # ray samples before the endpoint
        k = max(1, int(math.ceil((cfg.ray_check_m - cfg.ray_skip_m) / cfg.ray_step_m)))
        back = cfg.ray_skip_m + cfg.ray_step_m * np.arange(k)
        t = r[:, None] - back[None, :]                                      # (N, K)
        use = t > 0.05
        sx = pose.x + t * dx[:, None]
        sy = pose.y + t * dy[:, None]
        scr = g.world_to_cell(np.column_stack((sx.ravel(), sy.ravel()))).reshape(len(r), k, 2)
        sfree = (self._lookup(lo, scr, np.float32(0.0)) < cfg.free_lo) & use
        n_use = use.sum(axis=1)
        ray_ok = (n_use > 0) & (sfree.sum(axis=1) >= cfg.ray_free_frac * np.maximum(n_use, 1))
        lab = np.where(room, ROOM, np.where(end_free & ray_ok, NEW, UNKNOWN)).astype(np.int8)
        labels[valid] = lab
        return labels

    # ------------------------------------------------------------------ updates
    def decay(self, stamp_s: float) -> None:
        """Advance time: dynamic (and extra layers with ``half_life_s``) decay toward 0."""
        if self._t is None:
            self._t = stamp_s
            return
        dt = stamp_s - self._t
        if dt <= 0:
            return
        self._t = stamp_s
        f = np.float32(0.5 ** (dt / self.config.dynamic_half_life_s))
        self.dynamic *= f
        for lay in self.extra.values():
            hl = lay.params.get("half_life_s")
            if hl and lay.data.dtype.kind == "f":
                lay.data *= np.float32(0.5 ** (dt / float(hl)))

    def update(self, scan: Scan, pose: Pose2D, stamp_s: float | None = None,
               integrate_static: bool = True) -> LayerUpdate:
        """One localized scan: classify, track blobs, update dynamic and (slowly) static."""
        cfg = self.config
        now = scan.stamp_s if stamp_s is None else float(stamp_s)
        self.decay(now)
        labels = self.classify_points(scan, pose)
        new = labels == NEW
        g = self.static
        self.last_clusters = []
        if new.any():
            self._track(scan, pose, new, labels, now)
        else:
            self._track_none(now)
        new = labels == NEW  # promoted blobs' beams were relabelled ROOM
        masks = g.ray_masks(scan, pose, cfg.max_range_m)
        if masks is not None:
            free_all, occ_all = masks
            free = free_all & ~occ_all
            size = g.width * g.height
            new_cells = np.zeros(size, dtype=bool)
            static_occ = np.zeros(size, dtype=bool)
            valid = labels != INVALID
            r = scan.ranges_m
            a = scan.angles_rad + pose.theta
            ends = np.column_stack((pose.x + r * np.cos(a), pose.y + r * np.sin(a)))
            new_cells[g._flat(g.world_to_cell(ends[new]))] = True
            static_occ[g._flat(g.world_to_cell(ends[valid & ~new]))] = True
            if integrate_static:
                g.apply_slow(free, static_occ, cfg.slow_rate, cfg.confident_abs)
            dyn = self.dynamic.reshape(-1)
            dyn[free] = np.maximum(dyn[free] - np.float32(cfg.dynamic_clear), 0.0)
            dyn[new_cells] = np.minimum(dyn[new_cells] + np.float32(cfg.dynamic_hit), np.float32(cfg.dynamic_max))
            self._seen_free(free, now)
        return LayerUpdate(labels, tuple(self.blobs()))

    def _clusters(self, scan: Scan, pose: Pose2D, new: np.ndarray):
        cfg = self.config
        sub = Scan(scan.stamp_s, scan.seq, scan.angles_rad, np.where(new, scan.ranges_m, np.nan), scan.quality)
        obs = cluster_scan(sub, break_distance_m=cfg.cluster_break_m, min_points=cfg.cluster_min_points)
        out = []
        for o in obs:
            c = pose.transform_points(np.array([o.centroid_xy]))[0]
            out.append((c, o.width_m, o.n_points))
        return out

    def _track(self, scan: Scan, pose: Pose2D, new: np.ndarray, labels: np.ndarray, now: float) -> None:
        cfg = self.config
        clusters = self._clusters(scan, pose, new)
        pairs = []
        for ci, (c, _, _) in enumerate(clusters):
            for bi, b in enumerate(self._blobs):
                d = float(np.hypot(*(c - b.xy)))
                if d <= cfg.blob_match_m:
                    pairs.append((d, ci, bi))
        pairs.sort()
        used_c, used_b, assoc = set(), set(), {}
        for d, ci, bi in pairs:
            if ci in used_c or bi in used_b:
                continue
            used_c.add(ci)
            used_b.add(bi)
            assoc[ci] = bi
        for ci, (c, w, n) in enumerate(clusters):
            if ci in assoc:
                b = self._blobs[assoc[ci]]
                b.xy, b.width, b.n, b.last_s = c, w, n, now
                b.seen += 1
                b.free_views = 0
                if float(np.hypot(*(c - b.anchor))) > cfg.still_m:
                    b.anchor, b.still_since_s = c.copy(), now
                if not b.promoted and now - b.still_since_s >= cfg.promote_after_s:
                    b.promoted = True
            else:
                b = _Blob(self._next_id, c, w, n, now, now, anchor=c.copy(), still_since_s=now)
                self._next_id += 1
                self._blobs.append(b)
            b.history.append((now, c.copy()))
            if not b.promoted:
                self.last_clusters.append((c, w, n))
        # beams of promoted blobs become ordinary room hits (slow static update)
        promoted = [b for b in self._blobs if b.promoted and b.last_s == now]
        if promoted:
            idx = np.flatnonzero(new)
            r = scan.ranges_m[idx]
            a = scan.angles_rad[idx] + pose.theta
            pts = np.column_stack((pose.x + r * np.cos(a), pose.y + r * np.sin(a)))
            for b in promoted:
                near = np.hypot(pts[:, 0] - b.xy[0], pts[:, 1] - b.xy[1]) <= b.width / 2.0 + 0.1
                labels[idx[near]] = ROOM
        self._prune(now)

    def _track_none(self, now: float) -> None:
        self._prune(now)

    def _seen_free(self, free: np.ndarray, now: float) -> None:
        """Blobs not detected this scan whose centre cell a ray crossed count a 'seen free' view."""
        g = self.static
        keep = []
        for b in self._blobs:
            if b.last_s != now:
                cr = g.world_to_cell(b.xy[None, :])
                if g.in_bounds(cr)[0] and free[int(cr[0, 1]) * g.width + int(cr[0, 0])]:
                    b.free_views += 1
            if b.free_views < self.config.blob_free_views:
                keep.append(b)
        self._blobs = keep

    def _prune(self, now: float) -> None:
        cfg = self.config
        self._blobs = [b for b in self._blobs if now - b.last_s <= cfg.blob_max_unseen_s
                       and not (b.promoted and now - b.last_s > 5.0)]
        if len(self._blobs) > cfg.max_blobs:
            self._blobs.sort(key=lambda b: -b.last_s)
            del self._blobs[cfg.max_blobs:]

    def _moving(self, b: _Blob) -> bool:
        cfg = self.config
        if len(b.history) < 2:
            return False
        t_last, p_last = b.history[-1]
        for t, p in reversed(b.history):
            if t_last - t >= cfg.moving_window_s:
                break
        if t_last - t <= 0:
            return False
        return bool(np.hypot(*(p_last - p)) > cfg.moving_m * (t_last - t) / cfg.moving_window_s)

    def blobs(self, include_promoted: bool = False, min_persistence: int = 1) -> list[ChangeBlob]:
        """Current blobs; ``min_persistence`` hides one-scan flickers (mixed pixels, noise)."""
        return [b.snapshot(self._moving(b)) for b in self._blobs
                if (include_promoted or not b.promoted) and b.seen >= min_persistence]

    # ------------------------------------------------------------------ planning
    def planning_grid(self) -> OccupancyGrid:
        """A copy of ``static`` where dynamic cells (>= ``dynamic_occ``) and blocking extra layers
        are occupied (cell-wise max of log-odds): what ``nav.build_costmap`` should plan on."""
        g = OccupancyGrid(self.static.width, self.static.height, self.static.resolution_m,
                          self.static.origin_xy, self.static.l_occ, self.static.l_free,
                          self.static.l_min, self.static.l_max)
        lo = self.static.log_odds.copy()
        dyn = self.dynamic >= self.config.dynamic_occ
        lo[dyn] = np.maximum(lo[dyn], self.dynamic[dyn])
        for lay in self.extra.values():
            thr = lay.params.get("blocks_above")
            if thr is None:
                continue
            m = lay.values() >= float(thr)
            lo[m] = np.float32(g.l_max)
        g.log_odds = lo
        return g
