"""Localize the car inside a saved ``.lbmap`` and keep that map slowly up to date (M1).

``MapLocalizer`` matches each scan against a likelihood field built once from the saved map's
OCCUPIED cell centres (the same correlative matcher as ``odometry.py``, but against the whole
room instead of the last few keyframes, so the pose does not drift).

* ``initialize(scan, guess, search_xy_m, search_theta_rad)``: with a guess (dock pose, operator
  click) every pose in the window is scored; with ``guess=None`` a coarse grid over the map's
  FREE cells times ``global_headings`` headings. The best few distinct candidates are refined
  with ``match_scan``; when the runner-up is a *different* pose scoring nearly as well
  (symmetric rooms, featureless corridors) the result is ``ambiguous`` and the localizer stays
  uninitialized instead of guessing.
* ``update(scan)``: constant-velocity prediction + a small search window. If the map match is
  weak (mostly new obstacles, people around) it falls back to scan-to-scan matching against the
  last few map-matched scans for at most ``max_odometry_updates`` updates (``source ==
  "odometry"``). ``lost`` is declared after ``lost_after`` updates with neither, after
  ``max_odometry_updates`` without the map, or after two map scores below
  ``lost_score_floor``. A lost localizer holds the last good pose (it never silently jumps or
  drifts) and retries a local re-initialization around it every ``reinit_every`` scans.

``LocalizingMapper`` is the ``Mapper`` counterpart for a saved map: it localizes every scan and
integrates it with ``OccupancyGrid.integrate_slow`` (damped increments for confident cells) only
when the pose came from a converged, unambiguous map match. With ``layered=True`` (M2) the
update goes through ``layers.LayeredMap`` instead: new/moving things go to a short-lived dynamic
layer and never into the room map.

Tests use synthetic rooms (``locatebot.lidar.sim``): they prove software behaviour, not
real-sensor behaviour. Real recordings can be replayed with ``tools/relocalize.py``.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .detect import clean_scan
from .grid import FREE, OCCUPIED, OccupancyGrid
from .layers import LayeredMap, LayersConfig
from .mapfile import export_ros_map, load_map_layers, save_map
from .matching import LikelihoodField, _score_grid, _subsample, match_scan
from .types import RANGE_MAX_M, Pose2D, Scan, wrap_angle


@dataclass(frozen=True)
class LocalizerConfig:
    sigma_m: float = 0.10                  # likelihood-field sigma (fine field)
    occ_threshold: float = 0.65            # map cells with p > this are reference points
    free_threshold: float = 0.35           # map cells with p < this are candidate positions
    max_points: int = 600                  # scan points used for matching
    min_score: float = 0.35                # mean field value for a converged match
    track_xy_m: float = 0.25               # tracking search half-window
    track_theta_rad: float = 0.30
    init_xy_m: float = 1.0                 # default window around a guess
    init_theta_rad: float = math.radians(45.0)
    global_xy_step_m: float = 0.2          # candidate spacing for initialization
    global_headings: int = 72              # headings over 360 deg for a global init (5 deg)
    global_points: int = 100               # scan points scored per candidate
    n_candidates: int = 8                  # distinct candidates refined with match_scan
    distinct_xy_m: float = 0.3             # two solutions further apart than this are different
    distinct_theta_rad: float = math.radians(10.0)
    ambiguity_ratio: float = 0.95          # runner-up >= ratio * best (and distinct) -> ambiguous
    lost_after: int = 5                    # consecutive updates with neither map nor odometry match
    lost_score_floor: float = 0.12         # two consecutive map scores below this -> lost
    max_odometry_updates: int = 10         # consecutive updates without the map before lost
    odometry_scans: int = 3                # map-matched scans kept for the odometry fallback
    reinit_every: int = 10                 # scans between (re)initialization attempts
    recover_global: bool = True            # recover from lost with a global search (else local)
    recover_xy_m: float = 1.0              # local re-init window around the last good pose
    recover_theta_rad: float = math.radians(45.0)
    recover_score_ratio: float = 0.85      # a recovery must score >= this x the pre-loss score
    rebuild_changed_cells: int = 30        # rebuild the field when this many cells changed class

    def __post_init__(self) -> None:
        if self.global_headings < 1 or self.n_candidates < 1 or self.max_points < 1:
            raise ValueError("global_headings, n_candidates and max_points must be >= 1")
        if not 0.0 < self.ambiguity_ratio <= 1.0:
            raise ValueError("ambiguity_ratio must be in (0, 1]")


@dataclass(frozen=True)
class LocalizeResult:
    pose: Pose2D
    score: float
    converged: bool
    ambiguous: bool = False
    lost: bool = False
    source: str = "map"          # "map" | "odometry" | "init" | "prediction" | "none"
    runner_up: Pose2D | None = None
    runner_up_score: float = 0.0

    @property
    def accepted(self) -> bool:
        """The pose is usable (a converged map/odometry match, not lost, not ambiguous)."""
        return self.converged and not self.lost and not self.ambiguous


def _distinct(a: Pose2D, b: Pose2D, dxy: float, dth: float) -> bool:
    return math.hypot(a.x - b.x, a.y - b.y) > dxy or abs(wrap_angle(a.theta - b.theta)) > dth


class MapLocalizer:
    """Track the robot pose in a fixed map frame (the saved map's)."""

    def __init__(self, grid: OccupancyGrid, config: LocalizerConfig | None = None,
                 guess: Pose2D | None = None) -> None:
        self.config = config or LocalizerConfig()
        self._grid = grid
        self._field: LikelihoodField | None = None
        self._occ_mask: np.ndarray | None = None
        self.rebuild(grid)
        self._pose = guess if guess is not None else Pose2D(0.0, 0.0, 0.0)
        self._pending_guess = guess
        self._initialized = False
        self._velocity = Pose2D(0.0, 0.0, 0.0)
        self._last_good: Pose2D | None = None
        self._step = 0
        self._last_good_step = 0
        self._last_init_step: int | None = None
        self._misses = 0      # consecutive updates without a map match
        self._blind = 0       # consecutive updates with neither map nor odometry
        self._low = 0         # consecutive map scores below the floor
        self._lost = False
        self._ambiguous = False
        self._recent: deque[np.ndarray] = deque(maxlen=self.config.odometry_scans)
        self._odo_field: LikelihoodField | None = None
        self._good_score: float | None = None  # running mean of converged map scores
        self.n_lost_events = 0
        self.n_recoveries = 0

    # ------------------------------------------------------------------ map / field
    @property
    def field(self) -> LikelihoodField:
        return self._field

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def lost(self) -> bool:
        return self._lost

    @property
    def ambiguous(self) -> bool:
        return self._ambiguous

    @property
    def pose(self) -> Pose2D:
        return self._pose

    def _occupied(self, grid: OccupancyGrid) -> np.ndarray:
        p = grid.probability()
        return p > self.config.occ_threshold

    def rebuild(self, grid: OccupancyGrid | None = None) -> None:
        """(Re)build the likelihood field from the map's OCCUPIED cell centres."""
        if grid is not None:
            self._grid = grid
        occ = self._occupied(self._grid)
        pts = reference_points(self._grid, occ)
        self._field = LikelihoodField.from_points(pts, self._grid.resolution_m, self.config.sigma_m)
        self._occ_mask = occ

    def maybe_rebuild(self, grid: OccupancyGrid | None = None) -> bool:
        """Rebuild when at least ``rebuild_changed_cells`` cells changed occupied/not since the
        last build (the slowly updated map drifted away from the field). Returns True if rebuilt."""
        g = grid if grid is not None else self._grid
        changed = int(np.count_nonzero(self._occupied(g) != self._occ_mask))
        if changed >= self.config.rebuild_changed_cells:
            self.rebuild(g)
            return True
        return False

    def _points(self, scan: Scan, n: int | None = None) -> np.ndarray:
        return _subsample(scan.points_xy(), n or self.config.max_points)

    # ------------------------------------------------------------------ initialization
    def request_init(self, guess: Pose2D | None = None) -> None:
        """Forget the current track and (re)initialize on the next ``update`` (operator click)."""
        self._pending_guess = guess
        self._initialized = False
        self._lost = False
        self._last_init_step = None

    def _candidates(self, pts: np.ndarray, guess: Pose2D | None, sxy: float,
                    sth: float) -> list[tuple[float, Pose2D]]:
        cfg = self.config
        g = self._grid
        cls = g.classify(cfg.occ_threshold, cfg.free_threshold)
        step = cfg.global_xy_step_m
        if guess is None:
            rows, cols = np.nonzero(cls == FREE)
            if len(rows) == 0:
                return []
            lo = g.cell_to_world(np.array([[cols.min(), rows.min()]]))[0]
            hi = g.cell_to_world(np.array([[cols.max(), rows.max()]]))[0]
            xs = np.arange(lo[0], hi[0] + 1e-9, step)
            ys = np.arange(lo[1], hi[1] + 1e-9, step)
            n = cfg.global_headings
            thetas = -math.pi + 2.0 * math.pi * np.arange(n) / n
            allowed = (FREE,)
        else:
            nxy = max(0, int(math.ceil(sxy / step - 1e-9)))
            xs = guess.x + np.linspace(-nxy * step, nxy * step, 2 * nxy + 1)
            ys = guess.y + np.linspace(-nxy * step, nxy * step, 2 * nxy + 1)
            tstep = 2.0 * math.pi / cfg.global_headings
            nth = max(0, int(math.ceil(sth / tstep - 1e-9)))
            thetas = guess.theta + np.linspace(-nth * tstep, nth * tstep, 2 * nth + 1)
            allowed = (FREE, 255)  # near a guess the car may stand in a still-unknown cell
        # positions that are allowed in the map (FREE, or not OCCUPIED near a guess)
        cr = g.world_to_cell(np.array(np.meshgrid(xs, ys, indexing="ij")).reshape(2, -1).T)
        inb = g.in_bounds(cr)
        ok = np.zeros(len(cr), dtype=bool)
        cc = cr[inb]
        ok[inb] = np.isin(cls[cc[:, 1], cc[:, 0]], allowed)
        ok = ok.reshape(len(xs), len(ys))
        if not ok.any():
            return []
        sub = _subsample(pts, cfg.global_points)
        cf = self._field.coarse
        scores = np.empty((len(thetas), len(xs), len(ys)), dtype=np.float32)
        chunk = max(1, int(4_000_000 // max(1, len(xs) * len(ys) * len(sub))))
        for i in range(0, len(thetas), chunk):
            scores[i:i + chunk] = _score_grid(cf, sub, xs, ys, thetas[i:i + chunk], exact=False)
        scores[:, ~ok] = -1.0
        order = np.argsort(scores, axis=None)[::-1]
        out: list[tuple[float, Pose2D]] = []
        for flat in order[: 20000]:
            t, i, j = np.unravel_index(flat, scores.shape)
            s = float(scores[t, i, j])
            if s <= 0.0:
                break
            p = Pose2D(float(xs[i]), float(ys[j]), float(thetas[t]))
            if all(_distinct(p, q, 2.0 * step, 4.0 * math.pi / cfg.global_headings) for _, q in out):
                out.append((s, p))
                if len(out) >= cfg.n_candidates:
                    break
        return out

    def initialize(self, scan: Scan, guess: Pose2D | None = None, search_xy_m: float | None = None,
                   search_theta_rad: float | None = None) -> LocalizeResult:
        """Find the pose in the map: around ``guess`` (window defaults from the config) or, with
        ``guess=None``, over all FREE cells and headings. Sets the track only when the best
        solution is converged and not ambiguous."""
        cfg = self.config
        sxy = cfg.init_xy_m if search_xy_m is None else search_xy_m
        sth = cfg.init_theta_rad if search_theta_rad is None else search_theta_rad
        self._last_init_step = self._step
        pts = self._points(scan)
        fail_pose = guess if guess is not None else self._pose
        if len(pts) == 0:
            return LocalizeResult(fail_pose, 0.0, False, source="none", lost=self._lost)
        cands = self._candidates(pts, guess, sxy, sth)
        refined: list[tuple[float, bool, Pose2D]] = []  # (score, converged, pose)
        step = cfg.global_xy_step_m
        th_step = 2.0 * math.pi / cfg.global_headings
        for _, p in cands:
            # Refine within one candidate spacing. A result pinned to the window edge is on the
            # slope of a peak further away: follow it (twice at most). A candidate that never
            # settles is not a separate solution and does not count towards ambiguity.
            for _ in range(3):
                r = match_scan(self._field, pts, p, search_xy_m=step, search_theta_rad=th_step,
                               min_score=cfg.min_score)
                p = r.pose
                if r.converged or r.score < cfg.min_score:
                    break
            if r.converged:
                refined.append((r.score, True, r.pose))
        refined.sort(key=lambda t: -t[0])
        uniq: list[tuple[float, bool, Pose2D]] = []
        for item in refined:
            if all(_distinct(item[2], u[2], cfg.distinct_xy_m, cfg.distinct_theta_rad) for u in uniq):
                uniq.append(item)
        if not uniq:
            self._ambiguous = False
            return LocalizeResult(fail_pose, 0.0, False, source="init", lost=self._lost)
        best_score, _, best_pose = uniq[0]
        # polish the winner with the full tracking window (finest steps, converged flag)
        fin = match_scan(self._field, pts, best_pose, search_xy_m=cfg.track_xy_m,
                         search_theta_rad=cfg.track_theta_rad, min_score=cfg.min_score)
        if fin.score >= best_score:
            best_score, best_pose = fin.score, fin.pose
        runner = uniq[1] if len(uniq) > 1 else None
        ambiguous = runner is not None and runner[0] >= cfg.ambiguity_ratio * best_score
        res = LocalizeResult(best_pose, best_score, not ambiguous,
                             ambiguous=ambiguous, lost=self._lost and ambiguous, source="init",
                             runner_up=None if runner is None else runner[2],
                             runner_up_score=0.0 if runner is None else runner[0])
        self._ambiguous = ambiguous
        if not ambiguous:
            if self._lost:
                self.n_recoveries += 1
            self._set_track(best_pose, pts)
        return res

    def _set_track(self, pose: Pose2D, pts: np.ndarray) -> None:
        self._pose = pose
        self._initialized = True
        self._lost = False
        self._pending_guess = None
        self._velocity = Pose2D(0.0, 0.0, 0.0)
        self._last_good = pose
        self._last_good_step = self._step
        self._misses = self._blind = self._low = 0
        self._recent.clear()
        self._remember(pose, pts)

    def _remember(self, pose: Pose2D, pts: np.ndarray) -> None:
        self._recent.append(pose.transform_points(pts))
        self._odo_field = None

    # ------------------------------------------------------------------ tracking
    def update(self, scan: Scan) -> LocalizeResult:
        """Localize one scan (cleaned or raw, time order)."""
        cfg = self.config
        self._step += 1
        due = self._last_init_step is None or self._step - self._last_init_step >= cfg.reinit_every
        if not self._initialized:
            if not due:
                return LocalizeResult(self._pose, 0.0, False, ambiguous=self._ambiguous,
                                      source="none")
            return self.initialize(scan, self._pending_guess)
        if self._lost:
            if due:
                res = self._recover(scan)
                if res is not None:
                    return res
            return LocalizeResult(self._pose, 0.0, False, lost=True, source="none")

        pts = self._points(scan)
        predicted = self._pose.compose(self._velocity)
        if len(pts) == 0:
            res_map_score, conv = 0.0, False
            mres = None
        else:
            mres = match_scan(self._field, pts, predicted, search_xy_m=cfg.track_xy_m,
                              search_theta_rad=cfg.track_theta_rad, min_score=cfg.min_score)
            res_map_score, conv = mres.score, mres.converged
        self._low = self._low + 1 if res_map_score < cfg.lost_score_floor else 0

        if conv:
            self._misses = self._blind = 0
            self._good_score = mres.score if self._good_score is None else (
                0.9 * self._good_score + 0.1 * mres.score)
            self._accept(mres.pose)
            self._remember(mres.pose, pts)
            return LocalizeResult(mres.pose, mres.score, True, source="map")

        self._misses += 1
        source, pose = "prediction", predicted
        if len(pts) and self._recent and self._misses <= cfg.max_odometry_updates:
            if self._odo_field is None:
                self._odo_field = LikelihoodField.from_points(np.vstack(self._recent),
                                                              self._grid.resolution_m, cfg.sigma_m)
            ores = match_scan(self._odo_field, pts, predicted, search_xy_m=cfg.track_xy_m,
                              search_theta_rad=cfg.track_theta_rad, min_score=cfg.min_score)
            if ores.converged:
                source, pose = "odometry", ores.pose
        if source == "odometry":
            self._blind = 0
            self._accept(pose)
        else:
            self._blind += 1
            self._pose = predicted
        if (self._blind >= cfg.lost_after or self._misses > cfg.max_odometry_updates
                or self._low >= 2):
            self._declare_lost()
            return LocalizeResult(self._pose, res_map_score, False, lost=True, source=source)
        return LocalizeResult(pose, res_map_score, source == "odometry", source=source)

    def _recover(self, scan: Scan) -> LocalizeResult | None:
        """One recovery attempt while lost: search (globally by default), accept only an
        unambiguous solution that scores close to what tracking scored before the loss."""
        cfg = self.config
        self._last_init_step = self._step
        pts = self._points(scan)
        if len(pts) == 0:
            return None
        saved = self._snapshot()
        if cfg.recover_global:
            res = self.initialize(scan, None)
        else:
            res = self.initialize(scan, self._last_good, cfg.recover_xy_m, cfg.recover_theta_rad)
        floor = cfg.recover_score_ratio * (self._good_score or 0.0)
        if res.accepted and res.score >= floor:
            return res
        self._restore(saved)
        return None

    def _snapshot(self) -> tuple:
        return (self._pose, self._initialized, self._lost, self._velocity, self._last_good,
                self._last_good_step, self._misses, self._blind, self._low, tuple(self._recent),
                self.n_recoveries)

    def _restore(self, s: tuple) -> None:
        (self._pose, self._initialized, self._lost, self._velocity, self._last_good,
         self._last_good_step, self._misses, self._blind, self._low, recent, self.n_recoveries) = s
        self._recent = deque(recent, maxlen=self.config.odometry_scans)
        self._odo_field = None

    def _accept(self, pose: Pose2D) -> None:
        gap = max(1, self._step - self._last_good_step)
        d = self._last_good.inverse().compose(pose)
        self._velocity = Pose2D(d.x / gap, d.y / gap, d.theta / gap)
        self._last_good = pose
        self._last_good_step = self._step
        self._pose = pose

    def _declare_lost(self) -> None:
        if not self._lost:
            self.n_lost_events += 1
        self._lost = True
        self._pose = self._last_good if self._last_good is not None else self._pose
        self._velocity = Pose2D(0.0, 0.0, 0.0)
        self._last_init_step = self._step  # first recovery attempt after reinit_every scans


class LocalizingMapper:
    """``Mapper`` counterpart for a saved map: localize each scan, update the map slowly.

    Only scans whose pose came from a converged, unambiguous *map* match are integrated
    (``integrate_slow``: confident cells move at ``slow_rate`` x the normal increment).
    """

    def __init__(self, grid: OccupancyGrid, meta: dict | None = None, guess: Pose2D | None = None,
                 config: LocalizerConfig | None = None, slow_rate: float = 0.2,
                 max_range_m: float = RANGE_MAX_M, rebuild_check_every: int = 25,
                 layered: bool = False, layers_config: LayersConfig | None = None,
                 extra_layers: dict | None = None) -> None:
        self.grid = grid
        self.layers: LayeredMap | None = None
        if layered:
            lc = layers_config or LayersConfig(slow_rate=slow_rate, max_range_m=max_range_m)
            self.layers = LayeredMap(grid, lc, extra=extra_layers)
        self.meta = dict(meta or {})
        self.localizer = MapLocalizer(grid, config, guess=guess)
        self.slow_rate = float(slow_rate)
        self.max_range_m = max_range_m
        self.rebuild_check_every = int(rebuild_check_every)
        self.trajectory: list[tuple[float, float, float, float]] = []
        self.n_scans = 0
        self.n_integrated = 0
        self.n_rejected = 0
        self.n_rebuilds = 0
        self.last: LocalizeResult | None = None
        self.home = map_home(self.meta, guess)

    @classmethod
    def from_file(cls, path, guess: Pose2D | None = None, **kw) -> "LocalizingMapper":
        grid, meta, extra = load_map_layers(path)
        if kw.get("layered"):
            kw.setdefault("extra_layers", extra)
        return cls(grid, meta, guess=guess, **kw)

    def add_scan(self, scan: Scan) -> LocalizeResult:
        self.n_scans += 1
        cleaned = clean_scan(scan)
        res = self.localizer.update(cleaned)
        self.last = res
        if res.accepted:
            p = res.pose
            self.trajectory.append((cleaned.stamp_s, p.x, p.y, p.theta))
            if res.source in ("map", "init"):
                if self.layers is not None:
                    self.layers.update(cleaned, p, cleaned.stamp_s)
                else:
                    self.grid.integrate_slow(cleaned, p, rate=self.slow_rate, max_range_m=self.max_range_m)
                self.n_integrated += 1
                if self.n_integrated % self.rebuild_check_every == 0 and self.localizer.maybe_rebuild(self.grid):
                    self.n_rebuilds += 1
            elif self.layers is not None:  # odometry-fallback pose: track changes, keep the room
                self.layers.update(cleaned, p, cleaned.stamp_s, integrate_static=False)
        else:
            self.n_rejected += 1
            if self.layers is not None:
                self.layers.decay(cleaned.stamp_s)
        return res

    def save(self, stem: str | Path, meta: dict | None = None) -> dict[str, Path]:
        """Write <stem>.lbmap/.pgm/.yaml: the updated map, the original metadata (incl. its
        mapping trajectory) plus this session's summary."""
        stem = Path(stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        full = dict(self.meta)
        full.update(meta or {})
        loc = self.localizer
        full.update({
            "format": "locatebot-session-map",
            "updated_unix_s": time.time(),
            "sessions": int(self.meta.get("sessions", 1)) + 1,
            "home": [self.home.x, self.home.y, self.home.theta],
            "last_session": {"mode": "localizing", "scans": self.n_scans, "integrated": self.n_integrated,
                             "rejected": self.n_rejected, "lost_events": loc.n_lost_events,
                             "slow_rate": self.slow_rate},
        })
        paths = {k: stem.with_name(stem.name + "." + k) for k in ("lbmap", "pgm", "yaml")}
        save_map(paths["lbmap"], self.grid, full,
                 layers=self.layers.save_layers() if self.layers is not None else None)
        export_ros_map(self.grid, paths["pgm"], paths["yaml"])
        return paths


def reference_points(grid: OccupancyGrid, occ: np.ndarray) -> np.ndarray:
    """(N, 2) matcher reference points: one per OCCUPIED cell, at the log-odds-weighted
    centroid of the occupied cells in its 3x3 neighbourhood.

    A wall that lies on a cell boundary is two cells thick in the grid; plain cell centres
    leave a flat-topped likelihood across those two cells and the pose wanders by up to half a
    cell (measured ~3 cm on the synthetic room). The centroid puts the point on the wall line;
    a one-cell-thick wall keeps its cell centre (the neighbours along the wall share its offset).
    """
    rows, cols = np.nonzero(occ)
    if len(rows) == 0:
        return np.zeros((0, 2))
    w = np.where(occ, np.maximum(grid.log_odds, 0.0), 0.0).astype(np.float64)
    h, wd = w.shape
    pad = np.pad(w, 1)
    sw = np.zeros(len(rows))
    sx = np.zeros(len(rows))
    sy = np.zeros(len(rows))
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            ww = pad[rows + 1 + dr, cols + 1 + dc]
            sw += ww
            sx += ww * dc
            sy += ww * dr
    sw = np.maximum(sw, 1e-12)
    res = grid.resolution_m
    centres = grid.cell_to_world(np.column_stack((cols, rows)))
    return centres + np.column_stack((sx / sw, sy / sw)) * res


def map_home(meta: dict, guess: Pose2D | None = None) -> Pose2D:
    """Home pose of a saved map: ``meta["home"]``, else the first mapping trajectory pose,
    else ``guess``, else the map origin."""
    h = meta.get("home")
    try:
        if h is not None:
            return Pose2D(float(h[0]), float(h[1]), float(h[2]))
        traj = meta.get("trajectory")
        if traj:
            return Pose2D(float(traj[0][1]), float(traj[0][2]), float(traj[0][3]))
    except (TypeError, ValueError, IndexError):
        pass
    return guess if guess is not None else Pose2D(0.0, 0.0, 0.0)


__all__ = ["LocalizerConfig", "LocalizeResult", "MapLocalizer", "LocalizingMapper", "map_home", "OCCUPIED"]
