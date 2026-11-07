"""Per-scan obstacle detection: cleaning, sector/corridor clearance, coverage, clustering.

Pure, stateless functions of a `Scan`. They run on the Pi every revolution and feed the
emergency stop, so the semantics are fail-safe:

* NaN / no return / filtered beams mean UNKNOWN, never free.
* `sector_clearance` reports NaN (unknown) for a sector with no valid return.
* `corridor_clearance` returns ``math.inf`` when no valid point lies in the corridor; that
  only means "nothing seen", so callers must gate it on `forward_coverage`.
* `clean_scan` only removes a point when its neighbours *prove* it is an isolated FAR
  spike; a point CLOSER than both neighbours is kept (it may be a thin obstacle such as a
  pole or chair leg), and a point next to an unknown beam is kept.
* `forward_coverage` measures ANGULAR coverage (bins with a valid return), so beams that
  are missing entirely count as uncovered, not just beams that returned invalid.

"Valid" everywhere means `Scan.valid_mask()` (finite and within RANGE_MIN_M..RANGE_MAX_M).
Quality filtering happens in `clean_scan`; run it first on raw sensor scans.

The tests use synthetic scans from `locatebot.lidar.sim`; they prove software behaviour,
not real RPLIDAR C1 behaviour (e.g. real mixed-pixel edges or glass returns).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .types import RANGE_MAX_M, RANGE_MIN_M, Scan

_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Obstacle:
    centroid_xy: tuple[float, float]
    nearest_xy: tuple[float, float]
    nearest_range_m: float
    bearing_rad: float          # bearing of nearest point
    width_m: float              # distance between first and last point of the cluster
    n_points: int


def _wrap(a: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi)."""
    return (a + math.pi) % _TWO_PI - math.pi


def clean_scan(scan: Scan, range_min_m: float = RANGE_MIN_M, range_max_m: float = RANGE_MAX_M,
               min_quality: int = 1, spike_jump_m: float = 0.3) -> Scan:
    """Return a copy of `scan` with bad beams set to NaN (unknown).

    Removed: non-finite, out of [range_min_m, range_max_m], quality < min_quality, and isolated
    FAR spikes: a point that is FARTHER than BOTH angular neighbours by > spike_jump_m while
    those neighbours agree with each other within spike_jump_m. A point CLOSER than both
    neighbours is never removed: for the emergency stop it may be a real thin obstacle hit by
    a single beam, and dropping it would report the wall behind it. Neighbours are taken in angular
    order (angles need not be sorted) and wrap across the -pi/+pi seam; they are the
    neighbouring beams after range/quality filtering, so a point next to an unknown beam is
    never called a spike. Angles, quality, stamp and seq are unchanged.
    """
    r = np.array(scan.ranges_m, dtype=np.float64, copy=True)
    with np.errstate(invalid="ignore"):
        bad = ~np.isfinite(r) | (r < range_min_m) | (r > range_max_m) | (scan.quality < min_quality)
    r[bad] = np.nan

    n = len(r)
    if n >= 3:
        order = np.argsort(_wrap(scan.angles_rad), kind="stable")
        rs = r[order]
        prev = np.roll(rs, 1)
        nxt = np.roll(rs, -1)
        # Comparisons involving NaN are False, so unknown neighbours never flag a spike.
        # Only FAR outliers: a near single-beam return may be a thin obstacle and is kept.
        spike = (rs - prev > spike_jump_m) & (rs - nxt > spike_jump_m) & \
                (np.abs(prev - nxt) <= spike_jump_m)
        r[order[spike]] = np.nan

    return Scan(stamp_s=scan.stamp_s, seq=scan.seq, angles_rad=scan.angles_rad, ranges_m=r,
                quality=scan.quality)


def sector_clearance(scan: Scan, n_sectors: int = 8) -> np.ndarray:
    """Minimum valid range per angular sector, shape (n_sectors,).

    Sector 0 is centred on forward (angle 0), sectors follow in CCW order, each 2*pi/n_sectors
    wide; sector k covers [k*w - w/2, k*w + w/2). A sector with no valid return is NaN
    (unknown, NOT free).
    """
    if n_sectors < 1:
        raise ValueError("n_sectors must be >= 1")
    m = scan.valid_mask()
    w = _TWO_PI / n_sectors
    idx = (np.floor(((scan.angles_rad[m] + w / 2.0) % _TWO_PI) / w).astype(np.intp)) % n_sectors
    out = np.full(n_sectors, np.inf)
    np.minimum.at(out, idx, scan.ranges_m[m])
    out[np.isinf(out)] = np.nan
    return out


def corridor_clearance(scan: Scan, half_width_m: float, max_lookahead_m: float = RANGE_MAX_M) -> float:
    """Smallest x > 0 among valid points with |y| <= half_width_m and x <= max_lookahead_m.

    Returns math.inf if no such point. inf means "nothing seen", NOT "verified free": a scan
    with missing or invalid beams ahead also gives inf. Callers MUST require a high
    `forward_coverage` (e.g. close to 1.0 over the driving FOV) before treating inf, or any
    large value, as clear; otherwise treat the corridor as unknown and stop.
    """
    p = scan.points_xy()
    x, y = p[:, 0], p[:, 1]
    sel = (x > 0.0) & (np.abs(y) <= half_width_m) & (x <= max_lookahead_m)
    if not sel.any():
        return math.inf
    return float(x[sel].min())


def forward_coverage(scan: Scan, fov_rad: float = math.pi, bin_rad: float = math.radians(2.0)) -> float:
    """Fraction of angular bins across [-fov_rad/2, +fov_rad/2] that contain a valid return.

    The FOV is split into ceil(fov_rad / bin_rad) equal bins (width <= bin_rad). A bin counts
    as covered only if at least one VALID beam (`Scan.valid_mask()`) falls in it; bins with no
    beams at all, or only invalid beams, are uncovered. So a scan missing every beam in a
    forward wedge reports that wedge as uncovered, unlike a per-beam fraction. Returns 0.0 for
    an empty scan or no valid return in the FOV. fov_rad >= 2*pi covers the full circle.
    Use it to gate `corridor_clearance` / `sector_clearance` before trusting "nothing seen".
    """
    if fov_rad <= 0.0 or bin_rad <= 0.0:
        raise ValueError("fov_rad and bin_rad must be > 0")
    fov = min(fov_rad, _TWO_PI)
    n_bins = max(1, math.ceil(fov / bin_rad - 1e-9))
    w = fov / n_bins
    a = _wrap(scan.angles_rad[scan.valid_mask()])
    half = fov / 2.0
    if fov < _TWO_PI:
        a = a[np.abs(a) <= half]
    idx = np.clip(np.floor((a + half) / w).astype(np.intp), 0, n_bins - 1)
    covered = np.zeros(n_bins, dtype=bool)
    covered[idx] = True
    return float(covered.sum()) / n_bins


def cluster_scan(scan: Scan, break_distance_m: float = 0.15, break_ratio: float = 0.03,
                 min_points: int = 3) -> list[Obstacle]:
    """Split valid points into angular clusters and return them sorted by nearest_range_m.

    Points are sorted by (wrapped) angle. A new cluster starts where the Euclidean gap between
    consecutive points exceeds break_distance_m + break_ratio * range, using the larger of the
    two ranges (beam spacing grows with the farther point). The last and first clusters are
    joined across the -pi/+pi seam when that gap is below the threshold. Clusters with fewer
    than min_points are dropped. Unknown (NaN) beams are skipped, so a cluster may bridge a
    dropout if its neighbours are close in space.
    """
    m = scan.valid_mask()
    n = int(m.sum())
    if n == 0:
        return []
    a = _wrap(scan.angles_rad[m])
    order = np.argsort(a, kind="stable")
    a = a[order]
    r = scan.ranges_m[m][order]
    x = r * np.cos(a)
    y = r * np.sin(a)

    # gap[i] is between point i and point i+1 (mod n); gap[n-1] is the seam gap.
    nx, ny, nr = np.roll(x, -1), np.roll(y, -1), np.roll(r, -1)
    gap = np.hypot(nx - x, ny - y)
    brk = gap > break_distance_m + break_ratio * np.maximum(r, nr)
    if n == 1:
        brk[:] = True

    inner = np.flatnonzero(brk[:-1]) + 1          # cluster starts after inner breaks
    starts = np.concatenate(([0], inner))
    if len(starts) > 1 and not brk[-1]:
        # Join last and first cluster across the seam: rotate so the last cluster comes first.
        shift = int(starts[-1])
        rot = np.roll(np.arange(n), -shift)
        x, y, r = x[rot], y[rot], r[rot]
        starts = np.concatenate(([0], starts[1:-1] + (n - shift)))

    counts = np.diff(np.append(starts, n))
    keep = counts >= min_points
    if not keep.any():
        return []

    labels = np.repeat(np.arange(len(starts)), counts)
    nearest = np.lexsort((r, labels))[starts]      # argmin of r within each cluster
    ends = starts + counts - 1
    cx = np.add.reduceat(x, starts) / counts
    cy = np.add.reduceat(y, starts) / counts
    width = np.hypot(x[ends] - x[starts], y[ends] - y[starts])

    obstacles = [
        Obstacle(
            centroid_xy=(float(cx[k]), float(cy[k])),
            nearest_xy=(float(x[j]), float(y[j])),
            nearest_range_m=float(r[j]),
            bearing_rad=float(math.atan2(y[j], x[j])),
            width_m=float(width[k]),
            n_points=int(counts[k]),
        )
        for k, j in zip(np.flatnonzero(keep), nearest[keep])
    ]
    obstacles.sort(key=lambda o: o.nearest_range_m)
    return obstacles
