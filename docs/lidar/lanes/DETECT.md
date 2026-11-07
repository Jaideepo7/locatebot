# Lane lidar/detect — obstacle detection

File: `locatebot/lidar/detect.py`, test `tests/lidar/test_detect.py`. Pure functions of a `Scan`; no state, no I/O. The Pi runs these every scan, so keep each well under 5 ms for 1,000 beams (measure it in a test with a generous bound and print the timing).

## API (exact)
```python
@dataclass(frozen=True)
class Obstacle:
    centroid_xy: tuple[float, float]
    nearest_xy: tuple[float, float]
    nearest_range_m: float
    bearing_rad: float          # bearing of nearest point
    width_m: float              # distance between first and last point of the cluster
    n_points: int

def clean_scan(scan: Scan, range_min_m: float = RANGE_MIN_M, range_max_m: float = RANGE_MAX_M,
               min_quality: int = 1, spike_jump_m: float = 0.3) -> Scan
    # out-of-range, low-quality and isolated spikes (a point that differs from BOTH angular neighbours by > spike_jump_m while the neighbours agree with each other within spike_jump_m) become NaN. Same angles, stamp, seq.
def sector_clearance(scan: Scan, n_sectors: int = 8) -> np.ndarray
    # min valid range per sector; sector 0 centred on forward (angle 0), sectors in CCW order; NaN when a sector has no valid return (unknown, NOT free)
def corridor_clearance(scan: Scan, half_width_m: float, max_lookahead_m: float = RANGE_MAX_M) -> float
    # smallest x > 0 among valid points with |y| <= half_width_m and x <= max_lookahead_m; math.inf if none
def forward_coverage(scan: Scan, fov_rad: float = math.pi) -> float
    # fraction of beams with |angle| <= fov_rad/2 that have a valid return; 0.0 if no beams in the FOV
def cluster_scan(scan: Scan, break_distance_m: float = 0.15, break_ratio: float = 0.03, min_points: int = 3) -> list[Obstacle]
    # sort valid points by angle; start a new cluster where the gap between consecutive points > break_distance_m + break_ratio * range; join the last and first cluster across the -pi/+pi seam; drop clusters with < min_points; return sorted by nearest_range_m
```
## Required tests (use `sim.Room` + `simulate_scan`)
- `clean_scan` removes NaN/out-of-range/low-quality and a planted spike but keeps a genuine 3-point edge.
- Sector clearance: box in front shows in sector 0; left wall in sector 2 of 8; empty sector gives NaN.
- Corridor clearance: obstacle inside the corridor is reported, one just outside the width is not, nothing → inf.
- Forward coverage drops when you add dropout to the forward half.
- Clustering: two separate boxes give two obstacles with correct nearest ranges/bearings; a box straddling the ±pi seam is one cluster; walls of an empty room do not produce clusters smaller than min_points.
- Timing test for 1,000 beams.

## Rules for every lane
- Work only in your worktree and only in the files listed for your lane. `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `pyproject.toml` and this README belong to the integrator; if you need a change there, stop and report it.
- Python >= 3.10, numpy only (pyserial allowed lazily inside the RPLIDAR driver). No other dependencies, no network, no hardware access.
- Test first: write the tests, run them and see them fail, implement, then run the **whole** suite: `PYTHONPATH=. /opt/data/projects/locatebot-venv/bin/python -m pytest -q`.
- Use `locatebot.lidar.sim` for synthetic rooms/scans. Synthetic tests prove software behaviour, not real-sensor behaviour; say so in docstrings where it matters.
- Vectorise with numpy; no per-beam Python loops in hot paths unless the brief allows it.
- Commit only your files with a conventional message (`feat(lidar-<lane>): ...`). Do not merge, push, or touch other branches.
