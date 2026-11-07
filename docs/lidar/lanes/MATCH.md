# Lane lidar/match — scan matching and LiDAR odometry

Files: `locatebot/lidar/matching.py`, `locatebot/lidar/odometry.py`, tests `tests/lidar/test_matching.py`, `tests/lidar/test_odometry.py`.

The car kit has no wheel encoders, so the car's pose comes from the LiDAR alone. Implement **correlative scan matching** against a likelihood field (robust, simple, good at the 10 Hz / <=1 m/s regime). Do not build an occupancy grid here and do not import `grid.py` — use your own likelihood raster.

## API (exact)
```python
class LikelihoodField:
    @classmethod
    def from_points(cls, points_xy: np.ndarray, resolution_m: float = 0.05, sigma_m: float = 0.10, margin_m: float = 1.0) -> "LikelihoodField"
        # raster covering the points' bounding box + margin; value = exp(-d^2 / (2 sigma^2)) where d = distance to nearest reference point (compute with a vectorised distance transform or by splatting a Gaussian kernel per point and taking the max)
    def score(self, points_xy: np.ndarray) -> float   # mean field value at the points (0 outside the raster)

@dataclass(frozen=True)
class MatchResult: pose: Pose2D; score: float; converged: bool

def match_scan(field: LikelihoodField, scan_points_xy: np.ndarray, initial: Pose2D,
               search_xy_m: float = 0.3, search_theta_rad: float = 0.35,
               min_score: float = 0.35) -> MatchResult
    # pose of the scan (robot) in the field's frame. Coarse-to-fine grid search over (dx, dy, dtheta) around `initial`, evaluating all candidates of a level with numpy at once, then refine (smaller steps). converged = best score >= min_score and best is not on the search-window edge.

@dataclass(frozen=True)
class OdometryResult: pose: Pose2D; score: float; accepted: bool; is_keyframe: bool

class ScanOdometry:
    def __init__(self, keyframe_dist_m: float = 0.3, keyframe_angle_rad: float = 0.3,
                 submap_keyframes: int = 5, max_points: int = 600, min_score: float = 0.35): ...
    def update(self, scan: Scan) -> OdometryResult
        # first scan: pose (0,0,0), accepted, keyframe. Then: predict with constant velocity from the last two accepted poses; match the (downsampled to <= max_points) scan points against a likelihood field built from the last `submap_keyframes` keyframes' points in the map frame; accept if converged; add a keyframe when moved > keyframe_dist_m or turned > keyframe_angle_rad since the last keyframe. On rejection, keep the predicted pose but report accepted=False and do not add a keyframe.
    @property
    def pose(self) -> Pose2D
```
## Required tests (use `sim.Room` with some interior boxes so the room is not symmetric)
- `match_scan` recovers a known offset (e.g. 0.15 m, -0.1 m, 8°) from a perturbed initial guess within 2 cm / 1°.
- A scan of a completely different room gives converged=False.
- Odometry over a simulated 60-step trajectory (≈0.08 m and up to 4° per step, noise 1 cm, 2% dropout) ends within 0.10 m and 3° of ground truth. Report the final error.
- Prediction: a constant-velocity straight run converges each step.
- Timing: one `update` with 450 beams under 50 ms on a laptop CPU (generous bound in the test; print the timing).

## Rules for every lane
- Work only in your worktree and only in the files listed for your lane. `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `pyproject.toml` and this README belong to the integrator; if you need a change there, stop and report it.
- Python >= 3.10, numpy only (pyserial allowed lazily inside the RPLIDAR driver). No other dependencies, no network, no hardware access.
- Test first: write the tests, run them and see them fail, implement, then run the **whole** suite: `PYTHONPATH=. .venv/bin/python -m pytest -q`.
- Use `locatebot.lidar.sim` for synthetic rooms/scans. Synthetic tests prove software behaviour, not real-sensor behaviour; say so in docstrings where it matters.
- Vectorise with numpy; no per-beam Python loops in hot paths unless the brief allows it.
- Commit only your files with a conventional message (`feat(lidar-<lane>): ...`). Do not merge, push, or touch other branches.
