# Lane lidar/grid — occupancy grid and the map ("bin") file

Files: `locatebot/lidar/grid.py`, `locatebot/lidar/mapfile.py`, tests `tests/lidar/test_grid.py`, `tests/lidar/test_mapfile.py`.

## Grid API (exact)
```python
FREE, OCCUPIED, UNKNOWN = 0, 100, 255

class OccupancyGrid:
    def __init__(self, width_cells: int, height_cells: int, resolution_m: float = 0.05,
                 origin_xy: tuple[float, float] = (0.0, 0.0),
                 l_occ: float = 0.85, l_free: float = -0.4, l_min: float = -4.0, l_max: float = 4.0): ...
    @classmethod
    def around(cls, center_xy: tuple[float, float], size_m: float, resolution_m: float = 0.05) -> "OccupancyGrid"
    log_odds: np.ndarray            # float32, shape (height, width); row index = y, column = x; origin_xy is the world position of cell (0, 0)'s lower-left corner
    def world_to_cell(self, xy: np.ndarray) -> np.ndarray        # (N, 2) int64 (col, row); floor division
    def cell_to_world(self, cr: np.ndarray) -> np.ndarray        # (N, 2) cell centres
    def in_bounds(self, cr: np.ndarray) -> np.ndarray
    def integrate(self, scan: Scan, pose: Pose2D, max_range_m: float = RANGE_MAX_M) -> None
    def probability(self) -> np.ndarray                          # 1 / (1 + exp(-log_odds))
    def classify(self, occ_threshold: float = 0.65, free_threshold: float = 0.35) -> np.ndarray  # uint8 FREE/OCCUPIED/UNKNOWN
```
`integrate`: for each valid return, cells along the ray from the robot cell up to (not including) the endpoint get `l_free`, the endpoint cell gets `l_occ`; clamp to [l_min, l_max]. Each cell is updated at most once per scan per kind (dedupe with numpy, e.g. `np.unique` on flattened indices) so dense beams don't over-count. NaN beams update nothing (unknown stays unknown). Endpoints beyond `max_range_m` are skipped. Cells outside the grid are ignored, no exceptions. Ray traversal must be vectorised (e.g. sample each ray at resolution/2 steps with numpy, then dedupe) and fast enough to integrate a 450-beam scan into a 400x400 grid in under 20 ms on a laptop CPU; add a timing test with a generous bound and print the timing.

## Map file `.lbmap` (exact)
```
magic   8 bytes  b"LBMAP1\0\0"
header  struct "<HdddIII": version=1, resolution_m, origin_x, origin_y, width, height, meta_len
meta    meta_len bytes of UTF-8 JSON (session info, robot name, created time, anything)
cells   width*height float32 log-odds, row-major (row 0 = lowest y)
crc     u32 CRC32 of everything before it
```
```python
class MapFileError(ValueError): ...
def save_map(path, grid: OccupancyGrid, meta: dict | None = None) -> None   # atomic: write temp file in same dir then os.replace
def load_map(path) -> tuple[OccupancyGrid, dict]                          # MapFileError on bad magic/version/size/CRC
def export_ros_map(grid: OccupancyGrid, pgm_path, yaml_path, occ_threshold=0.65, free_threshold=0.196) -> None
    # PGM P5, 8-bit; image row 0 = HIGHEST y (flip rows); free 254, occupied 0, unknown 205.
    # YAML keys: image (pgm file name, relative), resolution, origin [x, y, 0.0], negate 0, occupied_thresh, free_thresh
```
## Required tests
- world/cell round trip incl. negative coordinates and floor behaviour; in_bounds.
- One scan from the centre of `sim.Room.box(4, 3)` marks walls OCCUPIED and the interior FREE; cells behind walls stay UNKNOWN; NaN beams change nothing; repeated integration saturates at l_max/l_min.
- Out-of-grid endpoints don't raise.
- `.lbmap` round trip is exact; corrupted byte → MapFileError; truncated file → MapFileError; meta round trip.
- PGM header/size/values and row flip; YAML fields.
- Timing test (450 beams, 400x400).

## Rules for every lane
- Work only in your worktree and only in the files listed for your lane. `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `pyproject.toml` and this README belong to the integrator; if you need a change there, stop and report it.
- Python >= 3.10, numpy only (pyserial allowed lazily inside the RPLIDAR driver). No other dependencies, no network, no hardware access.
- Test first: write the tests, run them and see them fail, implement, then run the **whole** suite: `PYTHONPATH=. .venv/bin/python -m pytest -q`.
- Use `locatebot.lidar.sim` for synthetic rooms/scans. Synthetic tests prove software behaviour, not real-sensor behaviour; say so in docstrings where it matters.
- Vectorise with numpy; no per-beam Python loops in hot paths unless the brief allows it.
- Commit only your files with a conventional message (`feat(lidar-<lane>): ...`). Do not merge, push, or touch other branches.
