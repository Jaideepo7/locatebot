# LiDAR detection and map building

**Status:** foundation committed; four lanes in progress on `lidar/*` branches. Hardware not yet tested: the RPLIDAR C1 driver is written from the published protocol and only tested against synthetic byte streams.

## What this subsystem does
1. **Read** the RPLIDAR C1 (fallbacks: Oradar MS200, RPLIDAR A1M8) and produce one `Scan` per revolution. Record/replay scans to `.lbscan` files so the team can develop without the car.
2. **Detect** obstacles in each scan: cleaned ranges, per-sector clearance, clustered obstacles, forward-corridor clearance and coverage. This feeds the Pi's safety stop and the Orin's planner.
3. **Track pose** without wheel encoders: match each scan against recent keyframes (correlative scan matching) to estimate where the car is.
4. **Build the map**: a log-odds occupancy grid, saved as a binary `.lbmap` file (the "bin file" from each exploration session) and exported as ROS-compatible `PGM + YAML` for viewing.

```text
RPLIDAR --serial--> rplidar.py --Scan--> detect.py ----> obstacles / clearance (safety, planner)
                         |                  |
                     scanlog.py         odometry.py --Pose2D--> grid.py --> mapfile.py --> session.lbmap
                     (.lbscan)          (matching.py)                                   session.pgm/.yaml
```

Where it runs: the LiDAR plugs into the **Pi** so the emergency stop works even if the Orin or the Ethernet link dies. The Pi runs `rplidar.py` + `detect.py` and forwards scans to the **Orin**, which runs `odometry.py`, `grid.py` and `mapfile.py`. All modules are plain Python + numpy and run on either board.

## Conventions (frozen)
- Robot frame: +x forward, +y left, angles counter-clockwise from forward, radians, metres, monotonic seconds.
- `Scan(stamp_s, seq, angles_rad, ranges_m, quality)`; `ranges_m` is NaN for "no return". **NaN means unknown, never free.** Arrays are read-only.
- `Pose2D(x, y, theta)` is the robot pose in the map frame; `compose`, `inverse`, `transform_points`.
- `RANGE_MIN_M = 0.05`, `RANGE_MAX_M = 12.0`.

## Lanes
| Branch | Lane | Files it owns |
|---|---|---|
| `lidar/io` | RPLIDAR driver + scan logs | `locatebot/lidar/rplidar.py`, `locatebot/lidar/scanlog.py`, `tools/record_scan.py`, `tests/lidar/test_rplidar.py`, `tests/lidar/test_scanlog.py` |
| `lidar/detect` | Obstacle detection | `locatebot/lidar/detect.py`, `tests/lidar/test_detect.py` |
| `lidar/grid` | Occupancy grid + map file | `locatebot/lidar/grid.py`, `locatebot/lidar/mapfile.py`, `tests/lidar/test_grid.py`, `tests/lidar/test_mapfile.py` |
| `lidar/match` | Scan matching + odometry | `locatebot/lidar/matching.py`, `locatebot/lidar/odometry.py`, `tests/lidar/test_matching.py`, `tests/lidar/test_odometry.py` |
| `lidar` | Integration | `locatebot/lidar/mapping.py`, `tools/build_map.py`, `tests/lidar/test_mapping_e2e.py`, this README |

The lanes import only `types.py` and `sim.py`, never each other, so they merge without conflicts. Exact APIs are in `docs/lidar/lanes/*.md`.

## Rules for every lane
- Work only in your worktree and only in the files listed for your lane. `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `pyproject.toml` and this README belong to the integrator; if you need a change there, stop and report it.
- Python >= 3.10, numpy only (pyserial allowed lazily inside the RPLIDAR driver). No other dependencies, no network, no hardware access.
- Test first: write the tests, run them and see them fail, implement, then run the **whole** suite: `PYTHONPATH=. /opt/data/projects/locatebot-venv/bin/python -m pytest -q`.
- Use `locatebot.lidar.sim` for synthetic rooms/scans. Synthetic tests prove software behaviour, not real-sensor behaviour; say so in docstrings where it matters.
- Vectorise with numpy; no per-beam Python loops in hot paths unless the brief allows it.
- Commit only your files with a conventional message (`feat(lidar-<lane>): ...`). Do not merge, push, or touch other branches.
