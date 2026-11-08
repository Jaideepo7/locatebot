# LiDAR detection and map building

**Status:** all four lanes merged into `lidar-integration`, reviewed, and review fixes merged; 121 tests pass. **Not yet tested on hardware:** the RPLIDAR C1 driver is written from the published protocol and only tested against synthetic byte streams; matching/mapping accuracy is measured on simulated rooms only.

## Quick start
```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[hardware,dev]'
python -m pytest -q                                   # 121 tests, no hardware needed

# Synthetic demo map (no hardware): writes maps/demo.lbmap, .pgm, .yaml
python tools/build_map.py --demo --out maps/demo --size 16

# On the Pi with the RPLIDAR C1 plugged in (C1 default 460800 baud; A1M8 uses 115200):
python tools/record_scan.py --port /dev/ttyUSB0 --out recordings/room1.lbscan --seconds 60
# Then build the session map (on the Orin or a laptop):
python tools/build_map.py --log recordings/room1.lbscan --out maps/room1 --size 20
```
The `.lbmap` file is the session map ("bin file"): log-odds grid + JSON metadata (trajectory, scan counts) + CRC. Open `maps/<name>.pgm` in any image viewer, or load `<name>.yaml` in ROS tools.

## First hardware session checklist
1. `record_scan.py` prints health/info and scans/second; expect about 10 scans/s on the C1.
2. Put a box straight in front of the sensor; confirm it shows up at angle 0 (forward). If it appears elsewhere, set `--mount-offset-deg`.
3. Measure a known distance with a tape and compare.
4. Record a slow walk around the room carrying the car (no motors), then build the map and check it looks like the room.
5. Only after that, record while driving slowly.

## Known limitations
- No loop closure: drift grows on long runs; keep exploration loops short.
- Motion during one revolution (scan skew) is not corrected; drive slowly while mapping.
- `corridor_clearance` returns inf when nothing is seen; the safety stop must also require high `forward_coverage`.
- A 2D scan misses obstacles above or below the sensor plane, glass and drop-offs.

## Localization in a saved map (M1)
`locatebot/lidar/localize.py`. A session can start from a saved `.lbmap` instead of a blank map: `MapLocalizer` tracks the pose in the saved map's frame, `LocalizingMapper` (the `Mapper` counterpart) updates the map slowly so it keeps improving across sessions ("semi-constant"). On the Orin: `LiveMapper.from_saved(path, guess)` / `tools/orin_run.py --map`. Offline check: `tools/relocalize.py`.

**Method.**
- Reference points: one per OCCUPIED cell (p > 0.65), at the log-odds-weighted centroid of the occupied cells in its 3x3 neighbourhood. A wall on a cell boundary is two cells thick; plain cell centres left a flat likelihood across both cells and a ~3 cm pose bias on the synthetic room (now ~3 mm). The likelihood field (same as odometry, sigma 0.10 m) is built once and rebuilt only when >= 30 cells changed occupied/not (checked every 25 integrated scans).
- `initialize(scan, guess, search_xy_m, search_theta_rad)`: every candidate pose (0.2 m grid x 5 deg headings) inside the window around the guess (default +-1 m, +-45 deg), or with `guess=None` over every FREE map cell and all 72 headings, is scored on the coarse field with 100 scan points; the best 8 distinct candidates are refined with `match_scan` (a refinement pinned to its window edge is followed up to twice; one that never settles is dropped). If the runner-up is a *different* pose (> 0.3 m or > 10 deg away) scoring >= 0.95 x the best, the result is `ambiguous` and nothing is set: the operator gives a start pose (`--start-pose`, `POST /relocalize`). Global init takes ~50-100 ms on the laptop.
- `update(scan)`: constant-velocity prediction + `match_scan` in a +-0.25 m / +-0.3 rad window (converged = score >= 0.35 and not pinned to the edge). If the map match fails (mostly new obstacles, people), it falls back to matching against the last 3 map-matched scans (`source == "odometry"`), for at most 10 updates in a row.
- **Lost** after 5 updates with neither map nor odometry, after 10 without the map, or after two map scores in a row below 0.12. A lost localizer returns `lost=True`, holds the last good pose (never a silent jump or drift), and every 10 scans tries a **global** re-initialization; it accepts only an unambiguous solution scoring >= 0.85 x the pre-loss score.
- `integrate_slow` (`OccupancyGrid.integrate_slow`, same ray model as `integrate`): cells with |log-odds| >= 1.5 move by `rate` x the normal increment (default 0.2), unknown/weak cells by the full increment; only for scans localized by a converged, unambiguous *map* match. `integrate` itself is bit-identical to before (golden test). A new box in free space becomes OCCUPIED after ~25 scans (2.5 s at 10 Hz) of continuous view; M2 keeps moving things out of the static map.
- `LocalizeResult(pose, score, converged, ambiguous, lost, source, runner_up, runner_up_score)`; `accepted` = converged and not lost/ambiguous. `source`: `init`, `map`, `odometry`, `prediction` (nothing matched), `none` (waiting for the next init attempt).

**Parameters** (`LocalizerConfig`): `sigma_m` 0.10, `min_score` 0.35, `track_xy_m` 0.25, `track_theta_rad` 0.30, `init_xy_m` 1.0, `init_theta_rad` 45 deg, `global_xy_step_m` 0.2, `global_headings` 72 (the brief suggested 12-24; 5 deg steps keep the coarse score informative at 3 m range and cost only a few ms), `ambiguity_ratio` 0.95, `lost_after` 5, `max_odometry_updates` 10, `lost_score_floor` 0.12, `reinit_every` 10, `recover_global` True, `recover_score_ratio` 0.85. `LocalizingMapper(slow_rate=0.2)`.

**Measured.** Synthetic (`tests/lidar/test_localize.py`): restart 0.5 m / -0.3 m / 25 deg from the dock guess converges on the first scan (error ~0.003-0.015 m depending on how walls fall on cells) and tracks a new 100-scan loop with < 0.1 m error; global init finds the pose in `demo_room`, `Room.box(6, 6)` reports ambiguous (runner-up 0.986 vs 0.986); teleport -> lost within 5 updates, recovered at the right pose by the global search; update ~5-6 ms per 508-point scan on the x86 laptop. Real C1 recordings (static sensor among desk clutter, 280 scans each), `tools/relocalize.py`:

| map | log | start | result |
|---|---|---|---|
| room_0157 | room_0157 | home / global | localized at scan 0, score 0.958 mean, pose std 1.6 mm / 0.14 deg, update p50 5-6 ms |
| room_0205 | room_0205 | home / global | localized at scan 0, score 0.970, std 1.0 mm / 0.26 deg |
| room_0157 | room_0205 | home | ambiguous at first (0.827 vs 0.789, 19 deg apart), localized at scan 40 (score 0.82 -> 0.95 as the map adapts) |
| room_0205 | room_0157 | home | localized (0.56 -> 0.93 as the map adapts, 3 field rebuilds) |
| either | the other | global | ambiguous (the two sessions saw a changed scene; never a confident wrong answer) |

These recordings test static stability only (the LiDAR did not move); tracking while driving needs a real drive.

**Failure modes and recovery.** Symmetric rooms and long featureless corridors -> `ambiguous`: give a start pose. A room that changed a lot since mapping -> low scores, odometry fallback, possibly lost; re-map or let the slow update adapt (it only integrates converged map matches). Glass, mirrors and things below/above the scan plane are invisible as in mapping. Lost: the status shows `LOST`; drive slowly back to a mapped area or `POST /relocalize {"x", "y", "theta_deg"}` with the true pose (or `{}` for a global search). Thresholds to tune on the car: `min_score` and `ambiguity_ratio` against real scores, `slow_rate` against how fast furniture should be learnt.

## Static vs dynamic layers and change detection (M2)
`locatebot/lidar/layers.py`, on by default in saved-map mode (`LiveMapper.from_saved(..., layered=True)`, `LocalizingMapper(layered=True)`, `tools/relocalize.py --layered`).

**Layers.** `static` = the room (M1's slowly updated log-odds grid). `dynamic` = short-lived positive evidence for new/moving things: +1.5 log-odds per NEW endpoint (max 4), -1.0 per ray passing through, and a half-life of 3 s toward unknown; never saved. `extra` = named layers other modules register (`register_layer(name, fill, dtype, scale, offset, params)`); `params["half_life_s"]` makes it decay, `params["blocks_above"]` makes it a planning obstacle; they are saved in `.lbmap` v2.

**Change detection** (`classify_points(scan, pose)`), per valid beam endpoint: `ROOM` if a static OCCUPIED cell (p > 0.65) is within one cell; `NEW` if the endpoint cell and its 8 neighbours are confidently FREE (log-odds < -1.5) **and** >= 80 % of the ray samples 0.1-1.0 m before it are confidently free; else `UNKNOWN`. NEW points are clustered with `detect.cluster_scan` (break 0.15 m, >= 2 points) and tracked as `ChangeBlob(id, centroid_xy, width_m, n_points, first_seen_s, last_seen_s, persistence, moving, promoted)`: nearest-neighbour association within 0.5 m, `moving` = moved > 0.1 m in the last 1 s; dropped when its spot was seen free in 3 scans or after 300 s unseen.

**Update rules** (`update(scan, pose, stamp)`): NEW endpoints only into `dynamic`. `static` gets the M1 slow update from every ray, but NEW endpoints add no occupancy and their cells are not freed; rays stop at the person, so a person in a doorway neither adds a wall nor frees the space behind them. A blob that stays within 0.15 m for `promote_after_s` (120 s) is promoted: from then on its points are ordinary slow hits (a moved chair becomes room after ~2.5 s more), and it is no longer reported. A static obstacle that is gone fades by the normal slow update. With an odometry-fallback pose (M1) blobs and dynamic are updated but static is not touched.

**Planning.** `planning_grid()` = static, with dynamic cells >= 0.62 log-odds and blocking extra layers set occupied (cell-wise max). `LiveMapper.grid_snapshot()` returns it in layered mode, so `nav.build_costmap` and the skills plan around new things without API changes; the map image shows it too.

**World state.** `LiveMapper.change_blobs()` (blobs seen in >= 3 scans) -> `build_world_state(changes=...)` -> `WorldState.changes` (`ChangeView`, nearest first, max 5, promoted ones left out) -> summary line `changes: 2 new things: 0.4 m wide at 1.2 m ahead (moving), ...` (absent when there are none).

**Parameters** (`LayersConfig`): `free_lo` -1.5, `ray_check_m` 1.0, `ray_free_frac` 0.8, `dynamic_half_life_s` 3.0, `dynamic_hit` 1.5, `dynamic_clear` 1.0, `dynamic_occ` 0.62, `cluster_break_m` 0.15, `cluster_min_points` 2, `blob_match_m` 0.5, `blob_free_views` 3, `blob_max_unseen_s` 300, `moving_m` 0.1 per `moving_window_s` 1.0, `still_m` 0.15, `promote_after_s` 120, `slow_rate` 0.2.

**Measured.** Classification 0.2 ms, full layer update ~2.3 ms per 508-point scan on the x86 laptop (10 m and 20 m grids). Synthetic tests (`tests/lidar/test_layers.py`): a new box -> NEW, in dynamic, static cells bit-identical after 50 scans; a box moving at 0.4 m/s keeps one track id and leaves no trail; still box promoted (5 s in the test); a person 1 m in front of a wall leaves the wall and the space behind untouched; the planning grid blocks a new box and clears ~5 s after it leaves. Real C1 replays (`--layered`, static sensor): room_0157 on its own map saw 8 blobs, all 2-3 points and 1-14 cm wide at one spot 0.9 m away (an intermittent thin return); 7 of them lived 1-3 scans (hidden by the 3-scan filter), one 9 s. room_0205 on its own map: 1 blob (2 scans). Cross-session replays: 0-9 one-scan blobs.

**Known failure modes.** Glass and mirrors (no return, or a return from behind: may read as NEW or as ROOM); thin chair legs (1-2 beams, below `cluster_min_points` at range); things within one cell of a wall are ROOM; space the static map never saw is UNKNOWN, not NEW; a blob that leaves the field of view is kept (up to 300 s) because nothing proves it left; promotion needs 120 s of continuous detections; localization error > ~5 cm turns wall hits into NEW points (watch the score).

### `.lbmap` v2 (`LBMAP2`)
`save_map(path, grid, meta, layers={name: MapLayer | array})` writes v2; without `layers` it writes v1 byte-for-byte as before (golden test). `load_map` reads both (v2: the static layer); `load_map_layers` also returns the extra layers. PGM/YAML export shows the static layer. Layout (little-endian):
```text
magic   b"LBMAP2\0\0"
header  "<HdddIIIH": version=2, resolution_m, origin_x, origin_y, width, height, meta_len, n_layers
meta    JSON
table   per layer: "<H" name_len + UTF-8 name, "<Bdd" dtype code/scale/offset, "<I" params_len + JSON params
blobs   per layer, width*height cells of its dtype, row-major (row 0 = lowest y)
crc     u32 CRC32 of everything before it
```
dtype codes 1 float32, 2 float16, 3 uint8, 4 int16, 5 uint16; value = stored x scale + offset. Layer 0 is always `static` (float32 log-odds). Saved-map sessions write v2 only when an extra layer exists (M3 adds `glass_or_low`, `wifi_rssi`).

## People, glass/low obstacles and Wi-Fi coverage (M3)
All three need saved-map (layered) mode; in new-map mode they are inert.

**People tracker** (`locatebot/lidar/people.py`, `PeopleTracker`). Input: each scan's NEW-point clusters (`LayeredMap.last_clusters`, map frame; promoted blobs left out). Candidates: a **leg pair** (two clusters 0.05-0.25 m wide, centres 0.1-0.5 m apart -> midpoint), a **body** (one cluster 0.25-0.6 m: legs merged by the clusterer, a person close by, or a LiDAR at hip height), a **single leg** (updates a track, never starts one). Geometry assumes the LiDAR at ~20-30 cm (shins); for a higher mount raise `body_width_m`. The brief's 0.08 m lower leg width became 0.05 m: the visible width of a 10 cm leg is ~7 cm at 2 m. Tracking: constant-velocity Kalman per track (x, y, vx, vy; acceleration noise 1.5 m/s^2; measurement sigma 0.06 m pair / 0.08 m body / 0.15 m single leg), greedy gated nearest neighbour (Mahalanobis^2 < 9.21 and < 1.0 m). Birth after 3 detections (tentative tracks die after 0.5 s), death after `max_missed_s` 2.0 s, `moving` above 0.15 m/s. A confirmed track that walked (> 0.4 m from its birth point, or moving for 1 s) is `likely_person`; one that never moved is an object (a chair) unless the camera confirms it. `wants_camera_check()` offers a likely person not confirmed or asked about in the last 10 s; the runtime then sets the vision items to `["person"]` for 5 s **only when no search/follow/go_to runs**; a vision "person" fix (range found within +-5 deg of the box centre, since it often falls between the legs) within 0.7 m of a track sets `confirmed_by_camera`. Those fixes are not added to the item tracker unless the user asked for "person". `WorldState.people` (`PersonView`, people only, nearest first, max 5) via `build_world_state(people=...)`. `follow(person|people)` uses the nearest person track within 1.0 m of the last target between vision fixes (or the nearest person before any fix) instead of raw clusters.

Measured (synthetic, `tests/lidar/test_people.py`): a two-leg walker with a gait at 0.3/0.6/1.0 m/s -> one track, mean position error 0.05 m (max 0.07 m: the LiDAR sees the near side of the legs, so the track sits ~half a leg towards the robot), speed 0.30/0.60/1.00 m/s; a 4-leg chair -> a change but not a person; two people crossing at 90 deg with the second 0.8 s ahead (closest ~0.4 m) keep their ids; layers + tracker ~2.6 ms per scan on the laptop. Not covered: two people arriving at a crossing together, side-by-side walkers < 0.3 m apart (one track), long occlusions (> 2 s: a new track).

**Ultrasonic `glass_or_low` layer** (`locatebot/lidar/ultrasonic.py`, `UltrasonicLayer`). The Orin gets `link.messages.Range` (topic `range`) from the Pi; for each reading the pose and scan nearest its Pi stamp are used. If no LiDAR point in the sensor cone is nearer than `r + 0.2 m`, the cone's arc at `r` gets +0.35 (cap 1.0) in `glass_or_low`; if the LiDAR sees something there nothing is added; `range_m=None` (no echo) adds nothing; readings beyond 2.0 m are ignored. The layer is never touched by LiDAR free rays, decays with a 10 min half-life, and blocks planning at >= 0.6 (two consistent readings) through `planning_grid()`. The layer exists only after the first `Range` (no messages: no layer, nothing changes). World-state note while a mark is < 3 s old: `glass or low obstacle: 0.6 m ahead (ultrasonic only)`. Parameters (`UltrasonicConfig`): `mount_xy`, `mount_yaw_rad` (measure on the car), `lidar_margin_m` 0.2, `max_range_m` 2.0, `weight` 0.35, `blocks_above` 0.6, `half_life_s` 600.

**Wi-Fi `wifi_rssi` layer** (`locatebot/orin/wifi.py`). `WifiReader` picks `/proc/net/wireless` (if it lists an interface) else `iw dev <iface> link`, at runtime; neither -> disabled, never an error. `orin_run.py --map` polls it at 1 Hz (`--wifi-iface` to choose) and `LiveMapper.add_wifi` writes a running mean per cell (`wifi_rssi`, NaN = no data; counts in `wifi_rssi_n`) at the localized pose. It never affects planning or occupancy. Saved in `.lbmap` v2. Operator page: "Wi-Fi overlay" checkbox (`/map.png?wifi=1`, red -90 dBm to green -40 dBm). Note when the link is weak (< -75 dBm): `wifi: weak here (-80 dBm); strongest nearby at (x, y) (-55 dBm)` (strongest cell within 3 m).

**Failure modes.** Chair legs (rejected only because they never move; a pushed chair walks), pets (small and fast; a walking cat is a likely person until the camera says otherwise), sitting/standing people seen first while still (objects until camera-confirmed), people within one cell of a wall (ROOM points), glass (the LiDAR cannot see it: only the ultrasonic layer helps, and only straight ahead), ultrasonic floor echoes and soft surfaces, the Wi-Fi layer only measures the Orin's own link.

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
| `lidar-integration` | Integration | `locatebot/lidar/mapping.py`, `tools/build_map.py`, `tests/lidar/test_mapping_e2e.py`, this README |

The lanes import only `types.py` and `sim.py`, never each other, so they merge without conflicts. Exact APIs are in `docs/lidar/lanes/*.md`.

## Rules for every lane
- Work only in your worktree and only in the files listed for your lane. `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `pyproject.toml` and this README belong to the integrator; if you need a change there, stop and report it.
- Python >= 3.10, numpy only (pyserial allowed lazily inside the RPLIDAR driver). No other dependencies, no network, no hardware access.
- Test first: write the tests, run them and see them fail, implement, then run the **whole** suite: `PYTHONPATH=. .venv/bin/python -m pytest -q`.
- Use `locatebot.lidar.sim` for synthetic rooms/scans. Synthetic tests prove software behaviour, not real-sensor behaviour; say so in docstrings where it matters.
- Vectorise with numpy; no per-beam Python loops in hot paths unless the brief allows it.
- Commit only your files with a conventional message (`feat(lidar-<lane>): ...`). Do not merge, push, or touch other branches.
