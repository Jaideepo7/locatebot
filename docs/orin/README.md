# Orin runtime: mapping, skills, LLM brain, operator page

**Status (2026-11-08):** built and tested in simulation (closed loop) and against the real RPLIDAR C1 for mapping. Never driven a real car: the motor/steering driver on the Pi does not exist yet.

## Pieces
| Module | What it does |
|---|---|
| `locatebot/orin/live_map.py` | `LiveMapper`: thread-safe live mapping (scan matching + occupancy grid), pose/scan history for frame capture times, `save()` to `.lbmap` |
| `locatebot/orin/world.py` | `build_world_state`: pose, sector clearances, item tracks, skill status -> `WorldState` for the LLM |
| `locatebot/orin/skills.py` | `SkillExecutor`: runs one `SkillCall` per tick on Nav-lite (see below) |
| `locatebot/orin/runtime.py` | `OrinRuntime`: wires mapper, vision results -> localization -> `ItemTracker`, executor and `Brain` |
| `locatebot/orin/app.py` | `OrinApp` control/drive threads, `PeerClock`, operator HTTP page |
| `tools/orin_run.py` | command line: `--sim`, `--lidar-port`, or `--pi-host`; `--llm none/claude/local`; `--vision` |

## Run it
```bash
# Demo, no hardware: simulated room and car. Open http://127.0.0.1:8090/
.venv/bin/python tools/orin_run.py --sim

# Bench: LiDAR plugged into the Orin, live map on the page, drive is a dry run (logged)
.venv/bin/python tools/orin_run.py --lidar-port /dev/ttyUSB0 --host 0.0.0.0

# The car: scans/frames from the Pi, drive commands back, vision on, LLM proposals need approval
.venv/bin/python tools/orin_run.py --pi-host <pi-address> --vision --llm claude --mode approve --host 0.0.0.0
```
`--llm nemotron --llm-model <id>` (the team's choice) uses NVIDIA's OpenAI-compatible API at `https://integrate.api.nvidia.com/v1` with the key from `NVIDIA_API_KEY`; the model id comes from build.nvidia.com and must support tool calling. `--llm claude` reads `ANTHROPIC_API_KEY` from the environment and needs internet (the open question in `docs/PROJECT.md` §8a). `--llm local` talks to an OpenAI-compatible server (`--llm-url`, `--llm-model`). With `--llm none` the operator sends skills from the page.

## Skills (`SkillExecutor`)
- `stop`; `go_to` a point or an item (stops `standoff_m` short); `return_home` (session start pose).
- `explore`: frontier exploration until no reachable frontier is left.
- `search(items)`: explore, then investigate single unconfirmed sightings, then patrol waypoints ~1.5 m apart with a slow full-lock camera sweep at each; succeeds when every item is confirmed, fails listing found/missing when the patrol ends or after `search_timeout_s` (600 s). The LiDAR map is usually complete long before the camera has looked everywhere: that is why the patrol exists.
- `look_around`: one slow full-lock loop if the circle fits in known free space (the car can't turn on the spot).
- `follow(item)`: keeps `follow_distance_m` from the item; vision fixes every few seconds, LiDAR cluster nearest the last position in between; fails after `follow_lost_s` without confirmation.
- Waits for the map to fill in before deciding (a fresh grid has no FREE cells); spaces replans over `replan_interval_s`; explains failures ("in unmapped space: explore first", "no drivable path", "lost marker"); `tick` never raises.

## LLM and the `continue` tool
The brain wakes the LLM on events (an item confirmed, a skill ended, a safety stop). If the running skill is still right the LLM answers `continue`; re-calling the running skill also counts as continue (it is not restarted). Without this, mid-skill wake-ups either restarted the skill or counted as failures and tripped the fallback STOP (found by the end-to-end simulation).

## Timing (measured)
| Per tick | x86 laptop | Orin Nano CPU |
|---|---|---|
| control step p50 / p95 / max | 10 / 20 / 89 ms | 45 / 88 / 221 ms |
| scan integration p50 / max | 8 / 47 ms | 29 / 67 ms |

Scans, the 10 Hz control step and the 20 Hz drive sender run on separate threads. The sender repeats the newest command and sends zero if the control step has produced nothing for 0.5 s; the Pi stops the car if commands stop for 0.3 s. A one-off planning spike (228 ms on the laptop when a search builds its patrol, so roughly 0.6 s on the Orin) therefore makes the car pause, not run blind. If pauses are frequent on the car, move planning to a background thread.

## Clocks
Scans and frames from the Pi carry Pi monotonic stamps. `PeerClock` estimates the Pi-minus-Orin offset (running maximum of `stamp - receive time`, slowly decayed) so the runtime runs on the Pi timeline. Local LiDAR and `--sim` use the Orin clock directly.

## Operator page
`GET /` page (inline, no CDN, CSP blocks external assets), `GET /status`, `GET /map.png`, `POST /skill {name, args}`, `/stop`, `/approve`, `/reject`, `/resume`, `/mode {mode}`, `/save_map {name}`. POSTs must be `application/json`. No login: keep it on the team network. Checked in headless Chromium (form -> search running, no JS errors).

## Tests
`tests/orin/`: live map, world state, 21 closed-loop skill tests (bicycle model, simulated LiDAR building the grid, fake vision with FOV and occlusion, no-collision checks), end-to-end (scripted LLM -> search -> go_to -> stop with scan-matching odometry, the real vision worker and localization: items mapped within 0.1 m), app threads/HTTP and an `orin_run.py --sim` smoke test.

## Still needs the real car
- Motor/steering driver on the Pi; then the Pi must publish scans/frames and accept `drive` over the link (`tools/pi_drive.py` today only does manual drive).
- Tune `VehicleConfig` (wheelbase, steering angle, width, speed at max throttle) and the pursuit settings on the car.
- Check scan-matching odometry while driving (no wheel encoders), and the planning pause rate on the Orin with vision running.
- Camera mount offset relative to the LiDAR (`LocalizeConfig.camera_xy`) and its field of view (`--hfov-deg`).
