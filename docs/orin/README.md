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
**Saved map (M1):** `--map maps/<name>.lbmap [--start-pose x,y,deg] [--save-map]` localizes in a saved map instead of starting a new one (no `--start-pose`: global search over the map; `ambiguous` in a symmetric room means: give one). The map frame and `home` (for `return_home`) are the saved map's. The map is updated slowly while localized; on exit it is written to `maps/<name>.v2.lbmap/.pgm/.yaml`, or over `<name>.lbmap` with `--save-map` (nothing is written if it never localized). The status table shows `map: localized (map) score 0.95` / `LOST` / `ambiguous`; `POST /relocalize {"x": .., "y": .., "theta_deg": ..}` (or `{}` for a global search) re-initializes. See `docs/lidar/README.md` (Localization). In this mode the map is layered (M2): new or moving things (seen where the saved room is confidently free) go to a short-lived dynamic layer, the skills plan around them (`grid_snapshot()` is the planning grid), the LLM gets a `changes:` line, and nothing that moves is burned into the room; something that stays put for 2 minutes becomes part of the room. M3 adds, in the same mode: a LiDAR people tracker (`people:` summary line; follow(person) uses its tracks; when no search runs the runtime asks vision for "person" at low priority to confirm a walking track), the ultrasonic `glass_or_low` layer from the Pi's `Range` messages (blocks planning; note `glass or low obstacle: ...`), and the Wi-Fi coverage layer (1 Hz RSSI, `--wifi-iface`, page overlay checkbox, `wifi: weak here` note). See `docs/lidar/README.md` (M3).

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

## Goal (R4)
`OrinRuntime.set_goal(text, source="operator")` / `clear_goal()` hold what the operator (or, from R3, a Telegram customer) wants, e.g. "find the bolt". It is validated (`normalize_goal`), timestamped, thread-safe, recorded as an event (`goal set: ... (by operator)` / `goal cleared`) and put into `WorldState.goal`, which the LLM sees as the first line of its state. A new goal wakes the brain. An operator STOP still pauses the brain: setting a goal does not resume it (`/resume` does).

## Mission bridge (R3, `locatebot/orin/missions.py`)
`MissionBridge(runtime, publish, clock=time.time)` turns Telegram/operator missions from the Pi into skills. `tools/orin_run.py --pi-host` wires it: `pi_source` subscribes `mission_request` / `mission_cancel` (not newest-only) and calls `on_message`; `OrinRuntime.step` calls `bridge.tick(now, ws)` after the brain (outside the runtime lock; never blocks); statuses go into `PiDriveSink.queue` and are sent by the drive-sender thread, which owns the PUB socket.
- Validation (from `JetsonAgent`): expired, more than 5 s in the future, `expires <= issued`, or a second concurrent mission → `FAILED` with `EXPIRED` / `CLOCK_SKEW` / `INVALID_EXPIRY` / `BUSY`; a duplicate id gets its last status again.
- Start: `set_goal("find <target>", source="mission")`; with a brain it acts (approve/auto; a paused brain is resumed); without one the routine runs `search([target])`, then `go_to(item=target)`; a lost target re-searches at most twice.
- Status mapping, cancel and link loss: see the table in `docs/remote/README.md`. Target matching: case-insensitive exact item name, so wording matters (short nouns, `docs/vision/README.md`).
- `runtime.status()["missions"]`: active mission (id, target, state, step, link_down, expires_in_s), `link_up`, counters.

## Timing (measured)
| Per tick | x86 laptop | Orin Nano CPU |
|---|---|---|
| control step p50 / p95 / max | 10 / 20 / 89 ms | 45 / 88 / 221 ms |
| scan integration p50 / max | 8 / 47 ms | 29 / 67 ms |

Scans, the 10 Hz control step and the 20 Hz drive sender run on separate threads. The sender repeats the newest command and sends zero if the control step has produced nothing for 0.5 s; the Pi stops the car if commands stop for 0.3 s. A one-off planning spike (228 ms on the laptop when a search builds its patrol, so roughly 0.6 s on the Orin) therefore makes the car pause, not run blind. If pauses are frequent on the car, move planning to a background thread.

## Never drive on an untrusted pose (saved-map mode)
With `--map`, the pose comes from localizing in the saved map. While the localizer is **lost**, **ambiguous** or **not localized yet**, the pose is frozen at the last good value, so `OrinRuntime.step` outputs zero and doesn't tick skills. **Lost** (after having been localized) also cancels the running skill (`stopped: localization lost ...`), because its plan was built on a pose that is now wrong; **not localized yet / ambiguous** only holds, so a skill sent a moment early starts once the pose is trusted. Either way it adds a `localization ...; holding still` note for the LLM and logs one event when it starts and `localization recovered` when it ends. Recover with `/relocalize` (operator pose or `{}` for a global search). New-map (odometry) mode is unaffected. Tests: `tests/orin/test_pose_guard.py`.

## Clocks
Scans and frames from the Pi carry Pi monotonic stamps. `PeerClock` estimates the Pi-minus-Orin offset (running maximum of `stamp - receive time`, slowly decayed) so the runtime runs on the Pi timeline. Local LiDAR and `--sim` use the Orin clock directly.

## Operator page
`GET /` page (inline, no CDN, CSP blocks external assets), `GET /status`, `GET /map.png`, `POST /skill {name, args}`, `/stop`, `/approve`, `/reject`, `/resume`, `/mode {mode}`, `/save_map {name}`, `/goal {text}` (1–200 chars after whitespace collapse, no control characters, else 400), `/goal/clear`, `/relocalize {x, y, theta_deg}` (saved-map mode only; `{}` = global search). The page has a goal box with "Set goal" / "Clear"; `/status` has `goal: {text, source, set_s}` or `null`. POSTs must be `application/json`. No login: keep it on the team network. Checked in headless Chromium (form -> search running, no JS errors).

## Tests
`tests/orin/`: live map, world state, 21 closed-loop skill tests (bicycle model, simulated LiDAR building the grid, fake vision with FOV and occlusion, no-collision checks), end-to-end (scripted LLM -> search -> go_to -> stop with scan-matching odometry, the real vision worker and localization: items mapped within 0.1 m), app threads/HTTP and an `orin_run.py --sim` smoke test.

## Still needs the real car
- Motor/steering driver on the Pi; then the Pi must publish scans/frames and accept `drive` over the link (`tools/pi_drive.py` today only does manual drive).
- Tune `VehicleConfig` (wheelbase, steering angle, width, speed at max throttle) and the pursuit settings on the car.
- Check scan-matching odometry while driving (no wheel encoders), and the planning pause rate on the Orin with vision running.
- Camera mount offset relative to the LiDAR (`LocalizeConfig.camera_xy`) and its field of view (`--hfov-deg`).
