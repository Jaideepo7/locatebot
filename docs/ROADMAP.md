# locatebot roadmap: what comes next, in order

Written 2026-11-08 by the integrator session. Read `docs/PROJECT.md` and `AGENTS.md` first. Every task below is self-contained so it can be handed to one agent; each says what it depends on and what it can run in parallel with.

## How to read this
- **Gates (G0 → G6)** are strictly in order: a gate's tasks need the previous gate *passed*, because they need its hardware or behaviour to exist.
- **Inside a gate, tasks with no dependency on each other run in parallel** (one agent, one worktree under `.worktrees/`, one branch each; see `AGENTS.md`).
- **Software-only tasks are marked [SW]**: they can start *now* with fakes and simulation, before their gate's hardware exists. Tasks marked **[HW]** need the real car or a person's hands.
- Shared contracts (`locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `locatebot/mission/types.py`, `locatebot/link/messages.py`, `pyproject.toml`, `docs/PROJECT.md`) are changed only by the integrator, in **step C** below, *before* parallel agents start.

```text
C  contracts ─┬─ G0 hardware facts ── G1 Pi drives safely ── G2 Orin drives the car ── G5 LLM live ── G6 event-ready
              ├─ G3 storage (SpacetimeDB on the Pi) ── G4 dashboard ───────────────────┘
              ├─ R  remote control: Photon/Telegram on the Pi ──────────────────────────┘
              └─ M  smart map (long-term map, people, ultrasonic layer) ── validated on the car after G2
```

## What is done (2026-11-08)
LiDAR driver (verified on the real C1), mapping + `.lbmap`, scan matching, safety supervisor (LiDAR only), Pi control loop + manual-drive page (fake motor), ZeroMQ link (loopback only), vision on the Orin GPU (~2.1 s/item at 640 px), item localization, Nav-lite, skill executor, LLM brain with `continue` + fallback, Orin runtime + operator page + live radar. 540 tests pass on the laptop and the Orin. Nothing has driven a real car.

## Open decisions (owner: the team, not an agent)
| # | Decision | Blocks |
|---|---|---|
| D1 | Motor/steering hardware: ESC + steering servo, H-bridge + servo, or a motor HAT? Which Pi pins? | T1.1 |
| D2 | Where the LiDAR lives on the car: **Pi** (design: safety stop next to the motors) or Orin (current bench setup). Collision detection on the Pi needs the LiDAR on the Pi. Recommended: Pi. | T1.2, T1.3 |
| D3 | Pi 4 RAM size (2/4/8 GB) and OS (must be 64-bit for SpacetimeDB) | T3.1 placement |
| D4 | **Decided 2026-11-08: Nemotron through NVIDIA's API** (OpenAI-compatible `https://integrate.api.nvidia.com/v1`, key in `NVIDIA_API_KEY`): `tools/orin_run.py --llm nemotron --llm-model <id>`. Still open: which model id (must support tool calling), and internet at the event (D5). The LLM chooses skills; it never sends throttle/steering. | G5 |
| D5 | Telegram remote control at the event needs internet; is a hotspot allowed? | R tasks go live |
| D6 | Camera model and mount (USB webcam or Pi camera module) | T1.4, T2.4 |

---

## Step C — contracts first (integrator, ~1 h, before parallel work)
**C1. `locatebot/link/messages.py`**: add
- `Range` (Pi → Orin): `stamp_s`, `sensor` ("front_ultrasonic"), `range_m` (null = no echo), `fov_rad`. Topic `range`.
- `MissionRequest` (Pi → Orin): `mission_id`, `target` (normalised text), `issued_s`, `expires_s`. Topic `mission_request`.
- `MissionCancel` (Pi → Orin): `mission_id`, `reason`. Topic `mission_cancel`.
- `MissionStatus` (Orin → Pi): `mission_id`, `state` (PENDING, ACCEPTED, SEARCHING, TARGET_DETECTED, APPROACHING, COMPLETED, FAILED, CANCELLED), `message`, `reason`. Topic `mission_status`.
- `OrinStatus.boot_id` (random per process start) so the Pi knows when to reset drive `seq` (transport lane proposal).
**C2. `locatebot/safety/supervisor.py`**: add `Reason.FAULT` (Pi lane proposal) and `Reason.ULTRASONIC` (see T1.2).
**C3. `locatebot/mission/types.py`**: add `WorldState.goal: str = ""` (the operator's request in words) and `WorldState.people: tuple[PersonView, ...] = ()` (`PersonView`: x, y, range_m, bearing_rad, speed_mps, confirmed_by_camera). Make `SKILL_ARGS[SEARCH]["items"]` a `tuple` (agent lane proposal: keeps `SkillCall` hashable).
**C4. `pyproject.toml`** extras: `pi = ["pigpio>=1.78"]` (or `lgpio`), `remote = ["websockets>=12"]`, `storage = []` (bridge uses stdlib HTTP).
**Acceptance:** whole suite passes; encode/decode round-trip tests for every new message; README tables in `docs/link/README.md` updated.

---

## G0 — hardware facts (people, not agents) [HW]
Must be done before G1 hardware tasks. Write every answer into `docs/hardware/README.md` (new).
1. **Motor/steering (D1):** open the car; photograph the ESC/driver and servo; note voltages, connectors, whether the ESC needs arming, whether it has reverse/brake, steering servo pulse range. Measure **wheelbase**, **track width**, **max steering angle each way**.
2. **Pi 4:** RAM size, flash 64-bit Raspberry Pi OS (Bookworm) or Ubuntu 24.04 arm64, Python ≥ 3.10, clone repo, `pip install -e '.[hardware,link,dev]'`, run the suite (expect all pass).
3. **Network:** Pi ↔ Orin Ethernet with fixed IPs on a private subnet; team router/hotspot for laptop + phones; note addresses in a git-ignored `.env`, never in the repo.
4. **Mounts:** LiDAR at 20–30 cm height, level, clear 360° (D2); camera facing forward (D6); HC-SR04 front centre, 5–10° downward tilt; physical **kill switch** in the motor battery line.
5. **HC-SR04 wiring:** VCC 5 V, GND, TRIG to a GPIO, **ECHO through a divider (1 kΩ / 2 kΩ) to a 3.3 V GPIO** (5 V on a Pi pin can damage it).
6. **Power:** Orin from the barrel jack (USB-C cannot power it); Pi from a 5 V 3 A supply/bank; motor from its own battery.
**Gate G0 passed when:** `docs/hardware/README.md` answers D1, D2, D3, D6 with measurements and photos.

---

## G1 — the Pi drives the car safely
### T1.1 Real motor + steering driver [HW, needs D1]
- **Goal:** `locatebot/pi/motor_pwm.py` implementing `MotorDriver` (`set(throttle, steering)`, `stop()`, `close()`) for the hardware in D1.
- **Context:** `locatebot/pi/motor.py` (protocol + `FakeMotor`), `docs/pi/README.md`, `locatebot/pi/control.py` (only caller).
- **Spec:** hardware PWM via pigpio (Pi 4: GPIO 12/13/18/19 support hardware PWM) or lgpio; 50 Hz servo-style pulses for ESC + servo (typ. 1000–2000 µs, neutral 1500 µs, *from D1 measurements*); clamp to [-1, 1]; per-car calibration file `config/car.yaml` (neutral, max forward, steering left/right/centre, deadband, reverse disabled); `stop()` = neutral throttle + centred steering; ESC arming sequence if needed; `close()` leaves neutral. Never raise from `stop()`.
- **Tests:** fake pigpio backend recording pulse widths: mapping, clamping, deadband, neutral on stop/close/exception.
- **Bench (wheels off the ground, kill switch in hand):** full sweep; release = neutral within one tick; unplug Ethernet/kill the process → stops (ESC failsafe on signal loss must be configured: document how).
- **Done when:** `tools/pi_drive.py --motor pwm` drives the real wheels from the page; bench checklist results in `docs/pi/README.md`.

### T1.2 Collision detection: LiDAR + HC-SR04 fused on the Pi [SW now, HW later; needs C1, C2]
- **Goal:** the Pi safety supervisor stops the car when *either* sensor sees an obstacle in the path, and fails closed when a sensor is missing.
- **Context:** `locatebot/safety/supervisor.py` + `docs/safety/README.md` (LiDAR policy, reasons, corridor, coverage), `locatebot/pi/control.py` (`tick` calls `evaluate`), `docs/lidar/README.md` (NaN = unknown, never free).
- **Driver** `locatebot/pi/ultrasonic.py`: trigger 10 µs pulse, time the echo with pigpio callbacks (not busy-wait), 40–60 ms cycle, timeout = no echo (`None`), median of last 3, speed of sound with optional temperature, valid 0.03–4.0 m; background thread; `latest() -> (stamp_s, range_m | None)`. Fake backend for tests.
- **Fusion** (new `SafetyConfig` fields, all placeholders until measured): `us_stop_m` (default 0.35), `us_slow_m` (0.8), `us_timeout_s` (0.3), `us_required` (True once the sensor is fitted).
  - Ultrasonic reading ≤ `us_stop_m` while moving forward → stop, `Reason.ULTRASONIC`.
  - Between stop and slow → throttle cap, same linear ramp as the LiDAR slow zone; the more restrictive of LiDAR and ultrasonic wins.
  - Ultrasonic stale or missing while `us_required` → cap speed to a crawl (`us_fallback_speed`, e.g. 0.15) and report it; don't stop outright (LiDAR still guards), document why.
  - Disagreement (ultrasonic close, LiDAR clear) is *expected* for glass and low objects: ultrasonic wins, log it as `glass_or_low_obstacle` for the map layer (M3).
  - Order of checks stays fail-closed: e-stop → clock → scan → command → validity → reverse → coverage → LiDAR obstacle → ultrasonic → slow-down.
- **Link:** publish `Range` messages to the Orin (C1) at 10 Hz for the map layer and the LLM.
- **Tests:** synthetic scan + fake ultrasonic matrix (glass ahead: LiDAR clear/ultrasonic 0.3 m → stop; both clear → OK; ultrasonic stale → crawl; LiDAR stale → stop regardless; steering-in-place allowed); property test that output throttle never exceeds any single sensor's cap.
- **Bench later [HW]:** glass door, shoe on the floor, cardboard box; record stop distances at 3 speeds into `docs/safety/README.md`.
- **Done when:** suite passes, docs updated, bench table filled.

### T1.3 Pi ↔ Orin link wiring + camera capture on the Pi [SW now; needs C1]
- **Goal:** `tools/pi_drive.py` (or a new `tools/pi_run.py`) becomes the full Pi process: LiDAR → safety + Orin, camera → Orin, Orin drive → safety → motor.
- **Context:** `locatebot/link/transport.py` (`Publisher`, `Subscriber`, `DriveReceiver`, endpoints; read its README section on newest-only and seq), `locatebot/pi/control.py` (`submit_mission(DriveCmd)`, `reset_mission_seq`), `docs/link/README.md` (drive/e-stop rules: network release never releases e-stop).
- **Spec:** PUB on 5555: every scan, `pi_status` at 2 Hz, `Range` at 10 Hz, camera `Frame` at 2 Hz (640 px JPEG, quality ~80, capture time = Pi monotonic at exposure). SUB to the Orin on 5556 via `DriveReceiver`; reset seq when `OrinStatus.boot_id` changes. Camera backend interface with `FakeCamera` (tests), `V4l2Camera` (USB, via OpenCV or `v4l2-ctl`-free pure approach), `PiCamera2Camera` (libcamera); choose by flag; camera failure never affects driving. Threads: lidar, camera, link-in, control 20 Hz, page server.
- **RAM budget:** measure RSS on the Pi; target < 200 MB for this process.
- **Tests:** loopback end-to-end: fake LiDAR + fake camera on the "Pi" side, `orin_run.py`-style subscriber on the "Orin" side in the same test: scans and frames arrive, drive commands drive `FakeMotor`, stale drive → stop, Orin restart (new boot_id) → seq reset.
- **Done when:** with the real cable, `tools/orin_run.py --pi-host <pi>` shows the live map and radar fed from the Pi, and manual drive still works with the Orin unplugged.

### T1.4 Physical safety tuning [HW, needs T1.1–T1.3]
Measure braking distance at the capped speed, sensing latency, car width; set `SafetyConfig` (`stop_distance_m`, `half_width_m`, `slow_distance_m`, ultrasonic thresholds); record the `LOW_COVERAGE` rate on clean scans (add hysteresis only for coverage if needed). Drills: pull the LiDAR USB, pull the Ethernet, kill the Orin, kill the page tab, press E-stop at speed. **Gate G1 passed when** every drill stops the car and the numbers are in `docs/safety/README.md`.

---

## G2 — the Orin drives the car
### T2.1 Vehicle + pursuit calibration [HW, needs G1]
Fill `VehicleConfig` from G0 measurements; measure speed vs throttle (time 3 m at 3 throttles) for `max_speed_mps`; check `steering = +1` turns left; tune `PursuitConfig` (lookahead, `min_throttle_frac` must not stall, `goal_tolerance_m`). Drive `go_to` to taped floor marks 10 times; record error. Context: `docs/nav/README.md` ("To measure or tune on the real car").

### T2.2 Odometry while driving [HW + SW]
Record `.lbscan` logs while driving loops at 3 speeds; replay with `tools/build_map.py`; measure loop-closure error and rejected-scan rate. If heading drifts on turns: add an IMU (MPU6050 on the Pi I²C, gyro z only) as a motion prior to `ScanOdometry.update` (prediction instead of constant velocity). Investigate the one-off 14° heading jump seen on the bench (2026-11-08, `docs/orin/README.md`) with a 10-minute static log.

### T2.3 Planning off the control thread [SW now]
- **Why:** Orin control step max 221 ms; a search's first patrol plan ~0.6 s on the Orin → the drive sender sends zero and the car pauses.
- **Spec:** `SkillExecutor` gets a `Planner` worker thread: `tick` submits (start, goal, costmap version) and keeps following the old path (or holds zero) until the new one arrives; costmap built in the worker; cancellation by version number; `tick` never blocks > 20 ms. Keep the synchronous mode for tests (`planner="inline"`).
- **Tests:** existing 21 skill tests pass in both modes; a slow fake planner (0.8 s) never makes `tick` exceed 20 ms; stale results discarded after a new goal.

### T2.4 Vision on the car [HW, needs T1.3, D6]
Measure camera HFOV (`--hfov-deg`) and its position relative to the LiDAR (`LocalizeConfig.camera_xy`); align bearings (point at a post, compare vision bearing to LiDAR bearing); test the item list's wording on photos from the car camera (`tools/locate.py`); confirm item fixes land within 0.3 m of tape-measured truth. **Gate G2 passed when** the car maps a room driven manually, then `search` + `go_to` from the operator page finds and reaches 2 real items.

---

## G3 — local storage: SpacetimeDB on the Pi [SW now; needs C3, D3]
### T3.1 SpacetimeDB server + module
- **Facts:** official Linux **aarch64** build exists (`spacetime-aarch64-unknown-linux-gnu`, v2.14.1, 2026-11-04); needs a 64-bit OS. Use the docs via context7 for the 2.x module/HTTP APIs; don't rely on memory.
- **Placement:** on the Pi (keeps Orin RAM for vision); measure RSS idle and under load on the Pi; fall back to the laptop if it exceeds ~400 MB or the Pi has 2 GB.
- **Module** (TypeScript) `spacetime/locatebot/`: tables `robot_status` (one row: pose, skill, safety, estop, link health, updated_at), `item` (name, x, y, sigma, sightings, confirmed, first/last seen, photo_ref), `event` (t, kind, text), `mission` (id, source (dashboard/telegram), target, state, created/updated), `map_session` (name, file path on disk, created, size, scans), `health` (pi/orin CPU, RAM, temps, gpu, vision latency, scans/s). Reducers for each write; retention (keep last N events).
- **Rules:** the robot must work if SpacetimeDB is down; no internet; data dir under `/var/lib/locatebot/spacetime` (git-ignored); systemd unit.
- **Done when:** module publishes on the Pi; a test script writes and reads every table; RAM measured.

### T3.2 Python bridge [SW, needs T3.1 schema]
`locatebot/storage/bridge.py`: background thread with a bounded queue that turns `OrinApp.status()`, events, item updates and mission changes into reducer calls over SpacetimeDB's HTTP API (stdlib `urllib`); rate-limited (status ≤ 2 Hz); drops on overflow; never blocks the control loop; reconnects. Map files stay on disk (`maps/`), only metadata in the DB. Tests against a stub HTTP server; one integration test against a real local server (skipped if absent).

---

## G4 — dashboard [SW, needs T3.1]
React + TypeScript + SpacetimeDB client, **all assets bundled, no CDN**, served by the Pi (static files from a stdlib server or the SpacetimeDB host). Screens (`docs/PROJECT.md` §9): Mission (live map from `/map.png` + overlay of pose/items, skill, approve/reject LLM proposals, skill buttons, goal text box), Health, Manual drive (talks to the Pi page endpoints `/drive` `/estop` `/release`, hold-to-drive semantics identical to `locatebot/pi/server.py`), Radar (port of `/radar`). E-stop visible on every screen. Replaces the two stdlib pages only after it matches them (keep them as fallbacks). Check in headless Chromium like the stdlib pages (CDP script approach in `docs/orin/README.md`).

---

## R — remote control via Photon/Telegram, on the Pi (rework of `feat/photon-dispatch`)
**Review of the branch (2026-11-08):** 4 commits by chanabyte, 39 files, ~4,700 lines; branched before the Pi+Orin runtime existed. It merges onto `main` with one trivial `.gitignore` conflict and all 867 tests pass together. Verdict: **keep the messaging half, replace the robot half.**

| Part | Fit | Action |
|---|---|---|
| `backend/messaging/photon.py` (Standard Webhooks verification, `message.received` parsing, send client), `commands.py` (`/find /status /cancel /help`), `notifier.py` (outbox, retries, `Retry-After`), `store.py` (SQLite dedupe + outbox in one transaction), `missions.py` rules (one active mission, ownership, rate limit, timeouts), `config.py` (secret-safe env) | Good, transport-agnostic, well tested | **Keep**, move to `locatebot/remote/` |
| Inbound via public HTTPS webhook (needs a tunnel; "backend runs off the robot") | Wrong for the Pi | Replace with Photon **WebSocket delivery** (documented for processes without a public HTTP endpoint); keep webhook mode optional |
| FastAPI + uvicorn | Heavy for the Pi; rest of the repo is stdlib HTTP | Drop; stdlib `ThreadingHTTPServer` for the few local operator routes |
| `dispatch/ws_client.py` + Jetson `websockets` robot link | Duplicates our ZeroMQ link, adds a dependency on the Orin | **Replace** with `MissionRequest/Cancel/Status` over the existing link (C1) |
| `dispatch/agent.py` (`JetsonAgent`) + `MissionSink` | Good state machine; wrong side and transport | Keep the logic, run it on the **Orin** behind the ZeroMQ link (T R3) |
| `ScriptedSink`, `tools/fake_jetson.py` | Useful for tests | Keep as test fakes |
| "Mission" = single `target_description` | Our robot has skills + LLM | Map onto `WorldState.goal` + `search`/`go_to` (T R3) |

### R1. Port the messaging core to `locatebot/remote/` [SW, needs C1]
Move/adapt photon, commands, notifier, store, missions, config from `origin/feat/photon-dispatch` with their tests; keep history (`git checkout origin/feat/photon-dispatch -- <paths>` then `git mv`); remove FastAPI; credit the original commits in the message. All tests green without the `backend` extra.

### R2. Photon WebSocket inbound + Pi service [SW now, live test needs D5]
Implement the Photon WebSocket delivery client per `https://docs.photon.codes/docs/beta/message-delivery/websocket.md` (read it first; same event object as webhooks): outbound connection from the Pi, auth with the project key from env, reconnect with backoff, dedupe by event id in the SQLite store, same handlers as the webhook path. Process `tools/remote_run.py` on the Pi; RAM target < 80 MB; robot keeps working when offline (Telegram replies "robot offline / no internet").

### R3. Mission bridge on the Orin [SW, needs C1, C3, R1]
`locatebot/orin/missions.py`: receives `MissionRequest` over the link → validates (expiry, one at a time) → sets `WorldState.goal` = "find <target>" and either lets the LLM act (approve/auto) or, with `--llm none`, runs the routine `search([target])` then `go_to(target)`. Maps executor/tracker events to `MissionStatus`: ACCEPTED on start, SEARCHING when search runs, TARGET_DETECTED when the target track is confirmed (include map position), APPROACHING on `go_to`, COMPLETED on arrival, FAILED with `NOT_FOUND` / `NO_PATH` / `TIMEOUT`; `MissionCancel` → operator STOP → CANCELLED once stopped. Link loss to the Pi → stop (safety on the Pi also stops). Reuse `JetsonAgent`'s duplicate/expiry/one-at-a-time rules. Tests in simulation using `tests/orin/test_e2e_sim.py` patterns.

### R4. Brain goal input [SW, needs C3]
`render_world_state` prints `goal: <text>` at the top; the system prompt says to work towards the goal and call `stop` when it is done; goal set/cleared via `OrinRuntime.set_goal()` and the operator page text box. Tests: goal appears in the prompt; scripted policy reaches COMPLETED in the e2e sim.

### R5. Live Photon test [HW/people, needs D5, R2, G2]
Follow TEST-PLAN L1–L4 from the branch (`docs/dispatch/TEST-PLAN.md`): real project, Telegram bot, one captured event (PII removed) replacing the synthetic fixture; confirm `senderId` and `format: "plain"` behaviour.

---

## M — smart map: long-term map, people, extra layers [SW in sim now, validate after G2]
### M1. Reuse a saved map: localization mode [SW]
Load `.lbmap` at start; localize with the likelihood-field matcher against the saved grid (initial guess = dock pose or operator click); keep updating slowly (lower log-odds increments for confirmed cells). Tests: start from a pre-built sim map at a known offset → converge within 0.1 m / 3°.
### M2. Static vs dynamic layers + change detection [SW, needs M1]
Separate short-lived dynamic layer; scan points landing in confidently-FREE static cells are "new"; clustered with `cluster_scan`; dynamic points never burned into the static layer. `.lbmap` v2 (`LBMAP2`) with named layers; v1 still readable.
### M3. People tracking + ultrasonic/glass layer + Wi-Fi coverage layer [SW, needs M2, C3]
Leg-pair/blob tracker with velocity (Kalman or α-β), `WorldState.people`; camera confirmation ("person") at low rate; ultrasonic-only hits from T1.2 marked as `glass_or_low_obstacle` cells (low weight, never erased by LiDAR "free" rays); Wi-Fi RSSI layer (signal to the team router per cell) for link dead zones. Tests in sim with moving boxes as people.

---

## G5 — LLM live [needs G2, D4, R4]
Pick the Nemotron model id on build.nvidia.com that supports tool calling; check the reply parser handles its output (some Nemotron reasoning models also return `reasoning_content`, and some take a system-prompt switch for reasoning on/off); measure decision latency. Run it against the real robot in **approve** mode; log every prompt/answer; tune the system prompt on 10 real missions; measure latency and cost; confirm fallback (pull the hotspot mid-mission → robot finishes the skill and stops). Context: `docs/agent/README.md`, `locatebot/agent/llm.py`.

## G6 — event-ready [needs everything above]
systemd units with auto-restart and `MemoryMax=` (Pi: `pi_run`, `remote_run`, SpacetimeDB; Orin: `orin_run` with vision); boot order and health checks; offline kit (wheels/pip cache, npm bundle, model, SpacetimeDB binary on both devices); full dry run with Wi-Fi off (local dashboard only); failure drills (cable, Orin, LLM, Telegram, SpacetimeDB down); battery plan; demo script.

---

## What can start in parallel right now (no hardware needed)
After **Step C** is merged: **T1.2** (fusion logic + fake ultrasonic), **T1.3** (link wiring + fake camera), **T2.3** (background planner), **T3.1 + T3.2** (SpacetimeDB on the laptop first), **R1 → R2/R3/R4**, **M1 → M2**. Hardware-gated: T1.1, T1.4, T2.1, T2.2, T2.4, R5, G5, G6.
