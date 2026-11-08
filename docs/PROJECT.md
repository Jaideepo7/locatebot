# locatebot — project overview

**This is the current source of truth for the whole project.** The numbered planning docs (`docs/00-…` to `docs/08-…`) were written before the Raspberry Pi + Jetson split and before any code existed; where they disagree with this file, this file wins.

Last updated: 2026-11-08.

---

## 1. What we are building

An RC car that **maps a room with LiDAR, then searches it for a list of items** using an open-vocabulary vision model, for a 24-hour hackathon.

It runs in two phases:

1. **Explore.** The car drives around a room. The LiDAR builds a 2D occupancy map, saved as a session map file (`.lbmap`, the "bin file").
2. **Search.** The user gives an item list ("water bottle", "red box", …). The car drives around the mapped area while LocateAnything looks for each item in the camera feed. Each item found is pinned on the map with its photo.

A web dashboard shows the map, the item list, found items, robot health, and offers manual driving with live video.

### Non-negotiable rules
- **Driving never waits for the AI.** The vision model runs in the background; the car keeps driving on LiDAR while it thinks.
- **Safety lives on the Pi, next to the motors.** The LiDAR emergency stop must work even if the Orin, the model, the Ethernet cable or the dashboard fails.
- **No internet dependency during the event.** Everything (model, packages, dashboard, database) runs locally on our own network.
- **Keep build-time risk low.** Use the published q4_k model as-is; no custom quantization during the hackathon.

---

## 2. Hardware

| Part | Role |
|---|---|
| Jetson Orin Nano Super 8 GB | "Thinking": LocateAnything, mapping, odometry, mission logic, dashboard backend |
| Raspberry Pi 4 | "Doing": motors and steering, LiDAR reading, emergency stop, camera stream |
| RPLIDAR C1 360° (fallbacks: Oradar MS200, RPLIDAR A1M8) | 2D scans, plugged into the **Pi** |
| Camera (on the Pi) | Video feed for the model and the dashboard |
| Amazon RC car kit | Chassis, drive motor, steering |
| Ethernet cable | Pi ↔ Orin link |
| Battery + power bank | Drive power and board power |

### Hardware facts to respect
- **The Orin Nano dev kit cannot be powered from its USB-C port.** It takes 9–20 V on the barrel jack (bundled adapter is 19 V). Running it from a power bank needs a USB-C PD 20 V trigger cable to a barrel plug and a bank that supports 20 V output. Verify before the event.
- **8 GB is shared** between CPU and GPU. The q4_k model alone peaked around 4.8 GB in a CPU test.
- **No battery reading in the dashboard** (decided; no INA219 sensor). Watch the batteries manually.
- **Unknown yet:** how the car's motor and steering are driven (ESC / H-bridge / motor HAT) and whether it has wheel encoders. The code assumes **no encoders**: position comes from LiDAR scan matching.

---

## 3. System architecture

```text
                        ┌───────────────────────── Raspberry Pi 4 ─────────────────────────┐
 RPLIDAR C1 ──USB──▶    │ rplidar.py → detect.py → SAFETY STOP ──▶ motor/steering driver   │
                        │      │                        ▲                                  │
 Camera ──────────▶     │ camera streamer               │ drive commands (watchdog: stop   │
                        │      │                        │ if Orin/browser goes quiet)      │
                        └──────┼────────────────────────┼──────────────────────────────────┘
                               │ scans + JPEG frames     │
                         ══════╪══════ Ethernet ═════════╪═══════  (ZeroMQ, planned)
                               ▼                         │
                        ┌──────────────────────── Jetson Orin Nano ────────────────────────┐
                        │ mapping: odometry.py → grid.py → mapfile.py (.lbmap)             │
                        │ vision daemon: LocateAnything q4_k (kept loaded) → detections    │
                        │ item localizer: bearing + LiDAR range + pose → item on map       │
                        │ mission manager: explore → save map → search list → report       │
                        │ SpacetimeDB (local) ◀── bridge                                    │
                        └──────────────────────────────┬───────────────────────────────────┘
                                                       │ local Wi-Fi (own router/hotspot)
                                                  Browser dashboard
                                   (video + drive commands go directly to the Pi)
```

### Who decides what (priority order)
1. Physical kill switch / Pi motor watchdog
2. Pi LiDAR safety stop (obstacle too close, LiDAR stale, forward view not covered)
3. Manual drive commands from the dashboard (hold-to-drive)
4. Mission/navigation commands from the Orin
5. Vision results (only ever change *where* to go, never *whether it is safe*)

### Network
- Pi ↔ Orin: direct Ethernet, fixed IPs. Planned transport: ZeroMQ (`pyzmq`), keep-latest sockets for frames.
- Dashboard: our own travel router or a laptop hotspot. No venue Wi-Fi needed.
- Model weights, Python packages, npm packages and the engine build are all prepared **before** the event.

---

## 4. Software components and status

| Component | Where | Status |
|---|---|---|
| LiDAR driver + scan logs (`locatebot/lidar/rplidar.py`, `scanlog.py`, `tools/record_scan.py`) | Pi | **Done**, verified on the real C1 (2026-11-08): ~10 scans/s, ~508 points/scan, 2.2 s motor spin-up handled |
| Obstacle detection (`detect.py`) | Pi | **Done**, synthetic tests |
| Scan matching + odometry (`matching.py`, `odometry.py`) | Orin | **Done**, synthetic tests |
| Occupancy grid + `.lbmap` map file (`grid.py`, `mapfile.py`) | Orin | **Done**, synthetic tests |
| Session mapping pipeline + CLI (`mapping.py`, `tools/build_map.py`) | Orin | **Done**, end-to-end synthetic test |
| Model download (`scripts/get_model.sh`) | Orin | **Done**, tested on x86 server |
| Engine build (`scripts/build_locate_anything.sh`) | Orin | **Done**, CUDA build on the Orin verified with the streamed-weights patch |
| Vision engine + worker (`locatebot/vision/`) | Orin | **Done**; runs on the Orin GPU, ~2.1 s per item at 640 px |
| Pi ↔ Orin link + Pi safety stop + motor driver | Pi/Orin | Planned, needs motor hardware details |
| Item localization | Orin | Planned |
| Autonomous exploration | Orin | Planned |
| Mission manager | Orin | Planned |
| SpacetimeDB module + dashboard | Orin/laptop | Planned (§9) |
| Nav-lite (`locatebot/nav/`) | Orin | **Done**, simulation only (`docs/nav/README.md`) |
| LLM agent + fallback (`locatebot/agent/`) | Orin/laptop | **Done**, fake LLM only; never run against a real API/model (`docs/agent/README.md`) |
| Pi runtime + manual drive page (`locatebot/pi/`) | Pi | **Done** with a fake motor; page checked in headless Chromium; real motor driver needs hardware details (`docs/pi/README.md`) |
| Link transport (`locatebot/link/transport.py`) | Pi/Orin | **Done**, loopback only; not on the real cable |
| Orin runtime: live map, skills, brain wiring, operator page (`locatebot/orin/`, `tools/orin_run.py`) | Orin | **Done**, closed-loop simulation + real C1 mapping on the bench (`docs/orin/README.md`) |

Verified on hardware: the C1 LiDAR (laptop and Orin), the vision model on the Orin GPU, and the full test suite on the Orin. Not yet: the car's motors, the camera, the Pi, the Pi<->Orin cable.

---

## 5. Repository layout

```text
locatebot/
  lidar/            LiDAR subsystem (see docs/lidar/README.md)
    types.py        Scan, Pose2D, range limits         (shared contract — integrator-owned)
    sim.py          synthetic rooms + scans for tests   (integrator-owned)
    rplidar.py      RPLIDAR serial protocol driver
    scanlog.py      .lbscan record/replay
    detect.py       cleaning, clearance, coverage, obstacle clusters
    matching.py     likelihood-field correlative scan matcher
    odometry.py     keyframe LiDAR odometry
    grid.py         log-odds occupancy grid
    mapfile.py      .lbmap save/load + ROS PGM/YAML export
    mapping.py      session pipeline: scans → map files
tools/
  record_scan.py    record a .lbscan from the real sensor
  build_map.py      build a session map from a .lbscan, or a synthetic demo
scripts/
  get_model.sh               download pinned q4_k model to models/
  build_locate_anything.sh   build pinned locate-anything.cpp to third_party/
tests/lidar/        pytest suite
docs/
  PROJECT.md        this file
  lidar/            LiDAR design + lane specs
  vision/           model setup + test findings
  00-08, lanes/     original planning docs (partly superseded)
models/, third_party/   downloaded/built locally, git-ignored
```

---

## 6. LiDAR subsystem (built)

Full detail: `docs/lidar/README.md`.

**Conventions:** robot frame +x forward, +y left, angles counter-clockwise from forward, radians, metres, monotonic seconds. `NaN` range = **unknown, never free**.

**Pipeline:** `RPLidar.iter_scans()` → `clean_scan` → `ScanOdometry.update` → `OccupancyGrid.integrate` → `save_map` / `export_ros_map`.

**Safety-relevant behaviour**
- Driver raises `TimeoutError` if no complete revolution arrives within `scan_timeout_s` (default 2 s); the motor is stopped on every exit path; commands send STOP first so a still-streaming sensor recovers.
- `clean_scan` removes only points *farther* than both neighbours (a single close beam may be a thin pole and is kept).
- `corridor_clearance` returns `inf` when nothing is seen; callers must also require high `forward_coverage` (angular bins with a valid return) before trusting it.

**File formats**
- `.lbscan` (magic `LBSCAN2`): per-revolution records with header CRC and payload CRC; tolerates a truncated last record (power loss).
- `.lbmap` (magic `LBMAP1`): header, JSON metadata (trajectory, scan counts, session info), float32 log-odds cells, CRC32; written atomically. Also exported as `PGM + YAML` (ROS map_server format) for viewing.

**Measured on simulation (not hardware):** an 8×6 m room loop mapped with walls within 5 cm and final pose error ≈1 cm; ≈24 ms per scan through the whole pipeline on an x86 laptop CPU.

**Known limits:** no loop closure (drift grows on long runs), no scan de-skew (drive slowly while mapping), 2D scan misses obstacles above/below the sensor plane, glass and drop-offs.

---

## 7. Vision subsystem

### Model (set up)
- `locate-anything-q4_k.gguf` from `mudler/locate-anything.cpp-gguf`, pinned revision `4e50db5`, 4.72 GB, SHA-256 checked by `scripts/get_model.sh`.
- Engine: `mudler/locate-anything.cpp` pinned at `77376ab`, built with CUDA (sm_87) on the Orin by `scripts/build_locate_anything.sh`.
- Weights are NVIDIA's under NVIDIA's licence (research/non-commercial). Never commit them; `models/` is git-ignored.

### Findings so far (x86 CPU, see `docs/vision/README.md`)
- Model load ≈4 s; one detection ≈50 s on a slow laptop CPU. **Orin GPU speed is unknown — measure it first** (`hybrid` and `fast` modes).
- Prompt wording matters: `remote` found both remotes, `remote control` found none.
- Multi-item prompts (`a</c>b`) only found the first item in both tests → plan one item per photo, rotating through the list.
- Near-duplicate boxes and occasional whole-image boxes appear → post-process.
- Template: `Locate all the instances that matches the following description: X.`

### Vision daemon (designed, not built)
A long-running `systemd` service on the Orin (`locatebot-vision.service`, auto-restart, `MemoryMax=` cap).

- **Engine:** ctypes wrapper over `liblocate_anything.so` C API — `la_capi_load` once, `la_capi_locate_buffer` per JPEG. The model stays loaded.
- **Input:** receiver thread takes JPEG + capture timestamp from the Pi; keeps **only the newest frame**.
- **Worker:** runs the model on the newest frame for the current item (rotating), never blocking anything else. A running call cannot be cancelled; results for an item removed mid-call are discarded.
- **Post-processing:** merge same-label boxes with IoU > 0.7; drop boxes covering > ~80 % of the image; box centre → bearing `θ = −atan((cx − cx₀)/fx)` (left positive, matches LiDAR frame) from `config/camera.yaml`.
- **Output:** detections with the frame's *capture* time (so the mapper can look up the pose at that moment) + a 1 s heartbeat (alive, latency, last error).
- **Control:** set item list, pause/resume, status.
- **Planned files:** `locatebot/vision/engine.py`, `postprocess.py`, `service.py`, `config/camera.yaml`, `deploy/locatebot-vision.service`, `tests/vision/` (fake engine, including a "stays responsive during a slow call" test). Adds `pyzmq`.

### Item localization (planned)
Bearing from the vision daemon + LiDAR range along that bearing at the frame's capture time + robot pose → item position on the map, with uncertainty. Works only for items roughly at LiDAR height; an item on a high shelf needs another depth source. Never turn a box into a map position without a range.

---

## 8. Planned robot-side pieces

1. **Pi ↔ Orin link:** scans and frames Pi → Orin; drive commands Orin → Pi; heartbeats both ways.
2. **Pi safety stop + motor driver:** stop if an obstacle is inside the stopping envelope, the LiDAR is stale, forward coverage is low, or commands stop arriving. Speed limit in software. Needs the motor/steering interface (unknown).
3. **Exploration (phase 1):** drive towards the edges of known free space until coverage stops growing, then save the `.lbmap`. Car-like steering (no turning on the spot).
4. **Mission manager:** `IDLE → EXPLORING → MAP_SAVED → SEARCHING → REPORT`, with `BLOCKED / SENSOR_FAULT / E_STOP` from any moving state.

---

## 8a. LLM body: skills, Nav-lite and fallback (decided 2026-11-08)

An LLM is the high-level "brain"; the robot is its body. **The LLM decides *what*, the robot decides *how*.** It never drives the wheels.

```text
LLM ──(on events or every few s)──▶ SkillCall ──▶ skill executor (Nav-lite) ──▶ drive ──▶ Pi safety stop ──▶ motors
 ▲                                                    │
 └──── WorldState text summary ◀── pose, sector clearances, map coverage, item tracks, skill status
```

- **Contracts:** `locatebot/mission/types.py` (integrator-owned): `SkillName`, `SKILL_ARGS`, `SkillCall`, `SkillStatus`, `ItemView`, `WorldState`, `VehicleConfig`.
- **Skills (the only actions):** `explore`, `search(items)`, `go_to(item | x,y)`, `follow(item)`, `look_around`, `stop`, `return_home`. Arguments are validated before running.
- **Nav-lite (`locatebot/nav/`):** A* on the occupancy grid inflated by the car's half-width, path smoothing, pure-pursuit steering for a car-like (Ackermann) chassis with speed scaled by clearance, frontier exploration. No ROS.
- **Agent (`locatebot/agent/`):** `WorldState` → compact text; skills → tool definitions; pluggable LLM client (cloud API or local OpenAI-compatible server, decided later); **approve mode** (operator confirms each call) and **auto mode**.
- **Fallback:** if the LLM times out, errors or returns an invalid call, the current skill finishes and a preset routine runs (default: `stop`). Routines are plain skill lists, no AI.
- **Follow:** vision locks on to the item's bearing; the LiDAR cluster in that direction is tracked at scan rate; vision re-confirms every few seconds (the model takes ~2 s per frame).
- LocateAnything only finds what it is asked for: the LLM chooses the items to look for.
- The Pi safety stop and E-stop override every skill and every LLM decision.
- Open question: where the LLM runs (cloud API over a hotspot breaks the no-internet rule; local model on the laptop is weaker). The Orin has no RAM for an LLM next to LocateAnything.

### Pi runtime (`locatebot/pi/`)
Control tick: newest scan + chosen command (manual from the local dashboard beats mission from the Orin) → `SafetySupervisor.evaluate` → `MotorDriver`. Latched E-stop released only locally. Hold-to-drive: manual commands expire after `command_timeout_s`. `MotorDriver` is an interface with a fake; the real driver waits for the motor/steering hardware details.

### Link transport (`locatebot/link/transport.py`)
ZeroMQ PUB/SUB over the `messages.py` frames (`pyzmq`, extra `[link]`): newest-only for frames and drive commands, `seq` replay protection, re-stamp on receipt.

## 9. Dashboard and backend

**Backend:** SpacetimeDB, **run locally** (laptop by default; Orin only if its ARM build installs and leaves enough memory), TypeScript module. It holds slow-changing shared state: item list, found items, mission state, health numbers, saved sessions. The Orin's Python code talks to it through a small Node bridge or the HTTP API. **The robot must work fully if SpacetimeDB is down.**

**Frontend:** TypeScript + React + SpacetimeDB client, served locally with all assets bundled (no CDN).

| Screen | Contents |
|---|---|
| Mission (main) | Live map growing with robot pose and path; item list (add/remove/reorder, status per item); found-item pins with photo thumbnail and box; Start explore / Save map / Start search / Pause / Return; coverage % and items found |
| Health | Pi and Orin CPU, RAM, temperature (Orin GPU from `tegrastats`); link status green/amber/red for Pi, Orin, LiDAR, vision daemon; LiDAR scans/s, seconds per model run, Pi↔Orin round-trip; event log. **No battery reading.** |
| Manual drive | Live video with LiDAR radar overlay; WASD / joystick; speed limit; **hold-to-drive** (stops on release or disconnect); **E-stop visible on every screen** |
| Extras (if time) | Session replay from a saved `.lbmap`; detection debug view (raw boxes + confidence); settings (camera mount, speed limits, prompt tester) |

**Data paths**
| Data | Path |
|---|---|
| Map, items, results, mission state, health | SpacetimeDB → browser |
| Live video | Pi → browser directly (MJPEG) |
| Manual drive + E-stop | Browser → Pi directly (WebSocket); the Pi stops if commands stop; the LiDAR stop still overrides |

Priority for 24 h: E-stop + manual drive with video → mission view → health → extras.

---

## 10. Development workflow

- **One repo, feature branches per lane, integrate, then `main`.** Each lane owns its files; shared contracts (`locatebot/lidar/types.py`, `sim.py`, `pyproject.toml`) belong to the integrator. Lanes import only shared contracts, never each other, so merges don't conflict.
- **Test first.** Write the failing test, implement, run the whole suite. Hardware-free tests use `locatebot.lidar.sim` or fake engines/transports.
- **Merge complete, tested slices** after an independent review by a different model/session than the author.
- **Agents:** Claude Max holder = integrator/reviewer; Claude Pro ×3 and ChatGPT Plus (Codex) = lanes. Each person uses their own account.
- **Branches today:** `main` (everything merged). Local-only on the laptop: `lidar-integration`, `lidar/io`, `lidar/detect`, `lidar/grid`, `lidar/match` (merged history), and the old `integration`, `feat/*` planning branches (stale, can be deleted).

---

## 11. Before the hackathon (hardware bring-up)

1. **Orin:** power via barrel jack; `scripts/get_model.sh`; `export PATH=/usr/local/cuda/bin:$PATH && scripts/build_locate_anything.sh`; time 5–10 real item photos in `hybrid` and `fast` modes with `tegrastats` running; test each item's wording.
2. **LiDAR on the Pi:** `tools/record_scan.py`; check ~10 scans/s, forward = angle 0 (else `--mount-offset-deg`), tape-measure a distance; carry the car around a room and `tools/build_map.py --log …`.
3. **Car:** identify motor/steering interface; manual driving from the Pi with a physical kill switch; measure stopping distance at low speed.
4. **Network:** Pi ↔ Orin Ethernet with fixed IPs; own router/hotspot for the dashboard; full dry run with Wi-Fi off.
5. **Offline kit:** model, wheels/pip cache, npm packages, SpacetimeDB binary, engine build — all on the devices before arrival.

## 12. 24-hour plan (planning budget, not a promise)
| Hours | Focus |
|---|---|
| 0–2 | Bring-up check, network, assign lanes |
| 2–8 | Pi safety stop + drive link; vision daemon; dashboard E-stop/manual drive |
| 8–12 | Exploration + map saving on the real car |
| 12–18 | Search phase: vision → item localization → pins on map |
| 18–21 | Mission manager + dashboard mission view |
| 21–24 | Freeze, repeated runs, demo video, presentation |

**Cut first if late:** session replay, debug view, autonomous exploration (drive manually during phase 1 instead), multi-room maps. **Never cut:** the Pi safety stop, the E-stop, testing time.

## 13. Risks and open questions
| Risk / question | Mitigation |
|---|---|
| Model too slow or too big on the Orin | Measure early; one item per photo; `fast` mode; fall back to AprilTag stations |
| Motor/steering interface unknown | Identify now; placeholder driver in code |
| Orin power from power bank | 20 V PD trigger cable + compatible bank, or a separate battery + regulator |
| Odometry drift without encoders | Short exploration loops; slow driving |
| Items above/below the LiDAR plane | Accept 2D limitation for the demo or add a depth camera later |
| Venue network | Own router; everything offline |
| SpacetimeDB on ARM | Run it on the laptop |

## 14. Decisions log
| Date | Decision |
|---|---|
| 2026-11-06 | Jetson Orin Nano Super for on-robot AI; vision must never block driving |
| 2026-11-07 | Pi does control + LiDAR + camera; Orin does model, mapping, mission |
| 2026-11-07 | Map = log-odds occupancy grid saved as `.lbmap` (+ PGM/YAML) |
| 2026-11-07 | Model = mudler q4_k GGUF via locate-anything.cpp, pinned, never committed |
| 2026-11-07 | Backend = local SpacetimeDB (TypeScript module); no cloud, no internet dependency |
| 2026-11-07 | Dashboard: mission view, health (no battery), manual drive with video + E-stop |
| 2026-11-08 | Orin GPU needs the streamed-weights engine patch and 640 px frames (~2.1 s per item) |
| 2026-11-08 | LLM as high-level brain through fixed skills; Nav-lite (own A* + pure pursuit + frontiers) instead of Nav2; preset routines as fallback |
| 2026-11-08 | The LLM can answer `continue` to keep the running skill (mid-skill wake-ups otherwise restarted skills or tripped the fallback) |
| 2026-11-08 | Search = explore, then investigate glimpses, then patrol with slow camera sweeps (LiDAR mapping finishes long before the camera has looked everywhere) |
