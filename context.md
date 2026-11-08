# locatebot: project context

One file to get up to speed on the whole project. Last updated 2026-11-08.
For full detail, `docs/PROJECT.md` is the source of truth; it wins wherever this file or the old numbered docs (`docs/00-*` to `docs/08-*`) disagree.

---

## 1. What it is

An RC car built for a 24-hour hackathon that **maps a room with LiDAR, then searches it for a list of items** with an open-vocabulary vision model.

1. **Explore.** The car drives around and builds a 2D occupancy map from the LiDAR, saved as a session map file (`.lbmap`).
2. **Search.** The user gives items ("water bottle", "red box"). The car patrols the mapped area while the vision model (LocateAnything) looks for each item in the camera feed. Each item found is pinned on the map.

Control is through a browser operator page and, optionally, a Telegram bot. An LLM (Nemotron via NVIDIA's API) can act as a high-level "brain" that picks what to do next.

A second use case shares the robot: a **store-guide agent** (`locatebot/store/`) that answers a shopper, checks stock and leads them to the shelf (mock robot so far).

## 2. Hardware

| Part | Role |
|---|---|
| Raspberry Pi 5 (Raspberry Pi OS, Debian 13 trixie) | "Doing": motors, LiDAR reading, safety stop, camera, ultrasonic |
| Jetson Orin Nano Super 8 GB | "Thinking": vision model, mapping, odometry, navigation, LLM brain; **the server**: operator HTTP API/page on port 8090 |
| Freenove-style 4WD board | 4 DC motors through H-bridges fed by a PCA9685 PWM chip (I2C `0x40`); **skid steering** (tank-style, turns on the spot); no steering servo. "Load" switch cuts motor/servo power, "Power" feeds the Pi |
| RPLIDAR C1 360° | On the **Pi** (USB, `/dev/ttyUSB0`). 2D scans (~10 scans/s, ~508 points/scan). Fallbacks: Oradar MS200, RPLIDAR A1M8 |
| Camera OV5647 (CSI) | Frames for the model and the dashboard; pan/tilt servo on PCA9685 ch 8/9 |
| HC-SR04 ultrasonic | Trigger GPIO 27, echo GPIO 22 (gpiozero) |
| Other board parts | Buzzer GPIO 17, IR line sensors GPIO 14/15/23, ADS7830 ADC `0x48` (battery on ch 2, uncalibrated), OLED `0x3C` |
| Battery | 3 × 18650 (11.1–12.6 V) for the car |
| Ethernet cable | Pi ↔ Orin link (fixed IPs) |

Measured pin/channel map: `prototypes/pi5_freenove/README.md`.

Hardware facts:
- The Orin must be powered from the **barrel jack** (9–20 V), not USB-C. Its USB-C port is device mode; plug the LiDAR into a USB-A port.
- 8 GB of memory is **shared** between CPU and GPU. The model alone takes about 4.7 GB.
- No wheel encoders: position comes from LiDAR scan matching only.
- No battery reading in software (the ADC is not calibrated to volts); watch batteries manually.
- **pigpio does not work on the Pi 5:** use gpiozero / lgpio.
- **LiDAR:** car front = LiDAR **2.4°**; it sees the car's own body behind it (rear returns ~0), so it needs an angle mask. The Orin's `--lidar-port` mode is a bench tool only.
- **Measured on the car (2026-11-08):** dark trousers and a ~30 cm box below the scan plane were invisible to the LiDAR; the ultrasonic saw both. Timed turns are unreliable on shiny floors. Short-brake stop: 20.8 cm from a box at 50 % (target 20 cm).
- **No hardware motor failsafe yet:** the PCA9685 keeps its last PWM if the process hangs or is killed.

## 3. Architecture

```text
                ┌──────────────────── Raspberry Pi 5 ────────────────────┐
 RPLIDAR ─USB─▶ │ rplidar driver → safety supervisor ──▶ PCA9685 motors  │
 Camera ──────▶ │      │                  ▲                               │
                │      │ scans + frames   │ drive commands                │
                └──────┼──────────────────┼───────────────────────────────┘
                       ▼   Ethernet, ZeroMQ (bind: open)   │
                ┌──────────────────── Jetson Orin Nano ──────────────────┐
                │ live mapping (scan matching + occupancy grid)           │
                │ vision worker (LocateAnything, newest frame only)       │
                │ item localization → item tracker                        │
                │ skill executor (Nav-lite) ◀── LLM brain (optional)      │
                │ operator HTTP API + page :8090 (the server)             │
                └──────────────────────────┬─────────────────────────────┘
                                           │ team Wi-Fi / hotspot
                          Browser operator page, Telegram bridge (my-spectrum-app/)
```

**Who decides (highest priority first):**
1. Physical kill switch / Pi motor watchdog
2. Pi LiDAR safety stop (obstacle too close, LiDAR stale, forward view not covered)
3. Manual drive from the Pi's page (hold-to-drive)
4. Mission/navigation commands from the Orin
5. Vision results (only change *where* to go, never *whether it is safe*)

**The LLM decides *what*, the robot decides *how*.** The LLM never drives the wheels; it calls one of a fixed set of skills.

## 4. Non-negotiable rules (invariants)

- Robot frame: +x forward, +y left, angles counter-clockwise, radians, metres, monotonic seconds.
- A `NaN` range means **unknown, never free**. `corridor_clearance` returns `inf` when nothing is seen, so anything that stops the car must also check `forward_coverage`.
- **Vision never blocks driving.** Model calls run in a background worker on the newest frame only.
- **The Pi's LiDAR safety stop is the final gate on motion**, above manual drive and mission commands.
- **No internet at runtime** by design: no CDN assets, no cloud services, no downloads after setup. (The Telegram bridge and a cloud LLM are deliberate exceptions; see §9.)
- The RPLIDAR driver stops the motor on every exit path and times out on a stalled stream.
- Model weights are NVIDIA-licensed (research/non-commercial): never commit or upload them.

## 5. Code map (`locatebot/`)

| Package | What it does | Runs on |
|---|---|---|
| `lidar/` | RPLIDAR driver, `.lbscan` record/replay, scan cleaning + obstacle detection, scan matching, odometry, occupancy grid, `.lbmap` save/load + ROS PGM/YAML export, offline mapping pipeline, synthetic rooms for tests | Pi + Orin |
| `safety/` | `SafetySupervisor`: per-tick check of each drive command against the newest scan; stops on e-stop, stale data, low coverage or obstacle; blocks reverse | Pi |
| `pi/` | Pi control loop (`PiController`), `MotorDriver` interface + `FakeMotor`, `PCA9685SkidSteer` (real driver for the Freenove board, not wired in yet), manual-drive web page (`:8080`) | Pi |
| `link/` | Pi ↔ Orin message format and ZeroMQ transport (scans, frames, status → Orin; drive, e-stop → Pi) | both |
| `vision/` | ctypes wrapper over `liblocate_anything.so`, background worker, post-processing (merge duplicate boxes, box → bearing) | Orin |
| `localize/` | Vision bearing + LiDAR range at the frame's capture time + pose → item position on the map; `ItemTracker` merges sightings and confirms items | Orin |
| `nav/` | Nav-lite: costmap, A* planner, pure-pursuit steering for a car-like chassis (approximate on the skid-steered car), frontier exploration, bicycle-model sim. No ROS | Orin |
| `mission/` | Shared contracts: `SkillName`, `SKILL_ARGS`, `SkillCall`, `SkillStatus`, `WorldState`, `VehicleConfig` | — |
| `agent/` | LLM brain: world state → text, skills → tool definitions, `NemotronClient` / `ClaudeClient` / `OpenAICompatClient` / `FakeLLM`, approve vs auto mode, fallback routines | Orin or laptop |
| `orin/` | Runtime wiring everything together (`OrinRuntime`), `SkillExecutor`, `LiveMapper`, `OrinApp` threads, operator HTTP page, `MissionBridge` (missions over the link; no Pi-side sender yet) | Orin |
| `store/` | Store-guide agent: catalog search, mock robot, LocateAnything + Nemotron VL check (`docs/store/README.md`) | Orin or laptop |
| `storage/` | PostgreSQL storage on the Pi 5: photos, items, events, missions, maps; never blocks the robot (built 2026-11-08, tested on a real PostgreSQL 16; not yet run on the Pi (`docs/storage/README.md`)) | Pi |

**Shared contracts (integrator-owned, don't change inside a lane):** `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `locatebot/mission/types.py`, `pyproject.toml`, `docs/PROJECT.md`.

Other folders: `tools/` (command-line entry points), `scripts/` (model download, engine build, engine patch), `tests/` (pytest, no hardware), `docs/` (design docs per subsystem), `my-spectrum-app/` (Telegram bridge, Node/TypeScript), `prototypes/pi5_freenove/` (standalone scripts run on the real car; a hardware reference only, they break the package invariants).

## 6. Skills: the robot's only high-level actions

| Skill | Args | Behaviour |
|---|---|---|
| `explore` | — | Frontier exploration until no reachable frontier is left |
| `search` | `{"items": [str, ...]}` (1–8) | Explore, investigate single sightings, then patrol waypoints ~1.5 m apart with a slow camera sweep; times out after 600 s |
| `go_to` | `{"item": str}` or `{"x", "y"}` | Drive to an item (stops short) or a map point |
| `follow` | `{"item": str}` | Keep a set distance behind a tracked item |
| `look_around` | — | One slow full-lock loop so the camera sweeps around |
| `stop` | — | Stop and hold |
| `return_home` | — | Drive back to the session start pose |

The LLM wakes on events (item confirmed, skill ended, safety stop) and either calls a skill or answers `continue`. If the LLM times out or errors, a preset routine runs (default: `stop`).

## 7. How to run

From the repo root. Install once: `python3 -m venv .venv && .venv/bin/pip install -e '.[hardware,dev,link]'`.

| Command | What it does |
|---|---|
| `.venv/bin/python -m pytest -q` | Full test suite, no hardware |
| `.venv/bin/python tools/orin_run.py --sim` | Whole Orin stack on a simulated room and car; open http://127.0.0.1:8090/ |
| `.venv/bin/python tools/orin_run.py --lidar-port /dev/ttyUSB0 --host 0.0.0.0` | Bench only: live map from a LiDAR plugged into the Orin (on the car the LiDAR is on the Pi); driving is a dry run |
| `.venv/bin/python tools/orin_run.py --pi-host <pi-ip> --vision --llm nemotron --mode approve --host 0.0.0.0` | The real car (once the Pi program drives the motors and publishes over the link) |
| `.venv/bin/python tools/pi_drive.py --sim` | Pi control loop + manual-drive page with a fake motor |
| `.venv/bin/python tools/record_scan.py --port /dev/ttyUSB0 --out recordings/x.lbscan --seconds 60` | Record real LiDAR scans |
| `.venv/bin/python tools/build_map.py --log recordings/x.lbscan --out maps/x` | Build a map from a recording (`--demo` for synthetic) |
| `scripts/get_model.sh` | Download the pinned q4_k model (~4.7 GB, checksum-verified) |
| `scripts/build_locate_anything.sh` | Build the pinned engine (CUDA on the Orin), applying `scripts/patches/` |

`orin_run.py` options: `--llm none | nemotron | claude | local` (`nemotron` reads `NVIDIA_API_KEY`; `claude` reads `ANTHROPIC_API_KEY`; `local` uses an OpenAI-compatible server via `--llm-url` / `--llm-model`), `--mode approve | auto`.

### Orin operator HTTP API (port 8090, no login: keep it on the team network)

| Method + path | Body | Purpose |
|---|---|---|
| `GET /` | — | Operator page |
| `GET /status` | — | JSON: pose, skill state, explored %, items, events, safety reason, brain, vision |
| `GET /map.png` | — | Current map image |
| `GET /scan.json`, `GET /radar` | — | Newest scan, live radar view |
| `POST /skill` | `{"name": "...", "args": {...}}` | Run a skill |
| `POST /stop` | `{}` | Stop |
| `POST /approve`, `/reject`, `/resume` | `{}` | Answer / resume the LLM brain |
| `POST /mode` | `{"mode": "approve" \| "auto"}` | Brain mode |
| `POST /save_map` | `{"name": "..."}` | Save the session map |

POSTs must send `Content-Type: application/json`.

## 8. Status

**Verified on real hardware:**
- RPLIDAR C1 on a laptop and on the Orin (~10 scans/s, ~508 points/scan, 2.2 s spin-up handled), and on the Pi 5 in the car
- Motors, camera, pan/tilt, ultrasonic and LiDAR on the Pi 5 car, through the standalone prototype scripts (`prototypes/pi5_freenove/`), not the `locatebot` package
- Vision model on the Orin GPU: ~2.1 s per item on 640 px frames (with the streamed-weights patch)
- Live LiDAR mapping on the Orin bench
- Full test suite on the Orin

**Built, tested in simulation only:** safety supervisor, Pi control loop and manual-drive page (fake motor), `PCA9685SkidSteer` (fake I2C bus), Pi↔Orin transport (loopback only), item localization, Nav-lite, LLM brain on the robot (fake LLM only), Orin runtime closed loop, mission bridge (fake Pi over loopback). The store-guide agent ran live against Nemotron with a mock robot.

**Retired:** the Python Photon/Telegram service (`locatebot/remote/`), replaced by the Spectrum bridge (§9).

**Not built / not done yet:**
- `PCA9685SkidSteer` **wired into the Pi program** (`tools/pi_drive.py`) plus a hardware failsafe: the package cannot drive the car yet
- The Pi publishing scans/frames and accepting drive commands over the real Ethernet link (which side binds is not decided)
- Camera capture in the Pi program
- A differential-drive model for Nav-lite and the safety corridor (they assume a car-like chassis)
- LiDAR body mask + 2.4° front offset in the package
- Tuning on the car: `VehicleConfig` (width, speed), stop distance, camera mount offset and field of view
- PostgreSQL storage running on the real Pi (built and tested on a laptop Postgres) and the dashboard (§9 of `docs/PROJECT.md`); the Orin operator page is used for now

## 9. Chat bridge: iMessage + Telegram (`my-spectrum-app/`)

A Node/TypeScript app on [Spectrum](https://photon.codes/docs/spectrum-ts) (`spectrum-ts`, Spectrum Cloud iMessage, `@spectrum-ts/telegram`) that turns chat messages into calls to the Orin operator API in §7. It is the **only** chat path: the older Python Photon service on the Pi (`locatebot/remote/`) was retired on 2026-11-08. No robot code depends on the bridge.

- **Commands** (slash optional): `find a, b` (search), `explore`, `goto <item>`, `follow <item>`, `look`, `home`, `stop`, `status`, `map` (sends the map image), `savemap <name>`, `approve`, `reject`.
- **Notifications:** polls `/status` every 2 s; tells chats that sent a command when an item is confirmed, a skill ends, a safety stop fires, the brain wants approval, or the robot is unreachable.
- **Access:** open by default: anyone who messages the bot can use it. Set `RESTRICT_SENDERS=1` to allow only `ALLOWED_SENDERS` (phone numbers or Apple ID emails for iMessage, chat ids for Telegram).
- **Config (`my-spectrum-app/.env`, git-ignored, never commit):** `SPECTRUM_PROJECT_ID`, `SPECTRUM_PROJECT_SECRET`, `IMESSAGE_ENABLED=1`, `SPECTRUM_TELEGRAM_BOT_TOKEN` (empty = Telegram off), `ROBOT_URL` (default `http://127.0.0.1:8090`), `RESTRICT_SENDERS`, `ALLOWED_SENDERS`. See `my-spectrum-app/.env.example`.
- **iMessage** uses Photon's cloud lines (no Mac needed). On Free/Pro plans the number is from a shared pool and each user's phone number must be added as a project user in the Photon dashboard.
- **Run:** `cd my-spectrum-app && npm install`, then `npm run telegram`, `npm run imessage`, or `npm run start` (whatever `.env` enables).
- **Status:** type-checks and the startup checks work; not yet run end to end against the robot or a real iMessage line.

**Networking.** The bridge only makes **outgoing** connections (to Photon/Telegram and to the Orin), so no ngrok, port forwarding or public server is needed. It does need internet, which the school network may block for the robot. Plan: give the Orin internet through a phone hotspot or travel router and run the bridge on the Orin. A cloud server (e.g. Vultr) only helps if the robot can already reach the internet, via an outgoing tunnel. Telegram adds seconds of lag: it is not the emergency stop.

## 10. Open questions and risks

| Question / risk | Current answer |
|---|---|
| Motor interface | Answered: PCA9685 + H-bridges, skid steer. `PCA9685SkidSteer` built, not wired in; no hardware failsafe yet (watchdog on the PCA9685 OE pin or motor power) |
| Pi ↔ Orin ZeroMQ link | Which side binds and which connects on the real car is **not confirmed** |
| Skid steering vs Nav-lite | Nav-lite and the safety corridor assume a car-like (Ackermann) model; works approximately through the driver's left/right mix; needs a differential-drive model |
| GPIO on the Pi 5 | pigpio does not work; use gpiozero / lgpio (the `pi` extra in `pyproject.toml` still names pigpio) |
| Where the LLM runs | Decided: Nemotron via NVIDIA's API; needs internet at the event (deliberate exception); the Orin has no RAM spare next to the vision model |
| Storage | PostgreSQL on the Pi 5 (not SpacetimeDB, which keeps all data in memory): built 2026-11-08, tested on a real PostgreSQL 16; not yet run on the Pi (`docs/storage/README.md`) |
| Odometry drift (no encoders, no loop closure) | Short exploration loops, drive slowly |
| Items above/below the LiDAR plane | Can't be localized (no range); accepted for the demo. Low and dark obstacles are invisible to the LiDAR too: the ultrasonic must guard them |
| Network at the venue / school | Own router or hotspot; keep everything runnable offline |
| Orin power from a battery | Needs a 20 V USB-C PD trigger cable to a barrel plug, or a separate battery + regulator |

## 11. Working rules

- Read `docs/PROJECT.md` first; agent rules live in `AGENTS.md`.
- Test first; run the **whole** suite before and after a change. No hardware in tests: use `locatebot.lidar.sim` and fakes.
- Synthetic tests prove software behaviour, not real-sensor behaviour or physical safety. Say what still needs a hardware test.
- One branch (and one git worktree under `.worktrees/`) per lane; lanes import only shared contracts, never each other.
- Conventional commits (`feat(lidar-detect): …`, `fix(vision): …`). Don't push, merge or add dependencies unless asked.
- Never commit: `models/`, `third_party/`, `maps/`, `recordings/`, `.env`, credentials, private hostnames or IPs.

## 12. Lessons already learned (don't repeat)

- **Prompts:** `remote` finds remotes where `remote control` finds none; multi-item prompts only find the first item. Use one item per call and test each item's wording.
- **Frame size:** send 640 px frames; 1600 px runs out of GPU memory. `hybrid` mode is as fast as `fast`.
- **Orin GPU memory:** keep `scripts/patches/locate-anything-streamed-weights.patch`, or the model won't load.
- **Spike filtering:** only remove LiDAR points *farther* than both neighbours, or thin poles vanish.
- **Coverage:** measure it over angular bins, not beam counts, or a blind wedge reads as 100 %.
- **C1 spin-up:** no data for ~2.2 s after SCAN; the driver allows 5 s.
- **Map names:** `Path.with_suffix` cuts dotted names; `Mapper.save` appends suffixes instead.
- **Serial access:** group `dialout` on Ubuntu/JetPack, `uucp` on Arch.
- **pkill over SSH:** use `pkill -f "[p]attern"` so it doesn't kill its own shell.
- **Orin CPU** is ~3x slower than a laptop for Python planning; no laptop-speed timing budgets in tests.
- **LLM mid-skill wake-ups** restarted skills until the `continue` tool was added.
- **pigpio on the Pi 5** doesn't work; GPIO goes through gpiozero / lgpio.
- **LiDAR self-hit:** the car's own body behind the LiDAR reads ~0 range; mask those angles.
- **Prototype scripts** (`prototypes/pi5_freenove/`) use clockwise degrees and treat no-return as free: reference only, never copy their conventions into the package.
