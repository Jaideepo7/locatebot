# Pi 5 + Freenove-style 4WD board: hardware prototype

**Status:** standalone prototype scripts, written and run on the real car on 2026-11-08. They live outside the `locatebot` package, are **not wired into `locatebot.pi`**, and have **no unit tests**. Use them as a hardware reference and a source for a real `MotorDriver`, not as production code.

Answers to the open hardware questions in `docs/PROJECT.md` §2 for this car: the drive is **4 DC motors through H-bridges fed by a PCA9685 PWM chip** (skid steering, no steering servo), and there are **no wheel encoders**.

## Hardware map (verified on the car)

Front = camera side. "Left/right" are the car's own left/right (looking forward from behind it).

| Part | Interface | Detail |
|---|---|---|
| Raspberry Pi 5, Raspberry Pi OS (Debian 13 trixie) | – | I2C, SPI enabled; serial left **off** (GPIO 14/15 are line sensors) |
| PCA9685 PWM chip | I2C bus 1 @ `0x40` (all-call `0x70`) | 50 Hz (shared with the servos) |
| Front-left motor | PCA9685 ch 0 / 1 | forward = ch **1** |
| Back-left motor | ch 2 / 3 | forward = ch **2** |
| Back-right motor | ch 4 / 5 | forward = ch **5** |
| Front-right motor | ch 6 / 7 | forward = ch **6** |
| Camera pan servo | ch 8 ("Servo 0") | **14.3 µs per degree** (measured: 220 µs = 15.4°), > 1500 µs looks **left**, usable ±35°, ~1.2° backlash |
| Camera tilt servo | ch 9 ("Servo 1") | 1500 µs ≈ level |
| ADS7830 8-ch ADC | I2C `0x48` | ch 2 = battery (raw ~160–180, **not calibrated to volts**); ch 0/1 probably light sensors |
| SSD1306 128×64 OLED | I2C `0x3C` | |
| Buzzer | GPIO 17 | active high |
| Ultrasonic | trigger GPIO 27, echo GPIO 22 | gpiozero `DistanceSensor` |
| IR line sensors L/M/R | GPIO 14 / 15 / 23 | 1 = surface detected |
| Camera | CSI CAM/DISP 0 | OV5647; this mount needs **no** rotation. libcamera picks a 1080p **crop** mode: f = **1290 px at 960 px wide, HFOV 40.8°** (measured with a LiDAR-measured turn). `AeExposureMode=Short` keeps photos sharp while driving. A Camera Module 3 + third-party cable was never detected |
| LiDAR | USB, CP2102N → `/dev/ttyUSB0` | **RPLIDAR C1**, 460800 baud, model byte `0x41`, health Good. Car front = LiDAR **182.4°** since the LiDAR was remounted on 2026-11-08 (it was 2.4° before). **Check after any remount:** `botd` `/bins` front slices must agree with the camera and the ultrasonic |
| Battery | 3 × 18650 in series (11.1–12.6 V) | Board "Power" switch feeds the Pi, "Load" feeds motors/servos. With board power off the I2C chips disappear |

Motor H-bridge truth table as used here: forward channel = PWM, other = 0 → drive; both = 4095 → **short brake** (stops in ~6 cm instead of coasting); both 0 → coast.

`pi/calibration.json` holds the measured forward channel per wheel, `swap_lr: false` (verified with before/after photos: `drive(l=40, r=-40)` turns **right**), the LiDAR front angle and per-wheel gains (`wheel_gain`, all 1.0: with fresh batteries and a correct LiDAR front the car drives straight). The copy here is the final state from the car (the Pi was off when this was committed; the values are the ones set live).

## Conventions: different from the package
These scripts predate reading the repo rules and **do not follow the package invariants**:
- LiDAR angles are the RPLIDAR's native **degrees, clockwise** (+90° = right), not radians CCW with +y left.
- `drive(left, right)` takes per-side speeds −100..100; turn sign: `turn > 0` = right.
- No `NaN`/coverage handling: a 10° slice with no returns is treated as 150 cm free (`motion.py`), which the package rules forbid ("NaN is unknown, never free").

## Files
| Path | What it does |
|---|---|
| `pi/hw.py` | Minimal drivers: `PCA9685`, `ADS7830`, `OLED` (smbus2 + Pillow) |
| `pi/car.py` | `Car`: `forward/backward/turn/drive/stop/brake`, time-capped, JSON results, calibration file, CLI |
| `pi/motion.py` | **asyncio motion core**: LiDAR thread + sonar/control/mission coroutines at 20 Hz. `approach` (drive until blocked) and `wander` (roam + avoid + event photos). Single-instance lock, SIGTERM/SIGHUP/SIGINT stop the motors |
| `pi/lidar.py` | RPLIDAR C1 standard-scan driver + top-down PNG plot |
| `pi/lidar_probe.py`, `pi/lidar_front.py` | Identify the LiDAR model; find close objects (front-angle calibration) |
| `pi/server.py` | HTTP control: `/snap /sense /drive /forward /backward /turn /pantilt /stop` (port 8000) |
| `pi/restart_server.sh`, `pi/stop_server.sh` | Start/stop `server.py` (it and `motion.py` both own the motors/sonar; run one at a time) |
| `pi/look.py`, `pi/pan_capture.py`, `pi/pantilt.py`, `pi/camtest.sh` | Camera + pan/tilt helpers |
| `pi/find_motors.py`, `pi/pulse.py`, `pi/servo.py`, `pi/stop.py` | Bring-up tools: channel finder, single pulses, servo sweep, all-channels-off |
| `laptop/carrun.sh` | Run a command **detached** on the Pi and collect its output (survives Wi-Fi drops, never double-starts) |
| `laptop/bot.py`, `laptop/step.sh`, `laptop/waitchange.py` | Client for `server.py`: drive + "camera is still" snapshots |
| `laptop/look.sh`, `laptop/lr_check.py` | Snapshot via SSH; left/right check by image shift (unreliable, see below) |
| `pi/botd.py`, `pi/restart_botd.sh` | **One daemon that owns the car** (sensors, 20 Hz control loop, camera, missions) and exposes it as HTTP tools for an LLM or a phone page (port 8000): `/state /stop /snap /photos /photo/<f> /look_around /pulses /turn /turn_deg /goto /aim /approach /wander /back /drift /wheel_gain /bins /beep`, plus a question inbox (`POST /ask`, `/inbox`, `POST /claim`, `POST /answer`) and a web page at `/`. Replaces `server.py` + the `motion.py` CLI while it runs |
| `pi/scanmatch.py`, `pi/turncal.py` | 2D ICP scan matching (rotation + shift between two scans) and a pivot calibration / closed-loop turn tool |
| `laptop/find.py`, `laptop/seek.py` | Earlier find-and-drive loops (kept for reference; `choose.py` is the one that works) |
| `laptop/choose.py` | **Find and drive to an object**: `scan "<what>"` → numbered choices + tagged photo sheet; `go N` → orient, drive, arrive (see below) |
| `orin/locate_server.py` | Runs on the Orin: keeps LocateAnything loaded on the GPU and answers `POST /locate?item=...` (JPEG body) with boxes; `GET /health` |

Laptop env: `CAR_SSH` (ssh target, default `car`, an `~/.ssh/config` alias), `CAR_URL` (default `http://raspberrypi.local:8000`), `ORIN_URL` (default `http://orin.local:8091`), `CAR_OUT` (default `./captures`). No addresses or credentials are stored in the repo.

## Find and drive to an object (LocateAnything + an LLM choosing)
Verified on the car on 2026-11-08: drove to an orange crate from ~5 m (one smooth run, stopped ~37 cm in front, confirmed by the ultrasonic), to the "EPICS 129" cardboard box several times, and found a yellow drill standing on the floor 4–5 m away.

```text
scan   car still; camera pans 35°L..35°R (5 photos) → LocateAnything on the Orin for "<what>"
       → colour check if a colour is named → same object in two photos merged → numbered choices + tagged sheet
choose an LLM (or a person) picks a number from the tagged photos
orient car still: ONE graded pulse at a time (0.04 / 0.06 / 0.08 s by error) + photo, until within ±4°
drive  far (box < 120 px): straight and continuous, NO steering; photo checked on the move (zoomed window for
       small targets); re-aim (stopped) only if > 4° off
       near: 0.4 s hops, a sharp stopped photo after each, re-aim if > 4° off
arrive ultrasonic/LiDAR confirms it is right in front (< ~40–60 cm) or it fills the frame;
       stopped by an obstacle while it was big and centred = arrived (the obstacle is the target)
```

```bash
# Orin (once per boot): model load ~65 s
~/locatebot/.venv/bin/python orin/locate_server.py            # needs ~/locatebot with the model + engine built
# Pi
pi/restart_botd.sh
# laptop
ORIN_URL=http://<orin>:8091 CAR_URL=http://<pi>:8000 python3 laptop/choose.py scan "orange box"
ORIN_URL=... CAR_URL=... python3 laptop/choose.py go 1
```

What made it work (each was a real failure first):
- **Keep the model loaded** on the Orin: 448 px frames take **1.1 s** per item (512 px 1.5 s, 640 px 2.1 s, 960 px ~5 s); the CLI reloads the 4.7 GB model every call (~60 s).
- **Colour check** for colour words: LocateAnything boxes brown cardboard for "orange box". Measured on this camera: orange crate hue 6 / sat 219, cardboard hue 33 / sat 115–145 (PIL hue 0–255). For "cardboard box" only vivid orange is rejected.
- **Turn while stopped, drive straight without steering.** Steering from a live LiDAR heading estimate while moving made the car circle (bad scan-to-scan matches in clutter). Measured straight driving: ±1–2° per 1.5 s at 30 %.
- **One pulse at a time.** Two or three pulses in a row overshoot (the first pulse from rest turns ~4°, following ones ~10°+).
- **Stop short, then creep.** A detection takes ~1.2 s; driving through it near the target overshoots. Re-aim at > 4°, or a small offset grows into driving past the target.
- **Clean start:** jump straight to the cruise power from standstill; a slow ramp lets one side start first and twists the car.

## Calibration numbers (2026-11-08)
- Braked pivot pulse at 0.36 power: 0.04 s ≈ 4°, 0.06 s ≈ 7°, 0.08 s ≈ 10° (first pulse from rest smaller). The LiDAR measures a still-to-still pivot to ~0.1° (ICP noise floor 0.01°) when the turn is small.
- Straight at 30 %: ~35 cm/s; 0.4 s hop ≈ 9 cm, ±0.8°.
- Moving-power floor 0.30 (0.25 let the weaker side stall and pivot the car); short-brake 0.1–0.15 s for crisp stops.

## Run
```bash
scp -r pi/ <pi>:car/                                  # code lives in ~/car on the Pi
ssh <pi> 'cd ~/car && python3 car.py calibration'     # no motion
ssh <pi> 'cd ~/car && python3 motion.py approach 40 3 --dry'   # full loop, motors off
laptop/carrun.sh 'python3 motion.py approach 50 6'    # drive at a box, stop at 20 cm
CARRUN_WAIT=320 laptop/carrun.sh 'python3 motion.py wander 40 180 --photos'
```
Keep the wheels off the ground for first runs and a hand near the car; there is no hardware kill switch.

## Measured on the real car (2026-11-08)
- **Approach + brake:** from 88 cm at a 50 % target, speed eased down from ~100 cm and the car stopped **20.8 cm** from a box (target 20 cm). Without predictive stop + short-brake it stopped at 5–7 cm (coasting on a shiny floor).
- **Wander, 3 min:** ~146 s driving, 40 avoid manoeuvres, 57 event photos, no hard collision reported by the sensors.
- **Turning by time is unreliable** on a slippery floor: 0.14 s pivot @ 40 % ≈ 16°, 0.2 s @ 50 % ≈ 40°. Turns need measuring (scan matching or a gyro).
- **Lower duty still moves fast:** 12 V into the motors means even ~30 % duty is quick; `motion.py` uses a 700/4095 floor.
- **LiDAR blind spots:** dark trousers returned nothing (legs 30 cm away were invisible); a ~30 cm cardboard box in front was below the scan plane. The ultrasonic caught both. → camera/ultrasonic are needed for low and dark objects.
- **SSH over the team Wi-Fi drops often**; run motion detached (`carrun.sh`).

## Known problems
- **No hardware failsafe.** The PCA9685 keeps its last PWM if the process is `kill -9`'d or hangs. Needed before unattended runs: a watchdog that pulls the PCA9685 OE pin or cuts motor power.
- **Rear clearance is always ~0** in `motion.py` (the LiDAR sees the car's own body behind it), so `avoid` never backs up.
- **Corner trap:** `wander` spent ~33 s pivoting back and forth between a wall and a cart. Needs an escape rule (several avoids in a few seconds → back up and turn ~150°).
- Came within ~1–3 cm of a person's dark-trousered legs: people need a camera detector and a larger margin.
- Some event photos are blurred (taken while still turning).
- `laptop/lr_check.py`'s phase correlation gave the **wrong** left/right answer once (low correlation peak); compare photos visually instead.
- Battery ADC is uncalibrated.
- **Going around obstacles is weak.** `choose.py` sidesteps (back up, ~30° toward the LiDAR-open side, short drive, look with the camera) but a small, far target behind an obstacle is often lost. Needs path planning on the LiDAR map.
- **A remounted LiDAR silently breaks everything:** with the front angle wrong by 180°, the car braked for things behind it, crawled, and the weaker side stalled into a left pivot. Re-check `/bins` after touching the LiDAR.
- `botd.py` is not started at boot (run `pi/restart_botd.sh`), and the motor chip occasionally returns I2C `Remote I/O error` (121); the control loop now logs it (`/state.control_err`) and keeps running instead of dying silently.
- `seek.py`/`find.py` live-LiDAR heading tracking (`/goto` without `visual=1`) is unreliable while moving; use `choose.py`.

## `locatebot.pi.MotorDriver` for this board
`locatebot/pi/pca9685_motor.py` (`PCA9685SkidSteer`, tests in `tests/pi/test_pca9685_motor.py`) implements the package's `MotorDriver` for this board: `set(throttle, steering)` (package frame, +steering = left) becomes `left = throttle - steering`, `right = throttle + steering`, scaled to keep the ratio, mapped through the forward-channel table above with a 700/4095 duty floor; `stop()` turns channels 0–7 off in one I2C write and never touches the camera servos; `brake()` short-brakes. It is tested against a fake I2C bus only: it has **not** driven the car yet and is not wired into `tools/pi_drive.py`. It still needs the hardware failsafe above.
