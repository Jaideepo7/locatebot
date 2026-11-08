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
| Camera pan servo | ch 8 ("Servo 0") | ~11 µs per degree, > 1500 µs looks **left** |
| Camera tilt servo | ch 9 ("Servo 1") | 1500 µs ≈ level |
| ADS7830 8-ch ADC | I2C `0x48` | ch 2 = battery (raw ~160–180, **not calibrated to volts**); ch 0/1 probably light sensors |
| SSD1306 128×64 OLED | I2C `0x3C` | |
| Buzzer | GPIO 17 | active high |
| Ultrasonic | trigger GPIO 27, echo GPIO 22 | gpiozero `DistanceSensor` |
| IR line sensors L/M/R | GPIO 14 / 15 / 23 | 1 = surface detected |
| Camera | CSI CAM/DISP 0 | OV5647 detected; this mount needs **no** rotation. A Camera Module 3 + third-party cable was never detected |
| LiDAR | USB, CP2102N → `/dev/ttyUSB0` | **RPLIDAR C1**, 460800 baud, model byte `0x41`, health Good. Car front = LiDAR **2.4°** |
| Battery | 3 × 18650 in series (11.1–12.6 V) | Board "Power" switch feeds the Pi, "Load" feeds motors/servos. With board power off the I2C chips disappear |

Motor H-bridge truth table as used here: forward channel = PWM, other = 0 → drive; both = 4095 → **short brake** (stops in ~6 cm instead of coasting); both 0 → coast.

`pi/calibration.json` holds the measured forward channel per wheel, `swap_lr: false` (verified with before/after photos: `drive(l=40, r=-40)` turns **right**) and the LiDAR front angle.

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

Laptop env: `CAR_SSH` (ssh target, default `car`, an `~/.ssh/config` alias), `CAR_URL` (default `http://raspberrypi.local:8000`), `CAR_OUT` (default `./captures`). No addresses or credentials are stored in the repo.

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

## Toward `locatebot.pi.MotorDriver`
A real driver for this board would implement `set(throttle, steering)` (both [-1, 1], +steering = left) as skid steering, e.g. `left = throttle - k·steering`, `right = throttle + k·steering`, clamp each side to [-1, 1], and map each wheel through the forward-channel table above (`car.Car._wheel` shows the mapping and the duty floor). `stop()` = all channels 0–7 off (or a 0.15 s short-brake then off); it must not touch ch 8/9 so the camera keeps its pose. It still needs the hardware failsafe above. Not written yet.
