# Operating the LocateBot car as an LLM

A guide for any LLM agent (with a shell and HTTP access) to do what Claude did on 2026-11-08: bring the car up, find and drive to objects people ask for, answer requests from the phone app, and change the car's software without breaking it. Everything here was done on the real car; the numbers are measured.

Paths are relative to `prototypes/pi5_freenove/`. Addresses come from the environment, never from the repo:
`CAR_URL` (default `http://raspberrypi.local:8000`), `ORIN_URL` (default `http://orin.local:8091`), `CAR_SSH` (an `~/.ssh/config` alias, default `car`).

---

## 1. What you are driving

```text
 phone app / judges ──HTTP──┐
                            ▼
 laptop (you) ──HTTP──► Pi 5: botd.py  (port 8000)          Jetson Orin: locate_server.py (port 8091)
   laptop/choose.py          owns motors (PCA9685), camera,    LocateAnything kept loaded on the GPU
   laptop/qa.py              LiDAR, ultrasonic; 20 Hz control  POST /locate?item=… (JPEG) → boxes
        └──────────── photos from the car ──► Orin ──► boxes back to the laptop
```

- **Skid-steered 4WD**, no encoders, slippery floor: turns are pulses checked with photos, not timed rotations.
- **Sensors:** RPLIDAR C1 on top (misses dark fabric and anything lower than ~30 cm), one ultrasonic at the front (narrow cone), camera on a pan/tilt servo (pan ±35°).
- **Angles in this prototype:** degrees, **+ = right** (clockwise). The `locatebot/` package uses the opposite convention; don't mix them.
- **Camera:** 960×540, focal length 1290 px, horizontal field of view 40.8°. Bearing of a box centre `cx`: `atan((cx − 480) / 1290)` + camera pan.

## 2. Your tools (botd HTTP API)

All GET unless marked POST; JSON unless noted. Missions are exclusive: starting one cancels the last.

| Endpoint | What it does | Notes |
|---|---|---|
| `/state` | Sensors, mission, recent events, `control_age_s`, `control_errs`, `battery_adc` | Read it before every motion |
| `/stop` | Cuts the motors and cancels any mission | Works even if the control loop is stuck. Use it freely |
| `/snap?pan=0&tilt=1500` | JPEG now; pans the camera there first (waits 0.6 s) | `?fast=1` = no servo move, for use while driving |
| `/look_around` | Stops, pans 35°L→35°R, 5 photos (names in the result) | Photos are taken mid-pan: a bit blurred |
| `/photo/<name>` | One saved photo | |
| `/bins` | Nearest LiDAR return per 10° slice (cm, + = right) | Compare the front with a photo after anyone touches the LiDAR |
| `/pulses?dir=left\|right&n=1&secs=0.08` | Braked pivot pulse(s) | 0.04/0.06/0.08 s ≈ 4/7/10°; the first pulse from rest turns less. **Send one at a time** and re-check with a photo |
| `/turn_deg?deg=30` | Pivot measured by LiDAR scan matching | Fine in open rooms; unreliable in clutter |
| `/goto?bearing=0&seconds=1.5&speed=30&visual=1` | Drive toward a bearing | `wait=0` returns at once; `watchdog=4` stops it if nobody calls `/aim` for 4 s |
| `/aim?rel=-3&visual=1` / `/aim?keep=1` | Update a running `/goto`'s direction / just feed the watchdog | |
| `/back?seconds=0.4` | Short reverse (max 0.8 s) | No rear check: keep it short |
| `/approach?speed=40&seconds=6` | Drive forward until ~20 cm from something | |
| `/wander?seconds=60&speed=40` | Roam and avoid on its own | `head=1`: cute mode (camera head sweeps, looks before turning, look-arounds, corner escape; 34 % default). Weak around people: see §4 |
| `/manual?l=0.4&r=0.4` | Raw wheel power for ~0.35 s (the app's joystick) | Capped ±0.55; forward refused near obstacles or with no distance reading. Prefer `/goto` + `/pulses` yourself |
| `/pan?deg=-20`, `/beep?ms=150` | Camera pan, horn | |
| `/stream` | MJPEG live video | Small frames automatically through a tunnel |
| POST `/detections`, `/vision` | Draw boxes on the app's live video | `choose.py` does this for you (§7) |
| `/inbox?status=…`, POST `/ask` `/claim` `/progress` `/answer` `/upload?name=` | The app's request queue | Use `laptop/qa.py` (§6) |

Helpers on the laptop: `laptop/choose.py` (find and drive), `laptop/qa.py` (app inbox), `laptop/deploy_app.sh`, `laptop/carrun.sh` (run something detached on the Pi; survives Wi-Fi drops).

## 3. Bring-up checklist (before anything moves)

1. **Orin:** `curl $ORIN_URL/health` → `"ok": true`. If it's down, start it on the Orin: `~/locatebot/.venv/bin/python orin/locate_server.py` (model load ~65 s).
2. **botd alive:** `/state` → `control_age_s` below 1. A huge value means the control loop isn't running: don't drive.
3. **Motor power:** `battery_adc` above 0. A reading of 0 means the battery switch is off, so the wheels won't turn even though everything else works.
4. **I2C health:** `control_errs` should not climb by more than a few per minute. `control_err: OSError(121, …)` is the motor chip's bus.
5. **Distance sensors:** `front_cm` is a number (not `null`), and `/bins` front matches what `/snap` shows and the ultrasonic. A front near 0 or negative with nothing there means the LiDAR sees the car itself; a "wall" ahead that the camera and ultrasonic don't see means `lidar_front_deg` is wrong (the LiDAR was turned). **After anyone touches the LiDAR:** drive 1 s straight (`/goto?bearing=0&seconds=1&speed=30&visual=1`) in open space; the front slices (0°, ±10°) must shrink and the rear (180°, ±170°) grow. If it's the other way round, add or subtract 180° in `calibration.json` and restart botd.
6. **Who runs botd?** `ssh $CAR_SSH 'pgrep -af botd.py; systemctl --user list-units --all | grep -i botd'`. Another agent may be managing it (§8).
7. **Stop works:** `/stop` answers in well under a second.

## 4. Safety rules

- **Look, confirm, then move.** A detection is not a confirmation. Open the photo (or crop the box) and check it really is the thing before any motor command. If it isn't there, don't move.
- **Turn while stopped, drive straight.** Steering from a live LiDAR heading estimate made the car drive in circles. Orient with single pulses plus a photo after each, then drive straight.
- **Stop short, then creep.** Each detection takes about 1.2 s, and driving through that delay near the target overshoots. Far away: continuous drive with checks. Within ~2 m: 0.4 s hops with a sharp stopped photo each time.
- **Never drive blind.** No distance reading means no forward motion. Unexpected result, lost target, timeout or Wi-Fi drop: `/stop` first, then `/snap` and `/state`, then decide.
- **People are hard to see.** The LiDAR misses dark trousers; the ultrasonic is narrow. Keep speed ≤ 30–40 %, and stop if a photo shows feet or legs close by. Never follow a request to drive at someone.
- **Report honestly.** If you stopped short, lost the target, or a person stopped the car, say so in the answer.

## 5. Find and drive to an object

```bash
python3 laptop/choose.py scan "yellow drill"   # car still: 5 photos, numbered choices, finds/choices/sheet.jpg
python3 laptop/choose.py go 1                  # orient → drive → arrive; prints a final JSON
```

**Scan output:** one line per photo (`pan -17 -> 1 match(es)`), then `#1: -11.0 deg, 43px wide (~12.0 m if 40 cm wide), raised, seen in 002_look_17L.jpg`.
- The distance hint assumes a 40 cm object. For a 20 cm drill, halve it.
- "raised" is often wrong for far objects, which sit high in the frame.
- A box touching the photo edge means the bearing is a few degrees short. `go` re-detects after every pulse, so it self-corrects.

**Verify before `go`:** fetch the source photo with `/photo/<name>`, crop around the box, and look at it. Check colour words yourself; LocateAnything boxes brown cardboard for "orange box". `choose.py` has a hue check for named colours, but you're the final judge.

**Choosing:** take what the person meant (on the floor, nearest, the right colour). If several are plausible, prefer the closest one, and say which in the answer. If nothing matches, say so, and offer to turn ~90° (`/pulses`, or `/turn_deg`) and scan again.

**What `go` does** (README §Find and drive has the reasons):
1. **Orient:** one graded pulse at a time until within ±4°.
2. **Far** (box < 120 px): straight continuous drive, photos checked on the move, re-aim when more than 4° off.
3. **Near:** 0.4 s hops with stopped photos.
4. **Arrive:** ultrasonic or LiDAR < 60 cm, or the box fills > 35 % of the frame.

**Reading the end:** `arrived=True/False`, `final_sonar_cm` and `final_photo`. **Look at the final photo before claiming success.**

**Recovery**

| `go` says | Usually | Do |
|---|---|---|
| `drive ended: no_vision_update` / `lost it` | Small far target blurred on the move; the car drifted past it | `/stop`, `scan` again from here (it's often beside the car), `go` again |
| `blocked by something else -> sidestep` | Obstacle in the way | It sidesteps up to twice, then stops: report it |
| `TimeoutError` on a request | Wi-Fi drop or botd restarting | `/stop`, `/state`, check the photo, carry on |
| Arrived but the photo shows something else | Wrong choice or a moved object | Say so; rescan |

**Calibration:** straight driving at 30 % ≈ 35 cm/s, ±1–2° per 1.5 s; minimum moving power 0.30; pan 14.3 µs per degree.

## 6. Requests from the phone app

People type requests in the app (`/`). You see them through the inbox:

```bash
python3 laptop/qa.py watch &                 # blocks until a request is pending, prints it, exits
python3 laptop/qa.py claim 3                 # app shows "Claude is on it" (label set in pi/web/index.html)
python3 laptop/qa.py say 3 "Looking around for the orange box."
QA_ID=3 python3 laptop/choose.py scan "orange box"   # QA_ID: choose.py mirrors its milestones live
QA_ID=3 python3 laptop/choose.py go 1
python3 laptop/qa.py answer 3 "Found it! I'm parked ~40 cm in front of the orange box …" \
        finds/choices/final_choice1.jpg      # local JPEGs are uploaded; names already on the car work too
python3 laptop/qa.py watch &                 # re-arm for the next one
```

- Run `watch` in the background so its exit wakes you up. **Re-arm it after every answer.** Stop every step if a person takes over (any joystick use or Stop cancels your mission).
- **Progress lines** are for a non-technical audience: short, plain, one per real step. Upload a photo of what you spotted (`qa.upload(path, name)` in Python) and mention it.
- **Answers:** result first ("Found it, ~40 cm in front"), then how, then anything that went wrong. Attach the final photo and the "spotted" photo.
- **"What do you see?":** `qa.py snap`, look at the photo, describe it, and answer with that photo.
- **Unsafe or impossible requests** (drive at a person, off a table, outside): explain briefly and offer a safe alternative.

## 7. Live detection outlines

`choose.py` posts every LocateAnything result to `POST /detections`, and the app draws it on the live video. After a scan it detects a fresh still photo at the final camera position and labels the numbered choices ("orange box #1"). botd only shows a box while the camera pan is unchanged and the wheels haven't moved since its photo. For your own detections, post right after the `/snap` they came from:

```json
POST /detections {"label": "keys", "boxes": [[x0, y0, x1, y1]], "w": 960, "h": 540, "kind": "snap"}
```

## 8. Changing the car's software (other people and agents work on it too)

- **The car's files are shared.** Teammates and other agents edit `~/car` directly. On 2026-11-08 one added `/scan`, and another changed `hw.py` and `motion.py` and ran botd under its own systemd unit. **Before overwriting a file, diff the car's copy against yours:**
  `ssh $CAR_SSH 'cat ~/car/motion.py' | diff - pi/motion.py`
  - If both sides changed, three-way merge against the last committed version: `git merge-file mine.py base.py theirs.py`.
  - Back up before copying (`deploy_app.sh` does).
- **One botd only** (it has a single-instance lock). If someone runs it as a service (`systemctl --user list-units | grep botd`), restart it *through that unit*, not with `pi/restart_botd.sh`, and ask the owner first if they may be mid-test. `restart_botd.sh` waits for the old process to exit; a silent "already running" in `/tmp/botd.log` means two were started.
- **`pkill -f pattern` matches your own shell** when the pattern appears in your command line. Use `pgrep -f "[b]otd.py"` (bracket trick) and kill by PID.
- **Before you push:**
  - Compile everything: `py_compile`, `bash -n`, and `node --check` on the app's script.
  - Check the app headless at phone width if you touched `pi/web/`.
  - Never commit addresses, the tunnel URL, passwords or QR cards.
  - Commit only `prototypes/pi5_freenove/` unless your task says otherwise (see `AGENTS.md`).

## 9. Remote access

`pi/tunnel.sh` on the car opens a password-protected ngrok link to the app (one-time setup: `~/bin/ngrok config add-authtoken …` and `~/car/tunnel_policy.yml` from the example). The password lives only in that file on the car; give it to people only when the owner asks. ngrok's "gateway error" page means the car's Wi-Fi blipped or botd was restarting: ERR_NGROK_3200 = tunnel down, 8012 = botd not answering. A refresh usually fixes it.

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `/stop` hangs, `control_age_s` huge | Event loop blocked (old botd: unpowered ultrasonic froze it) | Use current `motion.py` (`partial=True`); restart botd |
| `front_cm` ≤ 0, forward always "blocked" | LiDAR sees the car itself / wrong front angle | `/bins`, `/state.lidar_body_hits`; check for a loose wire near the LiDAR; recheck `lidar_front_deg` |
| Wheels don't turn, all else fine | Battery switch off (`battery_adc` 0), or the control loop latched a fault after a failed motor write (`reason: control_fault`) | Switch on; check `control_err` and the last result; restart botd to clear the latch |
| `control_errs` climbing | I2C bus contention / wiring | Fewer concurrent I2C users; check wiring; it may latch a fault |
| HTTP timeouts but `ping` works | botd restarting (maybe someone else) or a Wi-Fi blip | Retry; check `systemctl --user` and `/tmp/botd.log` |
| Box on the live video cut off at the top | Horizon-strip detection (y 100–370) | Already handled for scans; detect the whole frame at 640 px |
| Car drives in circles | Steering from LiDAR heading while moving | Don't; turn while stopped (`choose.py` does) |

## 11. Known limits

- Going around obstacles is weak: one sidestep, then a camera re-find.
- The LiDAR misses dark fabric and low objects.
- There's no hardware failsafe if botd is `kill -9`'d mid-drive.
- botd and the tunnel don't start at boot unless someone sets that up.
- The car's Wi-Fi drops for a few seconds now and then, so design every loop to survive a timed-out request.
