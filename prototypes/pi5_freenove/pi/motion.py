#!/usr/bin/env python3
"""
motion.py - asyncio motion core: smooth velocity control + obstacle slow-down.

Concurrent jobs:
  lidar   (thread)    RPLIDAR C1 -> clearance in a car-width lane ahead
  sonar   (coroutine) ultrasonic 15 Hz -> clearance straight ahead (catches LOW objects)
  control (coroutine) 20 Hz: target speed -> limit by clearance -> accel/brake limits -> motors
  mission (coroutine) what we're trying to do

CLI:
  python3 motion.py approach [speed%=40] [max_s=6]        drive forward, slow + stop for obstacles
  python3 motion.py approach 40 6 --dry                   same, but motors stay OFF (sensor check)
"""
import asyncio, json, math, os, sys, threading, time, warnings
warnings.filterwarnings("ignore")
from car import Car, _load_cal
from lidar import RPLidar

STOP_CM = 20.0           # hard stop when (predicted) clearance <= this
SLOW_CM = 100.0          # full speed allowed beyond this; linear slow-down in between
LOOKAHEAD_S = 0.25       # brake earlier the faster we close in (latency + stopping distance)
BRAKE_S = 0.15           # active short-brake duration when we have to stop while moving
MIN_DUTY_MOTION = 700    # lower floor than car.py so "slow" is actually slow
HALF_WIDTH_CM = 13.0     # half the car width + margin: lane that must be clear
LIDAR_TO_FRONT_CM = 12.0 # LiDAR centre -> front bumper (estimate)
# the car's own outline around the LiDAR: returns inside it are the car (2026-11-08: 8-14 cm hits at front-left
# made front clearance -4 cm, so forward was always "blocked"). The front sonar still covers the bumper zone.
BODY_FRONT_CM, BODY_REAR_CM, BODY_HALF_CM = LIDAR_TO_FRONT_CM + 1.5, 12.0, 11.5
ACCEL = 0.5              # power units per second when speeding up (gentle: hard starts made the car slip/drift)
DECEL = 3.0              # power units per second when slowing down
HZ = 20
SENSOR_STALE_S = 0.6


class State:
    def __init__(self):
        self.target = 0.0          # desired forward power 0..1
        self.turn = 0.0            # desired steering -1..1 (+ = right)
        self.cmd = 0.0             # actual commanded power after limits
        self.lim = 1.0
        self.lidar_cm = None; self.lidar_t = 0.0; self.lidar_scans = 0
        self.sonar_cm = None; self.sonar_t = 0.0
        self.src = "-"; self.clear = None
        self.running = True
        self.reason = None
        self.log = []


UNKNOWN_CM = 150.0       # a 10-degree slice with no LiDAR returns counts as "this free" (conservative)


def lidar_thread(st, front_deg):
    lid = RPLidar(); lid.start()
    try:
        for scan in lid.scans():
            if not st.running:
                break
            best, rear, body = 999.0, 999.0, 0
            bins = {}                              # car-frame slice centre (deg, + = right) -> nearest cm
            for a, d in scan:
                if d < 80:                         # < 8 cm: self-hits / noise
                    continue
                rel = (a - front_deg + 180) % 360 - 180
                th = math.radians(rel)             # clockwise angles: +y = right
                x, y = d * math.cos(th) / 10.0, d * math.sin(th) / 10.0
                if -BODY_REAR_CM < x < BODY_FRONT_CM and abs(y) < BODY_HALF_CM:
                    body += 1                      # inside the car's own outline: a wire / the chassis, not an obstacle
                    continue
                if x > 0 and abs(y) < HALF_WIDTH_CM:
                    best = min(best, x - LIDAR_TO_FRONT_CM)
                if x < 0 and abs(y) < HALF_WIDTH_CM:
                    rear = min(rear, -x - 12.0)
                b = int(round(rel / 10.0)) * 10
                bins[b] = min(bins.get(b, 9999.0), d / 10.0)
            st.bins = {b: bins.get(b, UNKNOWN_CM) for b in range(-180, 181, 10)}
            st.lidar_cm = best; st.rear_cm = rear; st.body_hits = body
            st.last_scan = scan                    # raw points for scan matching (measured turns)
            st.lidar_t = time.time(); st.lidar_scans += 1
    except Exception as e:
        st.lidar_err = repr(e)
    finally:
        lid.close()


async def sonar_task(st):
    from gpiozero import DistanceSensor
    # partial=True: never block. With partial=False gpiozero waits for a full queue of echoes, so an unpowered
    # or unplugged sensor froze the whole event loop (control loop, STOP, missions) - seen 2026-11-08.
    s = DistanceSensor(echo=22, trigger=27, max_distance=3, queue_len=2, partial=True)
    try:
        while st.running:
            d = s.distance * 100
            if d > 0.5:                     # 0.0 = no echo yet / sensor dead -> no update, reading goes stale
                st.sonar_cm = d if d < 295 else 999.0
                st.sonar_t = time.time()
            await asyncio.sleep(1 / 15)
    finally:
        s.close()


def limit_for(clear, slow_cm=None):
    slow = slow_cm or SLOW_CM
    if clear <= STOP_CM:
        return 0.0
    if clear >= slow:
        return 1.0
    return (clear - STOP_CM) / (slow - STOP_CM)


async def control_task(st, car, dry):
    dt = 1.0 / HZ
    t0 = time.time()
    car.min_duty = MIN_DUTY_MOTION
    prev_clear, prev_t, vclose = None, None, 0.0
    brake_until = 0.0
    st.control_errs = 0; st.control_err = None
    while st.running:
        try:
            st.control_tick = time.time()
            tick = time.time()
            srcs = []
            if tick - st.lidar_t < SENSOR_STALE_S and st.lidar_cm is not None:
                srcs.append(("lidar", st.lidar_cm))
            if tick - st.sonar_t < SENSOR_STALE_S and st.sonar_cm is not None:
                srcs.append(("sonar", st.sonar_cm))
            if srcs:
                st.src, st.clear = min(srcs, key=lambda s: s[1])
                # closing speed (cm/s), smoothed; only meaningful for real readings
                if prev_clear is not None and st.clear < 500 and prev_clear < 500:
                    inst = (prev_clear - st.clear) / max(1e-3, tick - prev_t)
                    vclose = 0.6 * vclose + 0.4 * max(0.0, min(inst, 300.0))
                prev_clear, prev_t = st.clear, tick
                st.vclose = vclose
                st.lim = limit_for(st.clear - vclose * LOOKAHEAD_S, getattr(st, 'slow_cm', None))
            else:
                st.src, st.clear, st.lim = "none", None, 0.0     # blind -> no forward motion

            moving_before = st.cmd > 0
            want = st.target * st.lim
            floor = getattr(st, 'min_power', 0.0) or 0.0
            if st.target > 0 and st.lim > 0 and want < floor:
                want = min(floor, st.target)                  # never creep too weakly to move on this floor
            if want > st.cmd and st.cmd == 0.0 and want >= 0.2:
                st.cmd = want                                 # clean start: both sides break free together (no veer)
            elif want > st.cmd:
                st.cmd = min(want, st.cmd + ACCEL * dt)
            else:
                st.cmd = max(want, st.cmd - DECEL * dt)
            if st.lim == 0.0:
                st.cmd = 0.0                                      # hard stop, no ramp
                if moving_before and st.target > 0:
                    brake_until = tick + BRAKE_S                  # active brake, don't coast
            if st.cmd < 0.02:
                st.cmd = 0.0

            manual = getattr(st, "manual", None)
            if manual and tick < st.manual_until and not dry:
                if manual == "brake":                             # short-brake: both H-bridge inputs on
                    for ch in range(8):
                        car.pwm.set_duty(ch, 4095)
                else:
                    ml, mr = manual                               # mission override (backing up / pivoting)
                    car._apply({"front_left": ml, "back_left": ml, "front_right": mr, "back_right": mr}, 1.0)
                st.log.append((round(tick - t0, 2), round(st.target, 2), round(st.lim, 2), round(st.cmd, 2),
                               "MANUAL", None if st.clear is None else round(st.clear, 1),
                               None if st.lidar_cm is None else round(st.lidar_cm, 1),
                               None if st.sonar_cm is None else round(st.sonar_cm, 1), round(vclose)))
                await asyncio.sleep(max(0.0, dt - (time.time() - tick)))
                continue
            if not dry:
                if tick < brake_until:
                    for ch in range(8):
                        car.pwm.set_duty(ch, 4095)
                    st.braking = True
                else:
                    if getattr(st, "braking", False):
                        car.stop(); st.braking = False
                    k = 0.45 if st.cmd > 0 else 0.0               # steer only while rolling forward
                    l = max(-1.0, min(1.0, st.cmd + st.turn * k))
                    r = max(-1.0, min(1.0, st.cmd - st.turn * k))
                    car._apply({"front_left": l, "back_left": l, "front_right": r, "back_right": r}, 1.0)
            st.log.append((round(tick - t0, 2), round(st.target, 2), round(st.lim, 2), round(st.cmd, 2),
                           ("BRAKE" if tick < brake_until else st.src),
                           None if st.clear is None else round(st.clear, 1),
                           None if st.lidar_cm is None else round(st.lidar_cm, 1),
                           None if st.sonar_cm is None else round(st.sonar_cm, 1),
                           round(vclose)))
            await asyncio.sleep(max(0.0, dt - (time.time() - tick)))
        except Exception as e:                    # never let one bad tick (I2C glitch...) kill control
            st.control_errs += 1; st.control_err = repr(e)
            try:
                car.stop()
            except Exception:
                pass
            await asyncio.sleep(dt)
    car.stop()


async def approach(st, speed, max_s):
    """Mission: drive forward at `speed`, let control slow/stop us; end when held stopped or time up."""
    st.target = speed / 100.0
    t0 = time.time(); held = None
    while time.time() - t0 < max_s:
        await asyncio.sleep(0.05)
        blocked = st.lim == 0.0 or (st.clear is not None and st.clear < STOP_CM + 5 and st.cmd < 0.05)
        if blocked:
            held = held or time.time()
            if time.time() - held > 0.6:
                st.reason = f"arrived_at_obstacle ({st.src} {st.clear:.0f} cm)" if st.clear is not None else "stopped_no_sensor"
                break
        else:
            held = None
    else:
        st.reason = "time_up"
    st.target = 0.0
    await asyncio.sleep(0.6)                                  # let the control loop ramp down


AVOID_CM = 38.0          # start an avoid manoeuvre when front clearance drops below this
STOP_FILE = "/tmp/stop_motion"


def free_bins(st):
    """LiDAR slices plus the ultrasonic folded into the straight-ahead slices."""
    bins = dict(getattr(st, "bins", {}) or {})
    if st.sonar_cm is not None and st.sonar_cm < 200 and time.time() - st.sonar_t < SENSOR_STALE_S:
        for b in (-10, 0, 10):
            bins[b] = min(bins.get(b, 999.0), st.sonar_cm + LIDAR_TO_FRONT_CM)
    return bins


def choose_heading(bins):
    """Most open direction within +-60 deg, biased toward straight ahead. Returns degrees (+ = right)."""
    best, best_score = 0, -1e9
    for b in range(-60, 61, 10):
        # a direction is only as free as its neighbours (car is wider than one slice)
        free = min(bins.get(b + o, 999.0) for o in (-10, 0, 10))
        score = min(free, 300.0) - 1.2 * abs(b)
        if score > best_score:
            best, best_score = b, score
    return best


def side_space(bins, sign):
    return sum(min(bins.get(sign * b, 0.0), 250.0) for b in range(30, 100, 10))


async def hold(st, l, r, secs):
    st.manual = (l, r); st.manual_until = time.time() + secs
    await asyncio.sleep(secs)
    st.manual = None


class Camera:
    """Still photos during missions; each saved with the sensor context in index.jsonl."""
    def __init__(self, outdir, car):
        import os
        from picamera2 import Picamera2
        from libcamera import Transform
        self.os, self.car, self.outdir, self.n = os, car, outdir, 0
        os.makedirs(outdir, exist_ok=True)
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_still_configuration(main={"size": (960, 540)}, transform=Transform()))
        self.cam.start()
        try:   # auto exposure + white balance (true colours), but prefer SHORT exposures: sharp while moving
            from libcamera import controls as _lc
            self.cam.set_controls({"AeExposureMode": _lc.AeExposureModeEnum.Short})
        except Exception:
            pass
        self.lock = threading.Lock()            # missions and HTTP snapshots share one camera

    def capture_bytes(self):
        import io
        buf = io.BytesIO()
        with self.lock:
            self.cam.capture_file(buf, format="jpeg")
        return buf.getvalue()

    def capture(self, tag, meta):
        self.n += 1
        path = f"{self.outdir}/{self.n:03d}_{tag}.jpg"
        with self.lock:
            self.cam.capture_file(path)
        meta["file"] = self.os.path.basename(path)
        with open(f"{self.outdir}/index.jsonl", "a") as f:
            f.write(json.dumps(meta) + "\n")
        return path

    def pan(self, us):
        self.car.pwm.set_pulse_us(8, max(1000, min(2000, int(us))))


PAN_US_PER_DEG = 14.3    # measured 2026-11-08: 220 us of pan = 15.4 deg (camera f = 1290 px @ 960, HFOV 40.8 deg)


def nearest_objects(bins, k=4):
    """Closest LiDAR slices (deg + = right, cm) - context saved with every photo."""
    return sorted(((b, round(d)) for b, d in bins.items() if d < 250), key=lambda x: x[1])[:k]


async def photo(st, tag, events, t0, pause=True, pan_deg=0):
    """Stop (so the shot is sharp), optionally pan the camera toward pan_deg (+ = right), shoot, resume."""
    cam = getattr(st, "cam", None)
    if cam is None:
        return
    saved_target, saved_turn = st.target, st.turn
    if pause:
        st.target = 0.0; st.turn = 0.0
    if pan_deg:
        cam.pan(1500 - pan_deg * PAN_US_PER_DEG)   # measured: 14.3 us per degree; pan > 1500 looks LEFT
    await asyncio.sleep(0.5 if (pause or pan_deg) else 0.05)
    meta = {"t": round(time.time() - t0, 1), "tag": tag, "front_cm": None if st.clear is None else round(st.clear),
            "sonar_cm": None if st.sonar_cm is None else round(st.sonar_cm), "camera_pan_deg": pan_deg,
            "nearest_lidar": nearest_objects(free_bins(st))}
    path = await asyncio.to_thread(cam.capture, tag, meta)
    if pan_deg:
        cam.pan(1500)
        await asyncio.sleep(0.3)
    events.append(f"{meta['t']:5.1f}s PHOTO {tag:12} front={meta['front_cm']}cm -> {path.split('/')[-1]}")
    st.last_photo = time.time()
    if pause:
        st.target, st.turn = saved_target, saved_turn
    return path


async def avoid(st, events, t0):
    """Blocked ahead: brake, back up if room behind, pivot to the open side until clear."""
    st.target = 0.0; st.turn = 0.0
    await asyncio.sleep(0.35)                                 # control loop brakes
    await photo(st, "blocked", events, t0, pause=False)       # what is in our way?
    bins = free_bins(st)
    left, right = side_space(bins, -1), side_space(bins, +1)
    side = 1 if right >= left else -1
    rear = getattr(st, "rear_cm", 999.0)
    events.append(f"{time.time()-t0:5.1f}s AVOID front={st.clear:.0f}cm  left_space={left:.0f} right_space={right:.0f} -> turn {'RIGHT' if side > 0 else 'LEFT'}  rear={rear:.0f}cm")
    if rear > 30:
        await hold(st, -0.35, -0.35, 0.45)                   # back up a little
        await hold(st, 0.0, 0.0, 0.15)
    t_piv = time.time()
    while time.time() - t_piv < 3.0:                          # pivot until the front is clear
        await hold(st, 0.42 * side, -0.42 * side, 0.1)
        if st.clear is not None and st.clear > 75 and (st.sonar_cm or 0) > 60:
            break
    await hold(st, 0.42 * side, -0.42 * side, 0.15)           # a bit extra to clear the corner
    await hold(st, 0.0, 0.0, 0.2)
    events.append(f"{time.time()-t0:5.1f}s   pivoted {time.time()-t_piv:.1f}s, front now {(st.clear if st.clear is not None else -1):.0f}cm")


async def wander(st, speed, max_s):
    """Mission: roam the room - cruise toward open space, avoid obstacles, never stop for long."""
    import os
    events, t0 = [], time.time()
    st.target = speed / 100.0
    last_print = 0
    while time.time() - t0 < max_s:
        if os.path.exists(STOP_FILE):
            events.append(f"{time.time()-t0:5.1f}s STOP FILE -> stopping"); break
        await asyncio.sleep(0.1)
        if st.clear is not None and st.clear < AVOID_CM:
            await avoid(st, events, t0)
            st.target = speed / 100.0
            continue
        # ---- "something interesting" photo triggers ----
        if getattr(st, "cam", None) is not None:
            now = time.time(); since = now - getattr(st, "last_photo", 0)
            bins = free_bins(st)
            side = min(((b, d) for b, d in bins.items() if 40 <= abs(b) <= 90 and d < 60), key=lambda x: x[1], default=None)
            if st.clear is not None and st.clear < 75 and since > 4:
                await photo(st, "approaching", events, t0)
                continue
            if side and since > 6:
                await photo(st, f"side_{'right' if side[0] > 0 else 'left'}", events, t0, pan_deg=max(-45, min(45, side[0])))
                continue
            if since > 15:
                await photo(st, "survey", events, t0)
                continue
        h = choose_heading(free_bins(st))
        st.turn = 0.7 * st.turn + 0.3 * max(-1.0, min(1.0, h / 45.0))
        if time.time() - last_print > 1.0:
            events.append(f"{time.time()-t0:5.1f}s cruise front={(st.clear if st.clear is not None else -1):.0f}cm heading={h:+d}deg turn={st.turn:+.2f} cmd={st.cmd:.2f}")
            last_print = time.time()
    st.target = 0.0; st.turn = 0.0
    await asyncio.sleep(0.6)
    st.reason = st.reason or "time_up"
    st.events = events


async def main(argv):
    mode = argv[0] if argv else "approach"
    speed = float(argv[1]) if len(argv) > 1 else 40
    max_s = min(float(argv[2]) if len(argv) > 2 else 6, 200 if mode == "wander" else 10)
    dry = "--dry" in argv
    car = Car(); cal = _load_cal(); st = State()
    st.cam = None
    if "--photos" in argv:
        outdir = os.path.expanduser("~/car/runs/") + time.strftime("%H%M%S")
        st.cam = Camera(outdir, car)
        car.pwm.set_pulse_us(8, 1500); car.pwm.set_pulse_us(9, 1500)   # camera straight ahead
        print("PHOTO_DIR", outdir, flush=True)
    threading.Thread(target=lidar_thread, args=(st, cal.get("lidar_front_deg", 0.0)), daemon=True).start()
    tasks = [asyncio.create_task(sonar_task(st)), asyncio.create_task(control_task(st, car, dry))]
    t_wait = time.time()
    while time.time() - t_wait < 8 and not (st.lidar_scans >= 2 and st.sonar_t):
        await asyncio.sleep(0.1)
    ready = {"lidar_scans": st.lidar_scans, "sonar_ok": bool(st.sonar_t), "waited_s": round(time.time() - t_wait, 1)}
    import os, signal
    try: os.remove(STOP_FILE)
    except OSError: pass
    me = asyncio.current_task()
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        asyncio.get_running_loop().add_signal_handler(sig, me.cancel)   # killed -> finally -> motors off
    try:
        if mode == "approach":
            await approach(st, speed, max_s)
        elif mode == "wander":
            await wander(st, speed, max_s)
    except asyncio.CancelledError:
        st.reason = "killed"
    finally:
        st.running = False
        await asyncio.sleep(0.2)
        car.stop()
        for t in tasks:
            t.cancel()
    rows = st.log
    if getattr(st, "events", None):
        print("EVENTS:"); print("\n".join(st.events))
        moving = [r for r in rows if r[3] > 0 or r[4] == "MANUAL"]
        print(json.dumps({"mode": mode, "result": st.reason, "drove_s": round(len(moving) / HZ, 1),
                          "avoids": sum("AVOID" in e for e in st.events)}))
        return
    moving = [r for r in rows if r[3] > 0]
    print(json.dumps({"mode": mode, "dry": dry, "speed": speed, "ready": ready, "result": st.reason,
                      "drove_s": round(len(moving) / HZ, 2), "peak_cmd": max((r[3] for r in rows), default=0),
                      "final_clear_cm": rows[-1][5] if rows else None}))
    print("t     tgt  lim  cmd  src    clear  lidar  sonar  vclose")
    last = -1
    for r in rows:
        if r[1] == 0 and r[3] == 0 and r[4] != "BRAKE" and r is not rows[-1]:
            continue                                   # skip idle rows while waiting for sensors
        if r[0] - last >= 0.1 or r[4] == "BRAKE" or r is rows[-1]:
            print(f"{r[0]:5.2f} {r[1]:4.2f} {r[2]:4.2f} {r[3]:4.2f} {r[4]:6} {str(r[5]):>6} {str(r[6]):>6} {str(r[7]):>6} {r[8]:>5}")
            last = r[0]


if __name__ == "__main__":
    import fcntl
    _lock = open("/tmp/motion.lock", "w")
    try:
        fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)      # only one motion.py may run
    except OSError:
        print(json.dumps({"ok": False, "error": "another motion.py is already running"})); sys.exit(1)
    asyncio.run(main(sys.argv[1:]))
