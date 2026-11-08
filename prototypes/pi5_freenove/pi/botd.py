#!/usr/bin/env python3
"""
botd.py - one daemon that owns the car (sensors, 20 Hz motion loop, camera, missions) and
exposes it as tools an LLM - or a person on the web page - can use. Replaces server.py and
the motion.py CLI while it runs (they all need the motors, sonar and camera).

Tools (GET, JSON unless noted)
  /state                                   sensors, mission, recent events
  /stop                                    cancel the mission, motors off
  /wander?seconds=60&speed=40              roam + avoid + event photos (returns at once)
  /wander?head=1&seconds=120               cute wander: camera head sweeps, looks before turning, look-arounds
  /approach?speed=40&seconds=6&hold=1      drive forward until ~20 cm from something; hold=1 keeps the
                                           starting heading using LiDAR scan matching (course correction)
  /back?seconds=0.4                        short reverse (no rear check - keep it short)
  /goto?bearing=-20&seconds=1.5&speed=40   drive toward a direction (+ = right of the car now), steering around
                                           obstacles on the way; LiDAR tracks the car's rotation meanwhile
  /look_around?tilt=1500                   stop, pan camera left->right, 5 sharp photos
  /turn?dir=left|right&seconds=0.3         pivot in place (APPROXIMATE - check with a photo)
  /turn_deg?deg=30                         MEASURED pivot (+ = right): pulses checked by LiDAR scan matching, +-3 deg
  /wait?seconds=30                         block until the current mission ends (or timeout)
  /snap?pan=0&tilt=1500                    JPEG right now (pan degrees, + = right)
  /photos?since=0                          photo log: file, t, tag, sensors
  /photo/<file>                            one photo (JPEG)
  /beep?ms=150
  /scan                                    newest raw LiDAR scan for the dashboard (deg CCW, 0 = front, metres)
  /vision                                  live detection overlay: boxes (0..1) still valid for the current view
  POST /detections {"label","boxes":[[x0,y0,x1,y1]|{"b":[..],"label":".."}],"w","h","kind":"snap"|"look","pan_deg"}
Command listening
  POST /ask      {"q": "..."} or form q=   queue a question
  /inbox?status=pending                    questions (pending | working | answered | all)
  POST /claim    {"id": n}                 mark a question as being worked on
  POST /answer   {"id": n, "text": "...", "photos": ["003_blocked.jpg"]}
  /              web page: ask box, answers with photos, live status, STOP
"""
import asyncio, io, json, os, signal, sys, threading, time, warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
warnings.filterwarnings("ignore")

from car import Car, _load_cal
from hw import ADS7830
from motion import (State, Camera, lidar_thread, sonar_task, control_task, wander, approach,
                    photo, hold, nearest_objects, free_bins)

PORT = 8000
WEB_FILES = {"/manifest.webmanifest": "application/manifest+json", "/sw.js": "text/javascript",
             "/icon-192.png": "image/png", "/icon-512.png": "image/png", "/app.css": "text/css"}
HOME = os.path.expanduser("~/car")
QA_PATH = os.path.join(HOME, "qa.json")
MAX_WANDER_S, MAX_APPROACH_S, MAX_TURN_S = 180, 10, 1.5


class Bot:
    def __init__(self, loop):
        self.loop = loop
        self.car = Car()
        cal = _load_cal()
        self.st = st = State()
        self.outdir = os.path.join(HOME, "runs", time.strftime("%Y%m%d-%H%M%S"))
        st.cam = Camera(self.outdir, self.car)
        self.car.pwm.set_pulse_us(8, 1500); self.car.pwm.set_pulse_us(9, 1500)
        threading.Thread(target=lidar_thread, args=(st, cal.get("lidar_front_deg", 0.0)), daemon=True).start()
        self.tasks = [loop.create_task(sonar_task(st)), loop.create_task(control_task(st, self.car, False))]
        # live detection overlay: remember where the camera points and when the wheels last moved, so a box from
        # the Orin is only drawn on the live video while it still matches what the camera sees
        st.pan_us, st.moved_t = 1500, 0.0
        _set = self.car.pwm.set_pulse_us
        def _tracked(ch, us, _set=_set):
            if ch == 8:
                st.pan_us = int(us)
            return _set(ch, us)
        self.car.pwm.set_pulse_us = _tracked
        self.last_snap = (0.0, 1500)            # (capture time, pan_us) of the newest /snap
        self.det = None                         # newest detections posted by the planner
        self.tasks.append(loop.create_task(self._motion_watch()))
        self.adc = ADS7830(self.car.bus)
        self.mission = None; self.mission_name = None; self.mission_t0 = None
        self.last_result = None
        self.events = []                       # rolling log for /state
        self.t0 = time.time()
        self.qa = self._load_qa()
        self.qa_lock = threading.Lock()
        try:
            from gpiozero import Buzzer
            self.buzzer = Buzzer(17)
        except Exception:
            self.buzzer = None

    # ---------- missions ----------
    def log(self, msg):
        self.events.append(f"{time.time()-self.t0:7.1f}s {msg}")
        del self.events[:-60]

    async def start(self, name, coro_fn):
        await self.cancel()
        self.st.reason = None; self.st.events = []
        self.mission_name, self.mission_t0 = name, time.time()
        self.log(f"START {name}")
        self.mission = asyncio.create_task(self._run(name, coro_fn))
        return {"ok": True, "started": name}

    async def _run(self, name, coro_fn):
        st = self.st
        try:
            res = await coro_fn()
            self.last_result = {"mission": name, "result": st.reason or "done", "detail": res,
                                "events": list(getattr(st, "events", []) or [])[-25:]}
        except asyncio.CancelledError:
            self.last_result = {"mission": name, "result": "cancelled"}
        except Exception as e:                  # a mission bug must never leave the car moving
            self.last_result = {"mission": name, "result": "error", "error": repr(e)}
        finally:
            st.target = 0.0; st.turn = 0.0; st.manual = None
            self.car.stop()
            self.log(f"END {name}: {self.last_result.get('result')}")
            for e in (getattr(st, "events", None) or [])[-10:]:
                self.log("   " + e.strip())
            self.mission_name = None

    async def cancel(self):
        if self.mission and not self.mission.done():
            self.mission.cancel()
            try:
                await self.mission
            except BaseException:
                pass
        self.st.target = 0.0; self.st.turn = 0.0; self.st.manual = None
        self.car.stop()

    async def look_around(self, tilt_us=1500):
        st = self.st
        st.target = 0.0; st.turn = 0.0
        await asyncio.sleep(0.5)                # let the car stop so photos are sharp
        self.car.pwm.set_pulse_us(9, int(tilt_us))
        shots, ev, t0 = [], [], time.time()
        for deg in (-35, -17, 0, 17, 35):       # true degrees, + = right (servo range is ~+-35)
            tag = "look_" + ("ahead" if deg == 0 else f"{abs(deg)}{'R' if deg > 0 else 'L'}")
            path = await photo(st, tag, ev, t0, pause=False, pan_deg=deg if deg else 0)
            shots.append(os.path.basename(path))
        self.car.pwm.set_pulse_us(9, 1500)
        return {"photos": shots}

    async def scan_points(self, k=3, settle=0.45):
        """Merged raw LiDAR points of k fresh scans after the car settled."""
        await asyncio.sleep(settle)
        pts, seen = [], self.st.lidar_scans
        t0 = time.time()
        while len(pts) < k and time.time() - t0 < 3:
            await asyncio.sleep(0.03)
            if self.st.lidar_scans != seen:
                seen = self.st.lidar_scans
                pts.append(list(getattr(self.st, "last_scan", []) or []))
        return [p for sc in pts for p in sc]

    # pivot model from turncal.py (clean pulses, power 0.35, short-brake): |deg| ~= rate*secs + offset.
    # It adapts: every measured pulse nudges the rate toward what THIS floor actually gives.
    TURN_RATE = {"right": (126.0, -3.6), "left": (193.0, -7.2)}

    async def hold_heading(self, stop_ev, steer=True):
        """While driving: track rotation by matching EACH scan to the PREVIOUS one (the car moved only a few cm,
        so the match is accurate) and adding the increments. Steer back if the car drifts (steer=True)."""
        import math
        from scanmatch import to_xy, icp
        st = self.st
        ref = await self.scan_points(k=1, settle=0.0)
        B = to_xy(ref)
        yaw, seen, bad = 0.0, st.lidar_scans, 0
        st.heading_err = 0.0
        while not stop_ev.is_set():
            await asyncio.sleep(0.03)
            if st.lidar_scans == seen:
                continue
            seen = st.lidar_scans
            cur = list(getattr(st, "last_scan", []) or [])
            if len(cur) < 100:
                continue
            A = to_xy(cur)
            phi, t, rms, frac = await asyncio.to_thread(icp, B, A, 0.0, (0.0, 0.0), 12)
            if frac < 0.4 or abs(math.degrees(phi)) > 25:      # implausible for ~0.1 s: skip, keep reference
                bad += 1
                if bad > 5:
                    B = A; bad = 0                              # re-anchor after repeated bad matches
                continue
            bad = 0
            yaw += -math.degrees(phi)                           # + = turned right
            B = A                                               # next increment from here
            st.heading_err = yaw
            if steer:
                st.turn = max(-0.4, min(0.4, -0.04 * yaw))      # drifted right (+) -> steer left (-)

    async def approach_hold(self, speed, secs):
        stop_ev = asyncio.Event()
        hh = asyncio.create_task(self.hold_heading(stop_ev))
        try:
            await approach(self.st, speed, secs)
        finally:
            stop_ev.set()
            try:
                await asyncio.wait_for(hh, 2)
            except BaseException:
                hh.cancel()
            self.st.turn = 0.0
        return {"heading_drift_corrected_deg": round(getattr(self.st, 'heading_err', 0.0), 1)}

    async def goto_bearing(self, bearing, secs, speed=30, aim_timeout=None, visual=False):
        """Commit: drive AT `bearing` (deg, + = right of the car at the start). LiDAR tracks rotation so the
        car keeps aiming at the target. Full speed until the last ~45 cm (power never drops below what
        moves the car). Blocked by something: one decisive wander-style move, then aim again."""
        st = self.st
        stop_ev = asyncio.Event()
        yaw_task = None if visual else asyncio.create_task(self.hold_heading(stop_ev, steer=False))   # only measures yaw
        st.heading_err = 0.0
        st.slow_cm, st.min_power = 45.0, 0.30          # 0.25 let the weaker left side stall -> pivot left
        st.goto_target, st.goto_aim_t = float(bearing), time.time()       # /aim can update it while driving
        t0, avoids = time.time(), 0
        st.target = speed / 100.0
        try:
            while time.time() - t0 < secs:
                await asyncio.sleep(0.1)
                if aim_timeout and time.time() - st.goto_aim_t > aim_timeout:
                    st.reason = "no_vision_update"; break                      # watchdog: vision went quiet -> stop
                if visual:                                                     # steer from photos only:
                    rel = st.goto_target                                       # each /aim sets a correction that
                    st.goto_target *= 0.85                                     # fades as the car turns toward it
                    st.turn = max(-0.4, min(0.4, rel / 50.0)) + self.TRIM          # + fixed trim cancels the car's pull
                else:
                    rel = st.goto_target - (getattr(st, "heading_err", 0.0) or 0.0)  # target vs the car NOW
                    st.turn = max(-0.5, min(0.5, rel / 40.0))                       # gentle steering straight at it
                if st.lim == 0.0 and st.clear is not None:                     # hard-blocked (~20 cm)
                    if abs(rel) < 12:
                        st.reason = "arrived_at_obstacle"; break               # most likely the target itself
                    avoids += 1
                    if avoids > 2:
                        st.reason = "blocked"; break
                    bins = free_bins(st)
                    left = sum(min(bins.get(-b, 0.0), 200.0) for b in range(30, 100, 10))
                    right = sum(min(bins.get(b, 0.0), 200.0) for b in range(30, 100, 10))
                    sgn = 1 if right >= left else -1
                    st.target = 0.0
                    await hold(st, -0.38, -0.38, 0.4)                          # back up
                    await hold(st, 0.45 * sgn, -0.45 * sgn, 0.3)                # decisive pivot to the open side
                    st.target = speed / 100.0
        finally:
            st.target = 0.0; st.turn = 0.0
            await asyncio.sleep(0.35)
            st.slow_cm, st.min_power = None, 0.0
            stop_ev.set()
            if yaw_task:
                try:
                    await asyncio.wait_for(yaw_task, 2)
                except BaseException:
                    yaw_task.cancel()
        yaw = 0.0 if visual else round(getattr(st, "heading_err", 0.0) or 0.0, 1)
        return {"rotated_deg": yaw, "target_rel_end": round(st.goto_target - yaw, 1), "stopped": st.reason or "time"}

    async def back(self, secs):
        await hold(self.st, -0.35, -0.35, secs)
        self.st.manual = "brake"; self.st.manual_until = time.time() + 0.1
        await asyncio.sleep(0.1); self.st.manual = None
        return {"backed_s": secs}

    async def turn_deg(self, target, power=0.35, tol=3.0, max_pulses=16):
        from scanmatch import match
        st = self.st
        st.target = 0.0; st.turn = 0.0
        prev = await self.scan_points()
        total, steps = 0.0, []
        for _ in range(max_pulses):
            rem = target - total
            if abs(rem) <= tol:
                break
            d = "right" if rem > 0 else "left"
            rate, off = self.TURN_RATE[d]
            secs = max(0.05, min((abs(rem) * 0.85 - off) / rate, 0.12))   # small pulses: matching stays reliable
            sgn = 1 if d == "right" else -1
            await hold(st, power * sgn, -power * sgn, secs)
            st.manual = "brake"; st.manual_until = time.time() + 0.1
            await asyncio.sleep(0.1); st.manual = None
            cur = await self.scan_points(k=2, settle=0.35)
            guess = sgn * max(0.0, rate * secs + off)
            m = await asyncio.to_thread(match, prev, cur, guess, 20, 5)
            trusted = m["inliers"] >= 0.5 and abs(m["deg_right"] - guess) < 15
            step = m["deg_right"] if trusted else guess              # bad match: use the floor model for this pulse
            total += step; prev = cur
            got = abs(m["deg_right"])
            if trusted and got > 1.5:                               # learn this floor
                implied = (got - off) / secs
                self.TURN_RATE[d] = (round(0.7 * rate + 0.3 * implied, 1), off)
            steps.append({"pulse_s": round(secs, 3), "dir": d, "measured": round(m["deg_right"], 1), "used": round(step, 1), "trusted": trusted,
                          "rms_mm": m["rms_mm"], "inliers": m["inliers"]})
        return {"target": target, "achieved": round(total, 1), "error": round(total - target, 1), "steps": steps,
                "learned_rates": dict(self.TURN_RATE)}

    DEG_PER_PULSE = 8.0                      # 0.08 s braked pulse; updated by /pulses?measure=1
    TRIM = 0.0                               # constant steering bias that makes 'straight' straight (/drift)

    async def drift_test(self, secs=1.2, speed=30):
        """Drive 'straight' (photos off, trim on) and measure the rotation with STATIONARY scans before/after."""
        from scanmatch import match
        pts0 = await self.scan_points(k=2)
        await self.goto_bearing(0.0, secs, speed, None, True)
        pts1 = await self.scan_points(k=2, settle=0.5)
        m = await asyncio.to_thread(match, pts0, pts1, 0.0, 40, 5)
        return {"trim": self.TRIM, "secs": secs, "rotated_deg": round(m["deg_right"], 1),
                "moved_m": m["shift_m"], "inliers": m["inliers"]}

    async def pulses(self, direction, n, measure=False, secs=0.08):
        from scanmatch import match
        pts0 = await self.scan_points(k=2) if measure else None
        for _ in range(int(n)):
            await self.turn(direction, secs)
        out = {"dir": direction, "pulses": int(n), "deg_per_pulse": self.DEG_PER_PULSE}
        if measure:
            pts1 = await self.scan_points(k=2)
            sgn = 1 if direction == "right" else -1
            guess = sgn * n * self.DEG_PER_PULSE
            m = await asyncio.to_thread(match, pts0, pts1, guess, 25, 5)
            out["measured_deg"] = round(m["deg_right"], 1); out["inliers"] = m["inliers"]
            per = abs(m["deg_right"]) / max(1, n)
            if m["inliers"] >= 0.5 and 2.0 < per < 25.0:
                self.DEG_PER_PULSE = round(0.5 * self.DEG_PER_PULSE + 0.5 * per, 2)
                out["deg_per_pulse"] = self.DEG_PER_PULSE
        return out

    async def turn(self, direction, secs, power=0.36):
        sgn = 1 if direction == "right" else -1
        await hold(self.st, power * sgn, -power * sgn, secs)    # left side + / right side - = turn right
        self.st.manual = "brake"; self.st.manual_until = time.time() + 0.12   # brake: no coasting round
        await asyncio.sleep(0.12); self.st.manual = None
        await hold(self.st, 0.0, 0.0, 0.25)
        return {"turned": direction, "seconds": secs, "note": "approximate; verify with /snap"}

    def scan_json(self):
        """Newest scan, shaped like the Orin's /scan.json (valid points only). The RPLIDAR reports
        clockwise angles; convert to the package frame (counter-clockwise, 0 = car front)."""
        st = self.st
        scan = list(getattr(st, "last_scan", None) or [])
        front = self.car.cal.get("lidar_front_deg", 0.0)
        a, r = [], []
        for ang, d in scan:
            if d < 80:                                       # < 8 cm: the car's own body / noise
                continue
            cw = (ang - front + 180) % 360 - 180             # car frame, clockwise (+ = right)
            a.append(round(-cw, 2) + 0.0); r.append(round(d / 1000.0, 3))
        return {"seq": st.lidar_scans or None, "stamp_s": getattr(st, "lidar_t", None), "n_beams": len(scan),
                "n_valid": len(a), "a": a, "r": r, "pose": None, "scans": st.lidar_scans,
                "front_deg_setting": front}

    # ---------- state ----------
    async def _motion_watch(self):
        st = self.st
        while True:
            now = time.time()
            man = getattr(st, "manual", None)
            if abs(st.cmd) > 0.02 or (man and man != (0, 0) and now < getattr(st, "manual_until", 0)):
                st.moved_t = now
            await asyncio.sleep(0.05)

    def post_detections(self, data):
        """Boxes from the Orin for the live overlay. kind "snap": they belong to the newest /snap photo;
        kind "look": a look_around photo taken at pan_deg while the car stood still."""
        w, h = float(data.get("w") or 960), float(data.get("h") or 540)
        boxes = []
        for bx in (data.get("boxes") or [])[:8]:
            b, lab = (bx.get("b"), bx.get("label")) if isinstance(bx, dict) else (bx, None)
            x0, y0, x1, y1 = [float(v) for v in b]
            boxes.append({"b": [round(max(0, x0 / w), 4), round(max(0, y0 / h), 4), round(min(1, x1 / w), 4),
                                round(min(1, y1 / h), 4)], "label": str(lab or data.get("label") or "")[:40]})
        now = time.time()
        if data.get("kind") == "look":
            cap_t, pan_us = now, int(1500 - float(data.get("pan_deg", 0)) * 14.3)
        else:
            cap_t, pan_us = self.last_snap
        self.det = {"label": str(data.get("label") or "")[:40], "boxes": boxes, "cap_t": cap_t, "pan_us": pan_us,
                    "post_t": now}
        return {"ok": True, "n": len(boxes)}

    def vision(self):
        d, st, now = self.det, self.st, time.time()
        if not d:
            return {"show": False, "boxes": []}
        same_view = abs(st.pan_us - d["pan_us"]) < 8 and st.moved_t <= d["cap_t"]
        fresh = now - d["post_t"] < 1.4 and abs(st.pan_us - d["pan_us"]) < 8   # taken on the move: show briefly
        show = bool(d["boxes"]) and (same_view and now - d["cap_t"] < 20 or fresh)
        return {"show": show, "label": d["label"], "boxes": d["boxes"] if show else [],
                "age_s": round(now - d["cap_t"], 1), "moving_since": not same_view}

    def state(self):
        st = self.st
        r = lambda v: None if v is None else round(v, 1)
        try:
            batt = self.adc.raw(2)
        except OSError:
            batt = None
        return {
            "mission": self.mission_name,
            "control_age_s": round(time.time() - getattr(st, "control_tick", 0), 2),
            "control_errs": getattr(st, "control_errs", 0), "control_err": getattr(st, "control_err", None),
            "mission_s": None if not self.mission_name else round(time.time() - self.mission_t0, 1),
            "last_result": self.last_result,
            "front_cm": r(st.clear), "front_source": st.src, "sonar_cm": r(st.sonar_cm),
            "lidar_lane_cm": r(st.lidar_cm), "rear_cm": r(getattr(st, "rear_cm", None)),
            "lidar_body_hits": getattr(st, "body_hits", None), "pan_us": getattr(st, "pan_us", None),
            "nearest_lidar_deg_cm": nearest_objects(free_bins(st), k=6),
            "speed_cmd": round(st.cmd, 2), "turn": round(st.turn, 2),
            "heading_err_deg": round(getattr(st, "heading_err", 0.0) or 0.0, 1), "turn_rates": dict(self.TURN_RATE),
            "battery_adc": batt, "photos": st.cam.n, "photo_dir": self.outdir,
            "pending_questions": sum(q["status"] == "pending" for q in self.qa),
            "events": self.events[-15:],
        }

    def photos(self, since=0):
        out = []
        try:
            with open(os.path.join(self.outdir, "index.jsonl")) as f:
                for i, line in enumerate(f):
                    if i >= since:
                        out.append(json.loads(line))
        except OSError:
            pass
        return out

    # ---------- questions ----------
    def _load_qa(self):
        try:
            with open(QA_PATH) as f:
                return json.load(f)
        except (OSError, ValueError):
            return []

    def _save_qa(self):
        tmp = QA_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.qa, f, indent=1)
        os.replace(tmp, QA_PATH)

    def ask(self, q):
        q = " ".join(str(q).split())[:300]
        if not q:
            return {"ok": False, "error": "empty question"}
        with self.qa_lock:
            item = {"id": len(self.qa) + 1, "q": q, "status": "pending", "asked": time.strftime("%H:%M:%S"),
                    "answer": None, "photos": []}
            self.qa.append(item); self._save_qa()
        self.log(f"QUESTION #{item['id']}: {q}")
        return {"ok": True, "id": item["id"]}

    def set_q(self, qid, **kw):
        with self.qa_lock:
            for item in self.qa:
                if item["id"] == int(qid):
                    item.update(kw); self._save_qa()
                    return {"ok": True, "question": item}
        return {"ok": False, "error": "no such question"}


PAGE = """<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Locate Bot</title><style>
:root{--bg:#f6f6f4;--fg:#1d1d1b;--mut:#6b6b66;--card:#fff;--line:#deded8;--acc:#1f6feb;--stop:#d1242f}
@media (prefers-color-scheme:dark){:root{--bg:#141414;--fg:#ececea;--mut:#9a9a94;--card:#1e1e1e;--line:#333;--acc:#4c8dff;--stop:#ff4d57}}
body{margin:0;font:16px/1.4 system-ui,sans-serif;background:var(--bg);color:var(--fg)}
main{max-width:760px;margin:auto;padding:12px 16px 80px}
#stop{position:sticky;top:0;z-index:2;width:100%;padding:14px;font-size:20px;font-weight:700;border:0;border-radius:10px;background:var(--stop);color:#fff}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;margin:12px 0}
form{display:flex;gap:8px}input{flex:1;padding:10px;font-size:16px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--fg)}
button{padding:10px 14px;border:1px solid var(--line);border-radius:8px;background:var(--card);color:var(--fg);font-size:15px}
button.pri{background:var(--acc);color:#fff;border:0}.row{display:flex;flex-wrap:wrap;gap:8px}
.mut{color:var(--mut);font-size:13px}.q{font-weight:600}.imgs img{width:31%;margin:1%;border-radius:6px}
pre{white-space:pre-wrap;font-size:12px;color:var(--mut);margin:0}
</style></head><body><main>
<button id=stop onclick="api('/stop')">STOP</button>
<div class=card><form onsubmit="ask(event)"><input id=q placeholder="Ask the bot... e.g. where is the red bucket?" autocomplete=off><button class=pri>Ask</button></form>
<div class=mut>Questions go to the LLM controlling the bot. It drives, looks, and answers here.</div></div>
<div class=card><div class=row><button onclick="api('/wander?seconds=60')">Wander 60 s</button><button onclick="api('/look_around')">Look around</button><button onclick="api('/approach')">Approach</button><button onclick="api('/turn?dir=left')">Turn L</button><button onclick="api('/turn?dir=right')">Turn R</button></div>
<div id=st class=mut style="margin-top:8px">connecting...</div></div>
<div id=qa></div><div class=card><pre id=ev></pre></div>
</main><script>
async function api(p,opt){try{const r=await fetch(p,opt);return await r.json()}catch(e){return {error:String(e)}}}
async function ask(e){e.preventDefault();const q=document.getElementById('q');if(!q.value.trim())return;
await api('/ask',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({q:q.value})});q.value='';load()}
function esc(s){return String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
async function load(){const s=await api('/state');if(!s.error){
document.getElementById('st').textContent=`mission: ${s.mission||'idle'}${s.mission_s?' ('+s.mission_s+' s)':''} | front ${s.front_cm??'?'} cm (${s.front_source}) | battery ${s.battery_adc} | photos ${s.photos}`;
document.getElementById('ev').textContent=(s.events||[]).slice().reverse().join('\\n')}
const qa=await api('/inbox?status=all');if(Array.isArray(qa)){document.getElementById('qa').innerHTML=qa.slice().reverse().map(x=>
`<div class=card><div class=q>#${x.id} ${esc(x.q)}</div><div class=mut>${esc(x.asked)} - ${esc(x.status)}</div>${x.answer?`<p>${esc(x.answer)}</p>`:''}<div class=imgs>${(x.photos||[]).map(f=>`<a href="/photo/${encodeURIComponent(f)}"><img src="/photo/${encodeURIComponent(f)}"></a>`).join('')}</div></div>`).join('')}}
load();setInterval(load,1500);
</script></body></html>"""


class FrameHub:
    """One camera grabber shared by every /stream viewer; it runs only while someone is watching."""
    def __init__(self, cam, period=0.16):
        self.cam, self.period = cam, period
        self.jpg, self.t, self.viewers, self.running = None, 0.0, 0, False
        self.cv = threading.Condition()
        self.small, self.small_t, self.small_lock = None, None, threading.Lock()

    def join(self):
        with self.cv:
            self.viewers += 1
            if not self.running:
                self.running = True
                threading.Thread(target=self._run, daemon=True).start()

    def leave(self):
        with self.cv:
            self.viewers -= 1

    def wait(self, last_t, timeout=2.0):
        with self.cv:
            self.cv.wait_for(lambda: self.t > last_t, timeout=timeout)
            return (self.jpg, self.t) if self.t > last_t else (None, last_t)

    def small_for(self, jpg, t):
        """480x270 low-quality copy of a frame (shared by all remote viewers): ~1/8 of the bytes."""
        with self.small_lock:
            if self.small_t != t:
                from PIL import Image
                im = Image.open(io.BytesIO(jpg)); im.draft("RGB", (480, 270))     # JPEG decodes at 1/2 scale: fast
                im = im.convert("RGB")
                if im.size != (480, 270):
                    im = im.resize((480, 270))
                buf = io.BytesIO(); im.save(buf, "JPEG", quality=55)
                self.small, self.small_t = buf.getvalue(), t
            return self.small

    def _run(self):
        while True:
            with self.cv:
                if self.viewers <= 0:
                    self.running = False
                    return
            t0 = time.time()
            try:
                jpg = self.cam.capture_bytes()
            except Exception:
                time.sleep(0.5); continue
            with self.cv:
                self.jpg, self.t = jpg, time.time()
                self.cv.notify_all()
            time.sleep(max(0.0, self.period - (time.time() - t0)))


class Handler(BaseHTTPRequestHandler):
    bot = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, (bytes, bytearray)):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _call(self, coro, timeout=40):
        return asyncio.run_coroutine_threadsafe(coro, self.bot.loop).result(timeout)

    def do_GET(self):
        b = self.bot
        u = urlparse(self.path); q = {k: v[0] for k, v in parse_qs(u.query).items()}
        f = lambda k, d: float(q.get(k, d))
        try:
            p = u.path
            if p.startswith("/fonts/") and p.endswith(".woff2"):     # self-hosted fonts: the app works offline
                path = os.path.join(HOME, "web", "fonts", os.path.basename(p))
                if os.path.isfile(path):
                    with open(path, "rb") as fh:
                        return self._send(200, fh.read(), "font/woff2")
                return self._send(404, {"ok": False})
            if p == "/" or p in WEB_FILES:
                name = "index.html" if p == "/" else p.lstrip("/")
                path = os.path.join(HOME, "web", name)
                if os.path.isfile(path):
                    with open(path, "rb") as fh:
                        return self._send(200, fh.read(), WEB_FILES.get(p, "text/html; charset=utf-8"))
                if p == "/":
                    return self._send(200, PAGE.encode(), "text/html; charset=utf-8")
                return self._send(404, {"ok": False})
            if p == "/stream":                     # MJPEG live video (~6 fps); ends when the viewer leaves
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                # through the internet tunnel (ngrok sets X-Forwarded-For) send small frames at ~3 fps: the full
                # 960 px stream is ~760 KB/s per viewer, too much for a tunnel's data allowance
                small = q.get("size") == "small" or (bool(self.headers.get("X-Forwarded-For")) and q.get("size") != "full")
                hub = Handler.frames; hub.join(); last = 0.0; sent = 0.0
                try:
                    while True:
                        jpg, last = hub.wait(last)
                        if jpg is None:
                            continue
                        if small:
                            if last - sent < 0.3:
                                continue
                            jpg = hub.small_for(jpg, last); sent = last
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
                finally:
                    hub.leave()
            if p == "/manual":                     # hold-to-drive: each call drives ~0.35 s, then auto-stop
                l = max(-0.55, min(0.55, f("l", 0))); r = max(-0.55, min(0.55, f("r", 0)))
                if b.mission_name:                 # a person driving overrides any running mission
                    self._call(b.cancel()); b.log("manual control took over")
                fwd = l > 0 and r > 0
                if fwd and b.st.clear is None:              # no LiDAR and no sonar reading: never drive blind
                    b.st.manual = None
                    return self._send(200, {"ok": False, "blocked": True, "front_cm": None, "why": "no_sensor"})
                clear = b.st.clear if b.st.clear is not None else 999
                if fwd and clear < 20 + 50 * max(l, r):      # faster -> stop further away (40 cm at 40 %)
                    b.st.manual = None
                    return self._send(200, {"ok": False, "blocked": True, "front_cm": round(clear)})
                b.st.manual = (l, r); b.st.manual_until = time.time() + 0.35
                return self._send(200, {"ok": True, "front_cm": round(clear)})
            if p == "/pan":
                deg = max(-35.0, min(35.0, f("deg", 0)))
                b.car.pwm.set_pulse_us(8, int(1500 - deg * 14.3))
                return self._send(200, {"ok": True, "pan_deg": deg})
            if p == "/state":
                return self._send(200, b.state())
            if p == "/stop":                      # cut the motors directly first: works even if the loop is stuck
                b.st.manual = None; b.st.target = 0.0; b.st.turn = 0.0
                b.car.stop()
                try:
                    self._call(b.cancel(), timeout=3)
                except Exception:
                    b.log("STOP: control loop did not answer (motors were cut directly)")
                    return self._send(200, {"ok": True, "stopped": True, "loop_ok": False})
                b.log("STOP (operator)")
                return self._send(200, {"ok": True, "stopped": True})
            if p == "/wander":                     # head=1: cute mode (camera head sweeps, looks before turning)
                cute = q.get("head") == "1"
                secs, spd = min(f("seconds", 60), MAX_WANDER_S), min(f("speed", 34 if cute else 40), 40 if cute else 60)
                return self._send(200, self._call(b.start("wander", lambda: wander(b.st, spd, secs, cute=cute))))
            if p == "/approach":
                secs, spd = min(f("seconds", 6), MAX_APPROACH_S), min(f("speed", 40), 60)
                if q.get("hold", "1") != "0":
                    return self._send(200, self._call(b.start("approach", lambda: b.approach_hold(spd, secs))))
                return self._send(200, self._call(b.start("approach", lambda: approach(b.st, spd, secs))))
            if p == "/goto":
                brg = max(-90.0, min(90.0, f("bearing", 0))); secs = min(f("seconds", 1.5), 20); spd = min(f("speed", 30), 55)
                wd = f("watchdog", 0) or None; vis = q.get("visual") == "1"
                self._call(b.start("goto", lambda: b.goto_bearing(brg, secs, spd, wd, vis)))
                if q.get("wait", "1") == "0":
                    return self._send(200, {"ok": True, "started": "goto", "continuous": True})
                return self._send(200, self._call(self._wait(secs + 6), timeout=secs + 10))
            if p == "/aim":                     # update the target direction while /goto is driving
                rel = max(-90.0, min(90.0, f("rel", 0)))
                if b.mission_name == "goto" and q.get("keep") == "1":   # missed a photo: keep the aim, feed watchdog
                    b.st.goto_aim_t = time.time()
                    return self._send(200, {"ok": True, "kept": round(b.st.goto_target, 1)})
                if b.mission_name != "goto":
                    return self._send(200, {"ok": False, "error": "not driving", "last_result": b.last_result})
                b.st.goto_target = rel if q.get("visual") == "1" else rel + (getattr(b.st, "heading_err", 0.0) or 0.0)
                b.st.goto_aim_t = time.time()
                return self._send(200, {"ok": True, "target": round(b.st.goto_target, 1)})
            if p == "/back":
                secs = max(0.1, min(f("seconds", 0.4), 0.8))
                self._call(b.start("back", lambda: b.back(secs)))
                return self._send(200, self._call(self._wait(5), timeout=8))
            if p == "/look_around":
                tilt = max(1000, min(2000, int(f("tilt", 1500))))
                self._call(b.start("look_around", lambda: b.look_around(tilt)))
                res = self._call(self._wait(25), timeout=30)
                return self._send(200, res)
            if p == "/turn":
                d = q.get("dir", "left")
                if d not in ("left", "right"):
                    return self._send(400, {"ok": False, "error": "dir must be left or right"})
                secs = max(0.05, min(f("seconds", 0.3), MAX_TURN_S))
                self._call(b.start("turn", lambda: b.turn(d, secs)))
                return self._send(200, self._call(self._wait(5), timeout=8))
            if p == "/scan":                       # newest raw LiDAR scan for the dashboard radar, in the
                return self._send(200, b.scan_json())  # PACKAGE frame: degrees CCW (+ = left), 0 = car front, metres
            if p == "/vision":                     # live detection overlay for the app (boxes 0..1 of the frame)
                return self._send(200, b.vision())
            if p == "/bins":                       # every 10-deg LiDAR slice (car frame, + = right): nearest cm
                bins = getattr(b.st, "bins", {}) or {}
                return self._send(200, {"front_deg_setting": b.car.cal.get("lidar_front_deg"),
                                        "bins": {str(k): round(v) for k, v in sorted(bins.items())}})
            if p == "/wheel_gain":                 # e.g. /wheel_gain?front_right=0.9&back_right=0.9 (saved)
                g = dict(b.car.cal.get("wheel_gain") or {})
                for w in ("front_left", "back_left", "front_right", "back_right"):
                    if w in q:
                        g[w] = max(0.5, min(1.3, f(w, 1.0)))
                b.car.cal["wheel_gain"] = g
                with open(os.path.join(HOME, "calibration.json"), "w") as fh:
                    json.dump(b.car.cal, fh, indent=2)
                return self._send(200, {"ok": True, "wheel_gain": g})
            if p == "/drift":
                if "trim" in q:
                    b.TRIM = max(-0.4, min(0.4, f("trim", 0)))
                if q.get("test", "1") == "1":
                    self._call(b.start("drift", lambda: b.drift_test(min(f("seconds", 1.2), 2.5), min(f("speed", 30), 45))))
                    return self._send(200, self._call(self._wait(15), timeout=20))
                return self._send(200, {"ok": True, "trim": b.TRIM})
            if p == "/pulses":
                d = q.get("dir", "left")
                if d not in ("left", "right"):
                    return self._send(400, {"ok": False, "error": "dir must be left or right"})
                n = max(1, min(40, int(f("n", 1)))); meas = q.get("measure") == "1"
                ps = max(0.03, min(0.12, f("secs", 0.08)))
                self._call(b.start("pulses", lambda: b.pulses(d, n, meas, ps)))
                return self._send(200, self._call(self._wait(60), timeout=70))
            if p == "/turn_deg":
                deg = max(-180.0, min(180.0, f("deg", 0)))
                self._call(b.start("turn_deg", lambda: b.turn_deg(deg)))
                return self._send(200, self._call(self._wait(40), timeout=45))
            if p == "/wait":
                return self._send(200, self._call(self._wait(min(f("seconds", 30), 200)), timeout=210))
            if p == "/snap" and q.get("fast") == "1":       # no servo move, no settle wait (while driving)
                t_cap = time.time(); jpg = b.st.cam.capture_bytes(); b.last_snap = (t_cap, b.st.pan_us)
                return self._send(200, jpg, "image/jpeg")
            if p == "/snap":
                pan_deg = f("pan", 0); tilt = int(f("tilt", 1500))
                b.car.pwm.set_pulse_us(8, max(1000, min(2000, int(1500 - pan_deg * 14.3))))
                b.car.pwm.set_pulse_us(9, max(1000, min(2000, tilt)))
                time.sleep(0.6)
                t_cap = time.time(); jpg = b.st.cam.capture_bytes(); b.last_snap = (t_cap, b.st.pan_us)
                return self._send(200, jpg, "image/jpeg")
            if p == "/photos":
                return self._send(200, b.photos(int(f("since", 0))))
            if p.startswith("/photo/"):
                name = os.path.basename(p[len("/photo/"):])
                path = os.path.join(b.outdir, name)
                if not (name.endswith(".jpg") and os.path.isfile(path)):
                    return self._send(404, {"ok": False, "error": "no such photo"})
                with open(path, "rb") as fh:
                    return self._send(200, fh.read(), "image/jpeg")
            if p == "/inbox":
                status = q.get("status", "pending")
                items = b.qa if status == "all" else [x for x in b.qa if x["status"] == status]
                return self._send(200, items)
            if p == "/beep":
                if b.buzzer:
                    b.buzzer.on(); time.sleep(max(0.02, min(f("ms", 150), 1000)) / 1000); b.buzzer.off()
                return self._send(200, {"ok": bool(b.buzzer)})
            return self._send(404, {"ok": False, "error": "unknown path", "doc": __doc__})
        except (BrokenPipeError, ConnectionResetError):
            return                              # client went away: not a reason to stop the car
        except Exception as e:
            try:
                self._call(b.cancel())
            except Exception:
                b.car.stop()
            return self._send(500, {"ok": False, "error": repr(e)})

    def do_POST(self):
        b = self.bot
        if urlparse(self.path).path == "/upload":     # raw JPEG body -> saved next to the run photos
            q = {k: v[0] for k, v in parse_qs(urlparse(self.path).query).items()}
            name = os.path.basename(q.get("name", "upload_%d.jpg" % int(time.time() * 1000)))
            if not name.endswith(".jpg"):
                name += ".jpg"
            n = int(self.headers.get("Content-Length") or 0)
            if not 0 < n <= 6_000_000:
                return self._send(400, {"ok": False, "error": "need a JPEG body < 6 MB"})
            with open(os.path.join(b.outdir, name), "wb") as fh:
                fh.write(self.rfile.read(n))
            return self._send(200, {"ok": True, "name": name})
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(min(n, 65536)).decode("utf-8", "replace")
        if "json" in (self.headers.get("Content-Type") or ""):
            try:
                data = json.loads(raw or "{}")
            except ValueError:
                return self._send(400, {"ok": False, "error": "bad json"})
        else:
            data = {k: v[0] for k, v in parse_qs(raw).items()}
        p = urlparse(self.path).path
        if p == "/ask":
            return self._send(200, b.ask(data.get("q", "")))
        if p == "/claim":
            return self._send(200, b.set_q(data.get("id", 0), status="working"))
        if p == "/detections":                     # boxes from the Orin -> drawn on the live video
            return self._send(200, b.post_detections(data))
        if p == "/progress":                       # live progress line on a question (shown in the app)
            with b.qa_lock:
                for item in b.qa:
                    if item["id"] == int(data.get("id", 0)):
                        item.setdefault("progress", []).append(time.strftime("%H:%M:%S ") + str(data.get("text", ""))[:300])
                        item["status"] = "working"
                        b._save_qa()
                        return self._send(200, {"ok": True})
            return self._send(404, {"ok": False, "error": "no such question"})
        if p == "/answer":
            photos = [os.path.basename(x) for x in (data.get("photos") or [])][:6]
            return self._send(200, b.set_q(data.get("id", 0), status="answered",
                                           answer=str(data.get("text", ""))[:2000], photos=photos))
        return self._send(404, {"ok": False, "error": "unknown path"})

    async def _wait(self, timeout):
        b = self.bot; t0 = time.time()
        while b.mission_name and time.time() - t0 < timeout:
            await asyncio.sleep(0.2)
        return {"ok": True, "running": b.mission_name, "last_result": b.last_result}


async def main():
    loop = asyncio.get_running_loop()
    bot = Bot(loop)
    Handler.bot = bot
    Handler.frames = FrameHub(bot.st.cam)
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    stop_ev = asyncio.Event()
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        loop.add_signal_handler(sig, stop_ev.set)
    bot.log(f"botd up on :{PORT}, photos in {bot.outdir}")
    print(f"botd up on :{PORT}", flush=True)
    try:
        await stop_ev.wait()
    finally:
        await bot.cancel()
        bot.st.running = False
        await asyncio.sleep(0.3)
        bot.car.stop(); bot.car.pwm.all_off()
        srv.shutdown()


if __name__ == "__main__":
    import fcntl
    _lock = open("/tmp/motion.lock", "w")
    try:
        fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)   # exclusive with motion.py
    except OSError:
        print(json.dumps({"ok": False, "error": "motion.py or botd.py already running"})); sys.exit(1)
    asyncio.run(main())
