#!/usr/bin/env python3
"""
car.py - safe movement primitives for the robot car (the layer an LLM will call).

Every move is time-limited and ALWAYS ends with motors stopped.
Every function returns a plain dict (JSON-friendly) describing what actually happened.

CLI (prints JSON):
    python3 car.py forward  [speed 1-100] [seconds]
    python3 car.py backward [speed 1-100] [seconds]
    python3 car.py turn <kind> [speed] [seconds]
          kind: spin_left | spin_right | front_left | front_right | back_left | back_right
    python3 car.py stop
    python3 car.py distance
    python3 car.py invert <wheel>      # flip a wheel that spins the wrong way
    python3 car.py calibration         # show wheel direction settings
"""
import json, os, sys, time, warnings

warnings.filterwarnings("ignore")
from smbus2 import SMBus
from hw import PCA9685

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "calibration.json")

# Wheel -> (channel_A, channel_B) on the PCA9685, found by testing.
# Which of the two drives the wheel FORWARD is set by calibration.json.
WHEELS = {
    "front_left":  (0, 1),
    "back_left":   (2, 3),
    "back_right":  (4, 5),
    "front_right": (6, 7),
}
# Default guess (Freenove layout): forward channel per wheel. Fixed by 'invert' after testing.
DEFAULT_FORWARD_CH = {"front_left": 1, "back_left": 2, "back_right": 5, "front_right": 7}

# Turn patterns: +1 = wheel forward, -1 = wheel backward
_FWD = {"front_left": 1, "back_left": 1, "back_right": 1, "front_right": 1}
TURNS = {
    "spin_left":   {"front_left": -1, "back_left": -1, "back_right": 1, "front_right": 1},
    "spin_right":  {"front_left": 1, "back_left": 1, "back_right": -1, "front_right": -1},
    "front_left":  {**_FWD, "front_left": -1},
    "front_right": {**_FWD, "front_right": -1},
    "back_left":   {**_FWD, "back_left": -1},
    "back_right":  {**_FWD, "back_right": -1},
}

# Safety limits
MAX_SECONDS_TURN = 6.0
MAX_SECONDS_FORWARD = 5.0
MAX_SECONDS_BACKWARD = 2.0       # no rear sensor -> keep reversing short
OBSTACLE_STOP_CM = 20.0          # forward() refuses/stops if something is closer than this
RAMP_S = 0.15                    # soft start / soft stop
MIN_DUTY, MAX_DUTY = 1200, 4095  # speed 1..100 maps onto this range (motors stall below ~1200)

TRIG_PIN, ECHO_PIN = 27, 22


def _load_cal():
    try:
        with open(CAL_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return dict(DEFAULT_FORWARD_CH)


def _save_cal(cal):
    with open(CAL_PATH, "w") as f:
        json.dump(cal, f, indent=2)


class Car:
    def __init__(self):
        self.bus = SMBus(1)
        self.pwm = PCA9685(self.bus, freq=50)   # starts with every channel OFF
        self.cal = _load_cal()
        self._sonar = None

    # ---------- sensors ----------
    def distance_cm(self):
        """Distance to the nearest thing in front, in cm (None if no reading)."""
        if self._sonar is None:
            from gpiozero import DistanceSensor
            self._sonar = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN, max_distance=3, queue_len=3)
            time.sleep(0.15)
        try:
            return round(self._sonar.distance * 100, 1)
        except Exception:
            return None

    # ---------- low level ----------
    def _wheel(self, name, power):
        """power -1.0 .. +1.0 (positive = forward) for one wheel.
        If calibration has swap_lr=true, left/right labels are mirrored (car's own frame)."""
        if self.cal.get("swap_lr"):
            name = name.replace("left", "@").replace("right", "left").replace("@", "right")
        a, b = WHEELS[name]
        fwd = self.cal.get(name, DEFAULT_FORWARD_CH[name])
        rev = b if fwd == a else a
        lo = getattr(self, "min_duty", MIN_DUTY)
        duty = 0 if power == 0 else int(lo + (MAX_DUTY - lo) * min(abs(power), 1.0))
        if power > 0:
            self.pwm.set_duty(rev, 0); self.pwm.set_duty(fwd, duty)
        elif power < 0:
            self.pwm.set_duty(fwd, 0); self.pwm.set_duty(rev, duty)
        else:
            self.pwm.set_duty(fwd, 0); self.pwm.set_duty(rev, 0)

    def _all(self, power):
        for w in WHEELS:
            self._wheel(w, power)

    def _apply(self, pattern, power):
        """pattern: {wheel: -1..+1}, power 0..1"""
        for w, s in pattern.items():
            p = s * power
            self._wheel(w, 0 if abs(s) < 0.01 else p)

    def stop(self):
        """Stop all wheel motors (servos keep holding the camera)."""
        for ch in range(8):
            self.pwm.set_duty(ch, 0)
        return {"ok": True, "action": "stop"}

    def brake(self, seconds=0.15):
        """Active brake: both H-bridge inputs ON (short-brake) briefly, then release."""
        for ch in range(8):
            self.pwm.set_duty(ch, 4095)
        time.sleep(seconds)
        self.stop()

    def drive(self, left=50, right=50, seconds=0.5):
        """Differential drive: left/right side speeds -100..100 (negative = backward).
        e.g. (50,50) straight, (50,35) gentle right, (40,-40) pivot right, (-40,-40) back."""
        l = max(-100, min(100, float(left))) / 100.0
        r = max(-100, min(100, float(right))) / 100.0
        pattern = {"front_left": l, "back_left": l, "front_right": r, "back_right": r}
        going_fwd = l >= 0 and r >= 0 and (l + r) > 0
        going_back = l <= 0 and r <= 0
        max_s = MAX_SECONDS_BACKWARD if going_back else MAX_SECONDS_FORWARD
        res = self._drive("drive", 100, seconds, max_s, going_fwd, pattern=pattern)
        res.update(left=round(l * 100), right=round(r * 100))
        return res

    # ---------- movement ----------
    def _drive(self, direction, speed, seconds, max_s, watch_front, pattern=None):
        speed = max(1, min(100, int(speed)))
        req = float(seconds)
        seconds = max(0.0, min(req, max_s))
        if pattern is None:
            sign = 1 if direction == "forward" else -1
            pattern = {w: sign for w in WHEELS}
        target = speed / 100.0
        result = {"ok": True, "action": direction, "speed": speed,
                  "seconds_requested": req, "seconds_allowed": seconds}
        if req > max_s:
            result["note"] = f"capped to {max_s}s for safety"

        if watch_front:
            d = self.distance_cm()
            result["distance_cm_start"] = d
            if d is not None and d < OBSTACLE_STOP_CM:
                result.update(ok=False, moved_s=0.0, stopped_because="obstacle_ahead")
                return result

        t0 = time.time()
        stopped = "completed"
        try:
            while True:
                t = time.time() - t0
                if t >= seconds:
                    break
                # soft start / soft stop
                ramp = min(1.0, t / RAMP_S, (seconds - t) / RAMP_S)
                self._apply(pattern, target * max(ramp, 0.05))
                if watch_front:
                    d = self.distance_cm()
                    if d is not None and d < OBSTACLE_STOP_CM:
                        stopped = "obstacle_ahead"
                        result["distance_cm_at_stop"] = d
                        break
                time.sleep(0.03)
        finally:
            self.stop()
        result["moved_s"] = round(time.time() - t0, 2)
        result["stopped_because"] = stopped
        if stopped != "completed":
            result["ok"] = False
        if watch_front:
            result["distance_cm_end"] = self.distance_cm()
        return result

    def forward(self, speed=50, seconds=1.0, check_obstacles=True):
        """Drive straight forward (camera side). Stops early if an obstacle is < 20 cm."""
        return self._drive("forward", speed, seconds, MAX_SECONDS_FORWARD, check_obstacles)

    def backward(self, speed=50, seconds=1.0):
        """Drive straight backward. Limited to 2 s because there is no rear sensor."""
        return self._drive("backward", speed, seconds, MAX_SECONDS_BACKWARD, False)

    def turn(self, kind="spin_left", speed=70, seconds=1.0):
        """Turn the car. kind = spin_left | spin_right (in place, 360-capable)
        or front_left | front_right | back_left | back_right (that wheel reverses, others forward)."""
        if kind not in TURNS:
            return {"ok": False, "action": "turn", "error": f"unknown turn '{kind}'",
                    "valid": list(TURNS)}
        res = self._drive("turn", speed, seconds, MAX_SECONDS_TURN, False, pattern=TURNS[kind])
        res["turn"] = kind
        return res

    def close(self):
        self.stop()
        if self._sonar:
            self._sonar.close()


def _cli(argv):
    if not argv:
        print(__doc__); return
    cmd, args = argv[0], argv[1:]
    if cmd == "invert":
        cal = _load_cal()
        w = args[0]
        a, b = WHEELS[w]
        cal[w] = b if cal.get(w, DEFAULT_FORWARD_CH[w]) == a else a
        _save_cal(cal)
        print(json.dumps({"ok": True, "wheel": w, "forward_channel_now": cal[w]})); return
    if cmd == "calibration":
        print(json.dumps({"forward_channel": _load_cal(), "wheels": WHEELS})); return

    car = Car()
    try:
        if cmd == "forward":
            sp = float(args[0]) if args else 50
            s = float(args[1]) if len(args) > 1 else 1.0
            nocheck = "--no-obstacle-check" in args
            out = car.forward(sp, s, check_obstacles=not nocheck)
        elif cmd == "backward":
            out = car.backward(float(args[0]) if args else 50, float(args[1]) if len(args) > 1 else 1.0)
        elif cmd == "turn":
            kind = args[0] if args else "spin_left"
            out = car.turn(kind, float(args[1]) if len(args) > 1 else 70,
                           float(args[2]) if len(args) > 2 else 1.0)
        elif cmd == "stop":
            out = car.stop()
        elif cmd == "distance":
            out = {"ok": True, "distance_cm": car.distance_cm()}
        else:
            out = {"ok": False, "error": f"unknown command {cmd}"}
        print(json.dumps(out))
    finally:
        car.close()


if __name__ == "__main__":
    _cli(sys.argv[1:])
