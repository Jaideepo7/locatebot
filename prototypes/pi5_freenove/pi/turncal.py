#!/usr/bin/env python3
"""turncal.py - measure in-place pivots with the LiDAR (ICP scan matching) and calibrate them.

  python3 turncal.py measure        no motion: noise floor of the measurement
  python3 turncal.py calibrate      pivot L/R at 2 powers x 4 pulse lengths x 2 reps, measure each
  python3 turncal.py closed         closed-loop turn_deg() to +-90, 45, 15, 5 (uses turncal.json)

+ degrees = clockwise = turned RIGHT. Car must be ON THE FLOOR with ~50 cm clear around it.
"""
import collections, json, os, sys, threading, time, warnings
warnings.filterwarnings("ignore")
import numpy as np
from car import Car
from lidar import RPLidar
from scanmatch import match

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "turncal.json")
POWERS, PULSES = (0.35, 0.50), (0.06, 0.10, 0.16, 0.25)
# rough prior (deg/s, offset) from the first small-pulse run; only used to seed the matcher
PRIOR = {("right", 0.35): (126, -3.6), ("left", 0.35): (193, -7.2),
         ("right", 0.50): (190, -6.3), ("left", 0.50): (244, -8.3)}


class Lidar:
    def __init__(self):
        self.lid = RPLidar(); self.lid.start()
        self.scans = collections.deque(maxlen=20); self.n = 0; self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        for sc in self.lid.scans():
            if not self.running:
                break
            self.scans.append(sc); self.n += 1

    def points(self, k=3, settle=0.45):
        """Merged points of k fresh scans taken after the car has settled."""
        time.sleep(settle)
        n0 = self.n
        while self.n < n0 + k:
            time.sleep(0.02)
        pts = []
        for sc in list(self.scans)[-k:]:
            pts += sc
        return pts

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.lid.close()


def guess(direction, power, secs):
    rate, off = PRIOR.get((direction, power), (150, -5))
    g = max(0.0, rate * secs + off)
    return g if direction == "right" else -g


def pivot(car, direction, power, secs, brake_s=0.1):
    sgn = 1 if direction == "right" else -1
    l, r = sgn * power, -sgn * power
    car._apply({"front_left": l, "back_left": l, "front_right": r, "back_right": r}, 1.0)
    time.sleep(secs)
    car.brake(brake_s)                         # crisp stop -> repeatable


def calibrate(car, lid):
    rows = []
    prev = lid.points()
    plan = [(d, p, s) for p in POWERS for s in PULSES for d in ("right", "left")]
    for rep in range(2):
        for d, p, s in plan:
            pivot(car, d, p, s)
            cur = lid.points()
            m = match(prev, cur, guess_right_deg=guess(d, p, s))
            rows.append({"dir": d, "power": p, "secs": s, **m})
            print(f"{d:5} p={p:.2f} t={s:.2f}s -> {m['deg_right']:+7.2f} deg  shift {m['shift_m']} m  "
                  f"rms {m['rms_mm']:4.1f} mm  inliers {m['inliers']:.2f}", flush=True)
            prev = cur
    good = [r for r in rows if r["inliers"] >= 0.6 and r["rms_mm"] < 25]
    fits = {}
    print(f"\n{len(good)}/{len(rows)} measurements trusted (inliers >= 0.6, rms < 25 mm)")
    print("SUMMARY  dir   power  secs   mean|deg|  spread  n")
    for d in ("right", "left"):
        for p in POWERS:
            xs, ys = [], []
            for s in PULSES:
                v = [abs(r["deg_right"]) for r in good if r["dir"] == d and r["power"] == p and r["secs"] == s]
                if v:
                    print(f"         {d:5} {p:.2f}   {s:.2f}   {np.mean(v):7.2f}   {np.ptp(v):5.2f}   {len(v)}")
                xs += [s] * len(v); ys += v
            if len(set(xs)) >= 2:
                a, b = np.polyfit(xs, ys, 1)
                fits[f"{d}@{p}"] = {"deg_per_s": round(float(a), 1), "offset_deg": round(float(b), 2)}
                print(f"         fit {d}@{p}: |deg| = {a:.1f} * secs {b:+.2f}")
    out = {"rows": rows, "fits": fits, "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(OUT, "w") as f:
        json.dump(out, f, indent=1)
    print("saved", OUT)


def turn_deg(car, lid, target, power=0.35, tol=1.5, max_pulses=12, start_pts=None):
    """Closed loop: pulse, measure with LiDAR, correct. + = right. Returns (achieved, pulses, pts)."""
    fits = json.load(open(OUT))["fits"]
    prev = start_pts if start_pts is not None else lid.points()
    total, pulses = 0.0, 0
    while pulses < max_pulses:
        rem = target - total
        if abs(rem) <= tol:
            break
        d = "right" if rem > 0 else "left"
        f = fits.get(f"{d}@{power}", {"deg_per_s": 150, "offset_deg": -5})
        secs = (abs(rem) * 0.85 - f["offset_deg"]) / max(f["deg_per_s"], 1.0)   # aim a bit short
        secs = max(0.05, min(secs, 0.30))
        pivot(car, d, power, secs)
        pulses += 1
        cur = lid.points(k=2, settle=0.35)
        g = max(0.0, f["deg_per_s"] * secs + f["offset_deg"]) * (1 if d == "right" else -1)
        m = match(prev, cur, guess_right_deg=g, spread_deg=20)
        total += m["deg_right"]; prev = cur
    return total, pulses, prev


def closed(car, lid):
    print("target   achieved   error   pulses")
    pts = lid.points()
    errs = []
    for tgt in (90, -90, 45, -45, 15, -15, 5, -5):
        got, n, pts = turn_deg(car, lid, tgt, start_pts=pts)
        errs.append(got - tgt)
        print(f"{tgt:+5d}   {got:+8.2f}   {got - tgt:+6.2f}   {n}", flush=True)
    print(f"mean |error| {np.mean(np.abs(errs)):.2f} deg, worst {np.max(np.abs(errs)):.2f} deg")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "measure"
    car = Car(); car.min_duty = 700
    lid = Lidar()
    try:
        while lid.n < 3:
            time.sleep(0.1)
        if mode == "calibrate":
            calibrate(car, lid)
        elif mode == "closed":
            closed(car, lid)
        else:
            a = lid.points(); time.sleep(3); b = lid.points()
            print("no motion (noise floor):", match(a, b))
    finally:
        car.stop(); lid.close()
