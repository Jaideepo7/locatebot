#!/usr/bin/env python3
"""seek.py "orange box" - detect FIRST, move ONLY when the target is confirmed.

Every step: photo straight ahead -> LocateAnything on the Jetson (wait for the answer) ->
  target confirmed  -> one action (turn toward it if far off to the side, else drive at it)
  not confirmed     -> NO motor action at all: stop and report.
No 360 search, no wandering, no driving blind.
"""
import io, json, math, os, sys, time
from PIL import Image
from find import (CAR, get, car, locate, colour_check, bearing_right_deg, annotate, COMMIT_FRAC)

BAND = (100, 370)          # horizon strip (y range) checked at higher detail if the full frame finds nothing


def confirmed(jpg, boxes, item):
    return [b for b in boxes if colour_check(jpg, b, item).get("match_frac", 1.0) >= COMMIT_FRAC]


def detect(jpg, item):
    """Full frame at 448 px; if nothing, the horizon strip at 768 px (~1.8x the detail)."""
    boxes, ms, (W, H) = locate(jpg, item, 448)
    ok = confirmed(jpg, boxes, item)
    if ok:
        return ok, W, H, f"full frame ({ms} ms)"
    im = Image.open(io.BytesIO(jpg)).convert("RGB")
    band = im.crop((0, BAND[0], W, BAND[1])); b = io.BytesIO(); band.save(b, "JPEG", quality=92)
    bb, ms2, _ = locate(b.getvalue(), item, 768)
    bb = [[x1, y1 + BAND[0], x2, y2 + BAND[0]] for x1, y1, x2, y2 in bb]
    return confirmed(jpg, bb, item), W, H, f"horizon strip ({ms} + {ms2} ms)"


def signature(jpg, box):
    """Appearance fingerprint: hue x saturation histogram of the box centre (normalised)."""
    import numpy as np
    im = Image.open(io.BytesIO(jpg)).convert("RGB")
    x1, y1, x2, y2 = box; cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) / 3, (y2 - y1) / 3
    hsv = np.asarray(im.crop((int(cx - w), int(cy - h), int(cx + w), int(cy + h))).convert("HSV")).reshape(-1, 3)
    hist, _, _ = np.histogram2d(hsv[:, 0], hsv[:, 1], bins=[16, 4], range=[[0, 256], [0, 256]])
    hist = hist / max(1.0, hist.sum())
    gray = np.asarray(im.crop((int(x1 + 0.1 * (x2 - x1)), int(y1 + 0.1 * (y2 - y1)), int(x2 - 0.1 * (x2 - x1)),
                               int(y2 - 0.1 * (y2 - y1)))).convert("L").resize((48, 24)), dtype=float)
    gray = (gray - gray.mean()) / (gray.std() + 1e-6)      # pattern: markings/logos on the object
    return hist, gray


def same_thing(sig_a, sig_b):
    """Colour histogram overlap AND greyscale pattern correlation; returns the weaker of the two (0..1)."""
    import numpy as np
    colour = float(np.minimum(sig_a[0], sig_b[0]).sum())
    pattern = float((sig_a[1] * sig_b[1]).mean())         # normalised cross-correlation, -1..1
    return min(colour, max(0.0, pattern) + 0.2)           # pattern must roughly match too


SEARCH_PULSE_S = 0.08          # one search step + brake: ~6-10 deg
MAX_SEARCH_PULSES = 45         # first search (never seen yet): ~ one full circle at ~8 deg/step
SAME_MIN = 0.0                 # appearance fingerprint OFF: it rejected the real box (lighting/scale); continuity checks remain
MAX_TURNS_WITHOUT_DRIVE = 18   # turning without driving forward is pointless: stop
LOST_SEARCH = ["near"] * 6 + ["far"] * 12   # after a lock: ~45 deg toward last seen side, then ~45 deg the other way


def structured_search(item, first, say, out, lock_sig):
    """Front sector, then behind. Each heading is photographed once: sweep ~45 deg to one side, jump back past
    the centre WITHOUT photos (already seen), sweep the other side; then turn ~180 and do the same behind."""
    other = "right" if first == "left" else "left"
    r = car(f"/pulses?dir={first}&n=2&measure=1", 70)                 # first step also calibrates deg/pulse
    dpp = float(((r.get("last_result") or {}).get("detail") or {}).get("deg_per_pulse", 8.0))
    step_n = max(1, round(15 / dpp))                                   # ~15 deg per photo step (camera sees 41)
    side_steps = max(1, round(45 / (step_n * dpp)))                    # photo steps per side
    say(f"  search: {dpp:.1f} deg per pulse -> {step_n} pulse(s) per photo step, {side_steps} steps per side")

    def look(tag):
        jpg = get(f"{CAR}/snap?pan=0", 30)
        boxes, W, H, how = detect(jpg, item)
        if lock_sig is not None:
            boxes = [b for b in boxes if same_thing(lock_sig, signature(jpg, b)) >= SAME_MIN]
        annotate(jpg, boxes, item, os.path.join(out, f"search_{tag}.jpg"))
        say(f"    search photo {tag}: {'FOUND' if boxes else 'nothing'}")
        return bool(boxes)

    for sector in ("front", "back"):
        if sector == "back":
            n180 = round(180 / dpp) - side_steps * step_n                # from the far side to facing backwards
            car(f"/pulses?dir={other}&n={max(1, n180)}", 90)
            say(f"  nothing in front -> turned ~180 to look behind ({n180} pulses)")
            if look("back_centre"):
                return True
        for i in range(side_steps):                                    # one side (first step already done in front)
            if not (sector == "front" and i == 0):
                car(f"/pulses?dir={first}&n={step_n}", 30)
            if look(f"{sector}_{first}{i + 1}"):
                return True
        jump = side_steps * step_n + step_n                            # back past the centre, no photos
        car(f"/pulses?dir={other}&n={jump}", 60)
        if look(f"{sector}_{other}1"):
            return True
        for i in range(1, side_steps):
            car(f"/pulses?dir={other}&n={step_n}", 30)
            if look(f"{sector}_{other}{i + 1}"):
                return True
    return False


def main(item, max_steps=80, search_dir="left"):
    out = os.path.join("finds", "seek_" + time.strftime("%H%M%S") + "_" + item.replace(" ", "_"))
    os.makedirs(out, exist_ok=True)
    log, t0 = [], time.time()
    say = lambda m: (log.append(f"{time.time()-t0:6.1f}s {m}"), print(log[-1], flush=True))
    report = {"item": item, "found": False, "arrived": False}
    car("/stop")                                              # make sure nothing is moving
    locked = None                                             # (bearing, width) of the committed target
    search_pulses = 0
    last_action = None
    lock_sig = None
    lost_steps = 0
    searched = False
    turns_since_drive = 0
    for step in range(max_steps):
        if last_action == "turn":
            turns_since_drive += 1
        elif last_action == "drive":
            turns_since_drive = 0
            searched = False                                  # progress made: a later loss may search again
        if turns_since_drive > MAX_TURNS_WITHOUT_DRIVE:
            say(f"step {step}: {turns_since_drive} turns with no forward progress -> stopping")
            break
        jpg = get(f"{CAR}/snap?pan=0", 30)
        boxes, W, H, how = detect(jpg, item)                  # wait for the model before ANY action
        if locked and boxes and last_action == "drive":   # right after a drive: must be near where it was
            boxes = [b for b in boxes                         # near where it should be, same kind of thing:
                     if abs(bearing_right_deg(b, W, 0) - locked[0]) < 25]
        if locked and boxes and lock_sig is not None:         # same object: looks the same
            boxes = [b for b in boxes if same_thing(lock_sig, signature(jpg, b)) >= SAME_MIN]
        if locked and boxes:                                  # always: same kind of thing (size, floor, shape)
            boxes = [b for b in boxes
                     if (b[2] - b[0]) > 0.4 * locked[1]
                     and (not locked[2] or b[3] >= 0.33 * H)      # on the floor if it was on the floor
                     and 0.5 < ((b[2] - b[0]) / max(1, b[3] - b[1])) / locked[3] < 2.0]   # similar shape
        annotate(jpg, boxes, item, os.path.join(out, f"step_{step:02d}.jpg"))
        st = car("/state")
        sonar = st["sonar_cm"] if (st["sonar_cm"] is not None and st["sonar_cm"] < 400) else None
        if not boxes and locked and locked[1] > 0.35 * W and abs(locked[0]) < 10:
            # it was big/close and is now "gone": it fills the frame (the detector drops frame-filling boxes)
            say(f"step {step}: target was close and now fills the view (sonar {sonar} cm) -> ARRIVED, no more motion")
            report["arrived"] = True
            break
        if not boxes and locked and locked[1] > 0.2 * W:      # was fairly close, now gone, not centred/large: stop
            say(f"step {step}: lost the target close by (last {locked[1]:.0f}px at {locked[0]:+.1f} deg) -> stopping, no motion")
            report["lost_close"] = True
            break
        if not boxes and locked and lost_steps == 0:          # lost after a lock: look with the CAMERA first (car still)
            seen = None
            for pan in (-30, 30):
                pj = get(f"{CAR}/snap?pan={pan}", 30)
                pb, W, H, _ = detect(pj, item)
                pb = [x for x in pb if lock_sig is None or same_thing(lock_sig, signature(pj, x)) >= SAME_MIN]
                if pb:
                    seen = bearing_right_deg(max(pb, key=lambda x: x[2] - x[0]), W, pan); break
            get(f"{CAR}/snap?pan=0", 30)
            if seen is not None:
                d = "right" if seen > 0 else "left"
                n = max(1, min(4, round(abs(seen) / 9)))      # ~9 deg per braked pulse
                for _ in range(n):
                    car(f"/turn?dir={d}&seconds=0.08", 20)
                say(f"step {step}: lost straight ahead, camera found it at {seen:+.1f} deg -> {n} {d} pulse(s)")
                last_action = "turn"; continue
        if not boxes:                                         # structured search: front sweep, then behind, no re-scans
            if searched:
                say(f"step {step}: already searched front and back -> stopping")
                break
            searched = True
            first = search_dir if not locked else ("right" if locked[0] > 0 else "left")
            hit = structured_search(item, first, say, out, lock_sig)
            if hit:
                last_action = "turn"; continue                    # next step photographs it and acts
            say(f"step {step}: '{item}' not found in front or behind -> stopping")
            break
        report["found"] = True
        b = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        brg, bottom, width = bearing_right_deg(b, W, 0), b[3], b[2] - b[0]
        locked = (brg, width, bottom >= 0.33 * H, width / max(1, b[3] - b[1]))   # bearing, size, on floor, shape
        lost_steps = 0
        if lock_sig is None:
            lock_sig = signature(jpg, b)                    # remember what the target looks like
        say(f"step {step}: CONFIRMED '{item}' ({how}) at {brg:+.1f} deg, bottom y {bottom:.0f}/{H}, "
            f"width {width:.0f}px, sonar {st['sonar_cm']} cm")
        report.update(last_box=[round(v) for v in b], last_bearing=round(brg, 1), last_sonar_cm=st["sonar_cm"])
        if (sonar is not None and sonar < 35 and abs(brg) < 15) or width > 0.80 * W or bottom > 0.95 * H:
            say("  ARRIVED: target right in front"); report["arrived"] = True; break
        if abs(brg) > 12:                                     # ONE short turn pulse, then photo + confirm again
            d = "right" if brg > 0 else "left"
            secs = round(max(0.05, min(0.11, 0.04 + abs(brg) / 220.0)), 3)   # ~4-12 deg with brake, never a spin
            car(f"/turn?dir={d}&seconds={secs}", 20)
            last_action = "turn"
            say(f"  action: one {d} turn pulse {secs}s (target at {brg:+.1f} deg) -> photo again")
            continue
        if sonar is not None and abs(brg) < 15:             # distance known: cover ~70% of the gap (~57 cm/s)
            hop = max(0.3, min(0.6, 0.7 * (sonar - 30) / 35.0))   # ~35 cm/s; never long hops (drift)
        else:
            hop = 1.2 if width < 40 else (0.7 if width < 120 else 0.4)   # big in frame = close = short hop
        # ---- locked on and roughly ahead: drive CONTINUOUSLY, re-aiming from each new photo while moving ----
        car(f"/goto?bearing={brg:.1f}&seconds=20&speed=30&watchdog=3&wait=0", 15)
        say(f"  action: CONTINUOUS drive at {brg:+.1f} deg (30%), re-aiming every photo")
        misses, arrived = 0, False
        while True:
            fj = get(f"{CAR}/snap?fast=1", 15)
            fb, W, H, how2 = detect(fj, item)
            fb = [x for x in fb if abs(bearing_right_deg(x, W, 0) - locked[0]) < 25
                  and (x[2] - x[0]) >= 0.6 * locked[1]                # approaching: must not shrink much
                  and (not locked[2] or x[3] >= 0.33 * H)
                  and same_thing(lock_sig, signature(fj, x)) >= SAME_MIN]   # same object
            stt = car("/state")
            if stt.get("mission") != "goto":
                res = (stt.get("last_result") or {}).get("detail") or {}
                say(f"    drive ended: {res.get('stopped')}"); break
            if not fb:
                misses += 1
                say(f"    photo: not seen ({misses}/2)")
                if misses >= 2:
                    car("/stop"); say("    lost it while driving -> stop"); break
                car("/aim?keep=1"); continue                      # missed one photo: same aim, feed the watchdog
            misses = 0
            x = max(fb, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]))
            b_now, w_now = bearing_right_deg(x, W, 0), x[2] - x[0]
            locked = (b_now, w_now, locked[2], w_now / max(1, x[3] - x[1]))
            annotate(fj, [x], item, os.path.join(out, f"drive_{step:02d}_{int(time.time()*10)%100000}.jpg"))
            son = stt["sonar_cm"] if (stt["sonar_cm"] is not None and stt["sonar_cm"] < 400) else None
            say(f"    photo: {b_now:+.1f} deg, width {w_now:.0f}px, bottom {x[3]:.0f}/{H}, sonar {son}")
            if (w_now > 0.70 * W and abs(b_now) < 15) or x[3] > 0.92 * H or (son is not None and son < 40 and abs(b_now) < 15):
                car("/stop"); say("  ARRIVED: target big and centred right in front"); report["arrived"] = arrived = True; break
            car(f"/aim?rel={b_now:.1f}")
        report.update(last_box=[round(v) for v in x] if fb else report.get("last_box"))
        if arrived:
            break
        last_action = "drive"
        continue
        g = car(f"/goto?bearing={brg:.1f}&seconds={hop}&speed=30", 30)
        last_action = "drive"
        det = (g.get("last_result") or {}).get("detail") or {}
        say(f"  action: drive {hop}s at {brg:+.1f} deg -> {det.get('stopped')} (rotated {det.get('rotated_deg')} deg)")
        if det.get("stopped") == "arrived_at_obstacle":
            jpg = get(f"{CAR}/snap?pan=0", 30)
            boxes, W, H, how = detect(jpg, item)
            annotate(jpg, boxes, item, os.path.join(out, "arrived_check.jpg"))
            close = [bx for bx in boxes if (bx[3] > 0.6 * H or (bx[2] - bx[0]) > 0.35 * W)
                     and abs(bearing_right_deg(bx, W, 0)) < 15]
            if close:
                say("  ARRIVED: stopped right at it (target big and centred in the photo)"); report["arrived"] = True; break
            say("  stopped by something that is NOT the target (target still far/small in the photo) -> no more motion")
            report["blocked_by_other"] = True
            break
    car("/stop")
    report.update(log=log, dir=os.path.abspath(out), final_state={k: car("/state").get(k) for k in
                  ("front_cm", "front_source", "sonar_cm", "nearest_lidar_deg_cm")})
    json.dump(report, open(os.path.join(out, "report.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in report.items() if k != "log"}, indent=1))


if __name__ == "__main__":
    args = sys.argv[1:]
    sd = "right" if "--right" in args else "left"
    main(" ".join(a for a in args if not a.startswith("--")) or "orange box", search_dir=sd)
