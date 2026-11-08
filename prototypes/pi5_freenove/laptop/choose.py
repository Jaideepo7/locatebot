#!/usr/bin/env python3
"""choose.py - LLM-in-the-loop target choice.

  python3 choose.py scan "cardboard box"   car stays still: camera pans, every match gets a number;
                                           writes finds/choices/choices.json + sheet.jpg (tagged photos)
  python3 choose.py go 1                   orient to choice 1 with small braked turn pulses
                                           (re-finding THAT object after each pulse), then drive straight to it
"""
import io, json, math, os, sys, time
from PIL import Image, ImageDraw
from find import CAR, get, car, bearing_right_deg, locate
from seek import detect
from find import locate_window

OUT = os.path.join("finds", "choices")
F_PX = 1290.0                                  # measured focal length at 960 px (HFOV 40.8 deg)


def log(m, _t0=[time.time()]):
    print(f"{time.time()-_t0[0]:6.1f}s {m}", flush=True)


def pan_of(name):
    tag = name.rsplit(".", 1)[0].split("_")[-1]
    if tag == "ahead":
        return 0.0
    n = float("".join(c for c in tag if c.isdigit()))
    return n if tag.endswith("R") else -n


def scan(item):
    os.makedirs(OUT, exist_ok=True)
    car("/stop")
    look = car("/look_around", 60)
    names = ((look.get("last_result") or {}).get("detail") or {}).get("photos", [])
    cands, shots = [], []
    for name in names:
        jpg = get(f"{CAR}/photo/{name}")
        pan = pan_of(name)
        boxes, W, H, how = detect(jpg, item)
        shots.append((name, jpg, pan, boxes))
        for b in boxes:
            cands.append({"bearing": round(bearing_right_deg(b, W, pan), 1), "width": round(b[2] - b[0]),
                          "bottom": round(b[3]), "on_floor": b[3] >= 0.33 * H, "photo": name,
                          "box": [round(v) for v in b], "pan": pan})
        log(f"{name}: pan {pan:+.0f} -> {len(boxes)} match(es)")
    # the same object seen in two overlapping photos -> keep one (the bigger view)
    cands.sort(key=lambda c: -c["width"])
    merged = []
    for c in cands:
        if all(abs(c["bearing"] - m["bearing"]) > 6 for m in merged):
            merged.append(c)
    merged.sort(key=lambda c: c["bearing"])                      # number left -> right
    for i, c in enumerate(merged, 1):
        c["n"] = i
        # rough distance from apparent width, assuming a ~40 cm wide box (only a hint for the chooser)
        c["dist_hint_m"] = round(0.40 * F_PX / max(1, c["width"]), 1)
    json.dump({"item": item, "time": time.strftime("%H:%M:%S"), "choices": merged},
              open(os.path.join(OUT, "choices.json"), "w"), indent=1)
    # tagged sheet: every photo with its numbered boxes
    tiles = []
    for name, jpg, pan, boxes in shots:
        im = Image.open(io.BytesIO(jpg)).convert("RGB"); d = ImageDraw.Draw(im)
        for c in merged:
            if c["photo"] == name:
                d.rectangle(c["box"], outline=(255, 230, 0), width=5)
                d.rectangle([c["box"][0], max(0, c["box"][1] - 46), c["box"][0] + 60, max(46, c["box"][1])], fill=(255, 230, 0))
                d.text((c["box"][0] + 18, max(4, c["box"][1] - 40)), str(c["n"]), fill=(0, 0, 0))
        d.text((8, 8), f"pan {pan:+.0f} deg", fill=(255, 255, 255))
        tiles.append(im.resize((480, 270)))
    sheet = Image.new("RGB", (480 * len(tiles) + 6 * (len(tiles) - 1), 270), "black")
    for i, t in enumerate(tiles):
        sheet.paste(t, (i * 486, 0))
    sheet.save(os.path.join(OUT, "sheet.jpg"), quality=90)
    print("CHOICES (left -> right):")
    for c in merged:
        print(f"  #{c['n']}: {c['bearing']:+.1f} deg, {c['width']}px wide (~{c['dist_hint_m']} m if 40 cm wide), "
              f"{'on the floor' if c['on_floor'] else 'raised'}, seen in {c['photo']}")
    print("sheet:", os.path.abspath(os.path.join(OUT, "sheet.jpg")))


def nearest(boxes, W, expected, max_off=15):
    best = None
    for b in boxes:
        brg = bearing_right_deg(b, W, 0)
        if abs(brg - expected) <= max_off and (best is None or abs(brg - expected) < abs(best[0] - expected)):
            best = (brg, b)
    return best


def fine_turn(brg):
    """ONE pulse toward the target; shorter when only a little off, so it creeps in instead of overshooting."""
    d = "right" if brg > 0 else "left"
    secs = 0.04 if abs(brg) < 8 else (0.06 if abs(brg) < 16 else 0.08)
    car(f"/pulses?dir={d}&n=1&secs={secs}", 30)
    return d, secs


def pan_find(item, width, say):
    """Car still: look with the camera for the target (similar size) and return its bearing, or None."""
    for pan in (-30, 30, -15, 15):
        pj = get(f"{CAR}/snap?pan={pan}", 30)
        pb, W2, H2, _ = detect(pj, item)
        pb = [x for x in pb if (x[2] - x[0]) >= 0.5 * width]
        if pb:
            get(f"{CAR}/snap?pan=0", 30)
            brg = bearing_right_deg(max(pb, key=lambda x: x[2] - x[0]), W2, pan)
            say(f"    camera found it at {brg:+.1f} deg (pan {pan:+d})")
            return brg
    get(f"{CAR}/snap?pan=0", 30)
    return None


def turn_onto(item, brg, width, say, tries=8):
    """Fine single pulses toward brg until the target is centred (re-finding it after each pulse)."""
    for _ in range(tries):
        if abs(brg) <= 4:
            return brg
        d, secs = fine_turn(brg)
        jpg = get(f"{CAR}/snap?pan=0", 30)
        bx, W, H, _ = detect(jpg, item)
        step = {0.04: 4.0, 0.06: 7.0, 0.08: 10.0}[secs]
        hit = nearest([x for x in bx if (x[2] - x[0]) >= 0.5 * width], W, brg - (step if d == "right" else -step), 20)
        brg = hit[0] if hit else brg - (step if d == "right" else -step)
    return brg


def go(n):
    data = json.load(open(os.path.join(OUT, "choices.json")))
    item = data["item"]
    c = next(x for x in data["choices"] if x["n"] == int(n))
    brg, width = c["bearing"], c["width"]
    log(f"go to choice #{n}: {item} at {brg:+.1f} deg, {width}px")
    # ---- ORIENT: small braked pulses toward it, re-finding THAT object after each pulse ----
    dpp, first, misses = 8.0, True, 0
    for i in range(14):
        if abs(brg) <= 4:
            break
        d, secs = fine_turn(brg)
        k = 1
        step_deg = {0.04: 4.0, 0.06: 7.0, 0.08: 10.0}[secs]
        expected = brg - (step_deg if d == "right" else -step_deg)
        jpg = get(f"{CAR}/snap?pan=0", 30)
        boxes, W, H, how = detect(jpg, item)
        hit = nearest(boxes, W, expected, 20)
        if hit:
            brg, width = hit[0], hit[1][2] - hit[1][0]; misses = 0
            log(f"  orient: {k} {d} pulse(s) -> choice #{n} now at {brg:+.1f} deg ({width:.0f}px)")
        else:
            brg = expected; misses += 1
            log(f"  orient: {k} {d} pulse(s) -> not re-found, expected at {brg:+.1f} deg (miss {misses})")
            if misses > 2:
                log("  lost the chosen object while orienting -> stop"); car("/stop"); return
    # ---- DRIVE: FAR = continuous straight, checked on the move; NEAR = short hops with sharp stopped photos ----
    NEAR_PX = 120                                   # box this wide (~2 m) -> stop short, then creep in with checked hops
    misses, reorients, arrived, sidesteps, hops = 0, 0, False, 0, 0
    growth = 1.0                                    # how much the box grew between the last two moving photos
    moving, t_start = False, time.time()

    def stopped_look():
        jpg2 = get(f"{CAR}/snap?pan=0", 30)                   # sharp: car is stopped
        bx, W2, H2, _ = detect(jpg2, item)
        return nearest([x for x in bx if 0.6 * width <= (x[2] - x[0]) <= 2.2 * width], W2, 0.0, 25), W2, H2

    def is_arrival(brg_, b_, W_, H_, son_):
        w_ = b_[2] - b_[0]
        return (son_ is not None and son_ < 40 and abs(brg_) < 15) or w_ > 0.7 * W_ or b_[3] > 0.92 * H_

    while time.time() - t_start < 120:
        near = width >= NEAR_PX or width * growth >= NEAR_PX   # also predict the next check (it keeps moving ~1.3 s)
        if near:                                              # ---- NEAR: hop, stop, look ----
            if moving:
                car("/stop"); moving = False
            r = car("/goto?bearing=0&seconds=0.4&speed=30&visual=1", 20)
            stopped = ((r.get("last_result") or {}).get("detail") or {}).get("stopped")
            hops += 1
            hit, W, H = stopped_look()
            st = car("/state")
            son = st["sonar_cm"] if (st["sonar_cm"] is not None and st["sonar_cm"] < 400) else None
            close_ahead = (son is not None and son < 60) or ((st.get("lidar_lane_cm") or 999) < 60)
            if not hit and not close_ahead and width > 0.12 * W:     # close but not in front: probably beside us
                side = None
                for pan in (-30, 30):
                    pj = get(f"{CAR}/snap?pan={pan}", 30)
                    pb, W2, H2, _ = detect(pj, item)
                    pb = [x for x in pb if (x[2] - x[0]) >= 0.6 * width]
                    if pb:
                        side = bearing_right_deg(max(pb, key=lambda x: x[2] - x[0]), W2, pan); break
                get(f"{CAR}/snap?pan=0", 30)
                if side is not None:
                    log(f"  hop {hops}: box is beside the car at {side:+.1f} deg -> turn to it")
                    for _ in range(6):
                        fine_turn(side)
                        hit, W, H = stopped_look()
                        if hit:
                            break
                        side = side - (7 if side > 0 else -7)
                    if hit:
                        brg, b = hit; width = b[2] - b[0]; continue
            if not hit:
                if width > 0.35 * W or close_ahead or stopped == "arrived_at_obstacle":
                    log(f"  hop {hops}: box now fills the view / stopped at it (sonar {son}) -> ARRIVED")
                    arrived = True; break
                log(f"  hop {hops}: lost it -> stop"); break
            brg, b = hit; width = b[2] - b[0]
            log(f"  hop {hops} ({stopped}): choice #{n} at {brg:+.1f} deg, {width:.0f}px, sonar {son}")
            if is_arrival(brg, b, W, H, son) or (stopped == "arrived_at_obstacle" and abs(brg) < 15):
                log(f"  ARRIVED at choice #{n}"); arrived = True; break
            if abs(brg) > 4:
                d, _ = fine_turn(brg)
                reorients += 1; log(f"    re-orient: {d}")
            continue
        # ---- FAR: continuous straight, photos on the move ----
        if not moving:
            car("/goto?bearing=0&seconds=20&speed=30&visual=1&watchdog=4&wait=0", 15)
            moving = True; log("  driving straight (continuous)")
        jpg = get(f"{CAR}/snap?fast=1", 15)
        if width < 160:
            boxes, W, H = locate_window(jpg, item, 480.0)
        else:
            boxes, _ms, (W, H) = locate(jpg, item, 448)
        st = car("/state")
        son = st["sonar_cm"] if (st["sonar_cm"] is not None and st["sonar_cm"] < 400) else None
        if st.get("mission") != "goto":                       # the drive stopped itself (obstacle / watchdog)
            moving = False
            res = ((st.get("last_result") or {}).get("detail") or {}).get("stopped")
            hit, W, H = stopped_look()
            log(f"  drive ended: {res}")
            if res == "arrived_at_obstacle" and (width >= 0.3 * W or (hit and hit[1][2] - hit[1][0] >= 0.3 * W)):
                log(f"  ARRIVED: the obstacle it stopped at is the box"); arrived = True; break
            if res == "arrived_at_obstacle":
                if sidesteps >= 2:
                    log("  stop: still blocked after 2 sidesteps"); break
                sidesteps += 1
                bins = {int(k): v for k, v in car("/bins").get("bins", {}).items()}
                left = sum(min(bins.get(a, 0), 300) for a in (-60, -50, -40, -30))
                right = sum(min(bins.get(a, 0), 300) for a in (30, 40, 50, 60))
                side = "right" if right >= left else "left"; back = "left" if side == "right" else "right"
                k = max(2, round(30 / dpp))
                log(f"  blocked by something else -> sidestep {side}")
                car("/back?seconds=0.4", 15); car(f"/pulses?dir={side}&n={k}", 30)
                car("/goto?bearing=0&seconds=0.9&speed=30&visual=1", 20); car(f"/pulses?dir={back}&n={k}", 30)
                hit, W, H = stopped_look()
                if not hit:                                       # after the sidestep it is off to a side: look
                    side_brg = pan_find(item, width, log)
                    if side_brg is not None:
                        brg = turn_onto(item, side_brg, width, log)
                        hit, W, H = stopped_look()
            if not hit:
                log("  stop: lost it"); break
            brg, b = hit; width = b[2] - b[0]
            if abs(brg) > 4:
                fine_turn(brg); reorients += 1
            continue
        hit = nearest([x for x in boxes if 0.75 * width <= (x[2] - x[0]) <= 1.8 * width], W, 0.0, 25)
        if not hit:
            misses += 1
            log(f"    moving photo: not seen ({misses}/2)")
            if misses < 2:
                car("/aim?keep=1"); continue
            car("/stop"); moving = False; misses = 0             # stop and REALLY look
            hit, W, H = stopped_look()
            if not hit:
                st2 = car("/state"); s2 = st2.get("sonar_cm") or 999; l2 = st2.get("lidar_lane_cm") or 999
                if width > 0.35 * W or s2 < 60 or l2 < 60:
                    log("  stopped: it's right in front -> ARRIVED"); arrived = True; break
                log("  stopped and looked: not there -> stop"); break
            log("    stopped and looked: found it again")
        misses = 0
        brg, b = hit
        growth = max(1.0, min(2.0, (b[2] - b[0]) / max(1.0, width)))
        width = b[2] - b[0]
        log(f"    {'moving' if moving else 'stopped'}: choice #{n} at {brg:+.1f} deg, {width:.0f}px, sonar {son}")
        if is_arrival(brg, b, W, H, son):
            car("/stop"); moving = False; log(f"  ARRIVED at choice #{n}"); arrived = True; break
        if abs(brg) > 4:
            if moving:
                car("/stop"); moving = False
            d, _ = fine_turn(brg); reorients += 1
            log(f"    re-orient #{reorients}: {d}")
            continue
        if moving:
            car("/aim?keep=1")
    car("/stop")
    log(f"  done: arrived={arrived}, re-orients={reorients}, near hops={hops}, {time.time() - t_start:.0f} s")
    jpg = get(f"{CAR}/snap?pan=0", 30)
    open(os.path.join(OUT, f"final_choice{n}.jpg"), "wb").write(jpg)
    st = car("/state")
    print(json.dumps({"choice": int(n), "final_sonar_cm": st["sonar_cm"], "final_front_cm": st["front_cm"],
                      "final_photo": os.path.abspath(os.path.join(OUT, f"final_choice{n}.jpg"))}))


if __name__ == "__main__":
    if sys.argv[1] == "scan":
        scan(" ".join(sys.argv[2:]) or "cardboard box")
    else:
        go(sys.argv[2])
