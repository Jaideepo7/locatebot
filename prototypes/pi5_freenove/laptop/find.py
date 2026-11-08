#!/usr/bin/env python3
"""find.py "orange box" - look for an item with LocateAnything (Jetson), turn to it, drive up to it.

Uses botd.py on the Pi (look_around, turn_deg, snap, approach) and locate_server.py on the Orin.
Writes annotated photos + report.json to ./finds/<timestamp>/ and prints a JSON report.
Env: CAR_URL (default http://raspberrypi.local:8000), ORIN_URL (default http://orin.local:8091 - set it),
     CAM_HFOV_DEG (default 41: OV5647 1080p crop mode used by the Pi camera stack).
"""
import io, json, math, os, sys, time, urllib.parse, urllib.request
from PIL import Image, ImageDraw

CAR = os.environ.get("CAR_URL", "http://raspberrypi.local:8000")
ORIN = os.environ.get("ORIN_URL", "http://orin.local:8091")
HFOV = math.radians(float(os.environ.get("CAM_HFOV_DEG", "41")))
SCAN_W, CONFIRM_W = 448, 640                       # measured on the Orin: 448 px 1.1 s, 640 px 2.1 s
# PIL hue 0-255 ranges + minimum saturation, measured on this camera: orange crate hue 6 / sat 219,
# brown cardboard hue 33-34 / sat 115-145 (must be rejected)
COLOURS = {"orange": (0, 28), "red": (-12, 5), "yellow": (29, 45),
           "green": (45, 120), "blue": (120, 185)}
MIN_SAT = {"orange": 160, "red": 150, "cardboard": 70, "brown": 70, "yellow": 140, "green": 90, "blue": 90}


def get(url, timeout=60):
    return urllib.request.urlopen(url, timeout=timeout).read()


def car(path, timeout=60):
    return json.loads(get(CAR + path, timeout))


def locate(jpeg_960, item, send_w=None):
    """Resize (448 px scan / 640 px confirm), ask the Orin, return boxes scaled back to the original."""
    send_w = send_w or SCAN_W
    im = Image.open(io.BytesIO(jpeg_960)).convert("RGB")
    W, H = im.size
    small = im.resize((send_w, round(H * send_w / W)))
    buf = io.BytesIO(); small.save(buf, "JPEG", quality=90)
    req = urllib.request.Request(f"{ORIN}/locate?item={urllib.parse.quote(item)}", data=buf.getvalue(),
                                 headers={"Content-Type": "image/jpeg"}, method="POST")
    r = json.loads(urllib.request.urlopen(req, timeout=60).read())
    k = W / send_w
    res = r["results"][0]
    boxes = [[v * k for v in b] for b in res["boxes"]]
    return boxes, res["ms"], (W, H)


def colour_check(jpeg, box, item):
    """If the item names a colour, check the box's centre really has that hue (brown cardboard != orange)."""
    if any(w in item.lower() for w in ("cardboard", "brown")):   # trust the detector; only reject vivid orange
        o = colour_check(jpeg, box, "orange")
        return {"colour": "cardboard", "ok": o.get("match_frac", 0) < 0.5, "match_frac": round(1 - o.get("match_frac", 0), 2),
                "mean_sat": o.get("mean_sat")}
    want = next((c for c in COLOURS if c in item.lower()), None)
    im = Image.open(io.BytesIO(jpeg)).convert("RGB")
    x1, y1, x2, y2 = box; cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) / 4, (y2 - y1) / 4
    hsv = im.crop((int(cx - w), int(cy - h), int(cx + w), int(cy + h))).convert("HSV")
    px = [p for p in hsv.getdata() if p[1] > 60 and p[2] > 50]            # ignore grey / dark pixels
    sat = sum(p[1] for p in hsv.getdata()) / max(1, len(list(hsv.getdata())))
    if not want:
        return {"colour": None, "ok": True, "mean_sat": round(sat)}
    lo, hi = COLOURS[want]
    hits = sum(1 for p in px if lo <= (p[0] if p[0] < 230 else p[0] - 256) <= hi and p[1] >= MIN_SAT[want])
    frac = hits / max(1, len(list(hsv.getdata())))
    return {"colour": want, "ok": frac >= 0.25, "match_frac": round(frac, 2), "mean_sat": round(sat)}


def bearing_right_deg(box, W, pan_deg):
    """Direction of the box centre relative to the car's front (+ = right)."""
    f = (W / 2) / math.tan(HFOV / 2)
    cx = (box[0] + box[2]) / 2
    return pan_deg + math.degrees(math.atan((cx - W / 2) / f))


def annotate(jpeg, boxes, label, path):
    im = Image.open(io.BytesIO(jpeg)).convert("RGB"); d = ImageDraw.Draw(im)
    for b in boxes:
        d.rectangle(b, outline=(255, 60, 0), width=4); d.text((b[0] + 4, b[1] + 4), label, fill=(255, 60, 0))
    im.save(path, quality=88)


def pan_of(tag):
    if tag.endswith("ahead"):
        return 0.0
    n = float("".join(c for c in tag.split("_")[-1] if c.isdigit()))
    return n if tag.endswith("R") else -n



COMMIT_FRAC = 0.5        # colour signature needed to commit to a target and start moving


def locate_window(jpg, item, cx_hint):
    """Track: run LocateAnything on a 480x270 window around the expected x (2x detail of a 448 px
    full-frame scan, same speed). Falls back to the full frame at 640 px. Boxes in full-image px."""
    im = Image.open(io.BytesIO(jpg)).convert("RGB"); W, H = im.size
    if cx_hint is not None:
        x0 = int(max(0, min(W - 480, cx_hint - 240))); y0 = H // 4
        crop = im.crop((x0, y0, x0 + 480, y0 + 270)); b = io.BytesIO(); crop.save(b, "JPEG", quality=92)
        boxes, ms, _ = locate(b.getvalue(), item, 480)
        boxes = [[x1 + x0, y1 + y0, x2 + x0, y2 + y0] for x1, y1, x2, y2 in boxes]
        boxes = [bx for bx in boxes if colour_check(jpg, bx, item).get("match_frac", 1.0) >= COMMIT_FRAC]
        if boxes:
            return boxes, W, H
    boxes, ms, (W, H) = locate(jpg, item, CONFIRM_W)
    return [bx for bx in boxes if colour_check(jpg, bx, item).get("match_frac", 1.0) >= COMMIT_FRAC], W, H


def pursue(item, best, out, say, report):
    """Commit and move: drive toward the target's direction, steering around obstacles on the Pi
    (/goto), then re-find it with the camera (tracking window) and update the direction."""
    bearing = best["bearing"]                    # target direction relative to the car (+ = right)
    f = (best["W"] / 2) / math.tan(HFOV / 2)
    misses = 0
    for step in range(24):
        if abs(bearing) > 25:                    # far off to the side: one measured turn first
            t = car(f"/turn_deg?deg={bearing:.1f}", 90)
            got = ((t.get("last_result") or {}).get("detail") or {}).get("achieved", bearing)
            bearing -= got
            say(f"  step {step}: big turn {got:+.1f} deg, target now {bearing:+.1f} deg")
        cx_hint = best["W"] / 2 + f * math.tan(math.radians(max(-20, min(20, bearing))))
        jpg = get(f"{CAR}/snap?pan=0", 30)
        boxes, W, H = locate_window(jpg, item, cx_hint)
        annotate(jpg, boxes, item, os.path.join(out, f"go_{step:02d}.jpg"))
        st = car("/state")
        hop = 2.0
        if boxes:
            misses = 0
            b = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
            bearing = bearing_right_deg(b, W, 0)
            bottom, width = b[3], b[2] - b[0]
            say(f"  step {step}: {item} at {bearing:+.1f} deg, bottom y {bottom:.0f}/{H}, width {width:.0f}px, sonar {st['sonar_cm']} cm")
            if bottom > 0.90 * H or width > 0.40 * W or (st["sonar_cm"] is not None and st["sonar_cm"] < 35 and abs(bearing) < 15 and bottom > 0.6 * H):
                say("  ARRIVED (target fills the bottom of the frame)"); report["arrived"] = True; return
            if bottom > 0.7 * H:
                hop = 1.0                            # close: shorter hops
        else:
            misses += 1
            # look left/right before trusting dead reckoning
            for pan in (-17, 17):
                pj = get(f"{CAR}/snap?pan={pan}", 30)
                pb, W, H = locate_window(pj, item, None)
                if pb:
                    bearing = bearing_right_deg(max(pb, key=lambda b: b[3]), W, pan); misses = 0; break
            get(f"{CAR}/snap?pan=0", 30)
            say(f"  step {step}: {'re-found by panning' if misses == 0 else f'not seen (miss {misses}/3)'} -> heading for {bearing:+.1f} deg")
            if misses > 3:
                say("  lost it"); return
        g = car(f"/goto?bearing={bearing:.1f}&seconds={hop}&speed=45", 30)
        if ((g.get('last_result') or {}).get('detail') or {}).get('stopped') == 'arrived_at_obstacle' and boxes:
            say('  ARRIVED (stopped right at it)'); report['arrived'] = True; return
        det = (g.get("last_result") or {}).get("detail") or {}
        if "target_rel_end" in det:
            bearing = det["target_rel_end"]
        say(f"    goto {hop}s: rotated {det.get('rotated_deg')} deg on the way, target now {bearing:+.1f} deg ({det.get('stopped')})")
    say("  gave up after 24 steps")


def main(item, rounds=3):
    out = os.path.join("finds", time.strftime("%H%M%S") + "_" + item.replace(" ", "_"))
    os.makedirs(out, exist_ok=True)
    log, t0 = [], time.time()
    say = lambda m: (log.append(f"{time.time()-t0:6.1f}s {m}"), print(log[-1], flush=True))
    report = {"item": item, "found": False}
    say(f"orin: {json.loads(get(ORIN + '/health', 10))}")
    for rnd in range(rounds):
        say(f"round {rnd+1}: look around")
        look = car("/look_around", 60)
        shots = (look.get("last_result") or {}).get("detail", {}).get("photos", [])
        best = None
        for name in shots:
            jpg = get(f"{CAR}/photo/{name}")
            boxes, ms, (W, H) = locate(jpg, item)
            pan = pan_of(name.rsplit(".", 1)[0])
            say(f"  {name}: pan {pan:+.0f} deg -> {len(boxes)} box(es) in {ms} ms")
            annotate(jpg, boxes, item, os.path.join(out, "look_" + name))
            for b in boxes:
                area = (b[2] - b[0]) * (b[3] - b[1])
                cc = colour_check(jpg, b, item)
                say(f"    box {[round(v) for v in b]} colour check {cc}")
                report.setdefault("candidates", []).append({"photo": name, "box": [round(v) for v in b], "colour": cc})
                if cc.get("match_frac", 1.0) < COMMIT_FRAC:
                    continue                                  # e.g. brown cardboard for "orange box"
                if best is None or area > best["area"]:
                    best = {"photo": name, "box": b, "area": area, "W": W,
                            "bearing": bearing_right_deg(b, W, pan)}
        if best:
            report.update(found=True, first_seen=best["photo"], bearing_deg=round(best["bearing"], 1))
            say(f"FOUND in {best['photo']} at {best['bearing']:+.1f} deg (+ = right) -> pursue")
            pursue(item, best, out, say, report)
            st = car("/state")
            jpg = get(f"{CAR}/snap?pan=0", 30)
            boxes, W, H = locate_window(jpg, item, None)
            annotate(jpg, boxes, item, os.path.join(out, "final.jpg"))
            report.update(confirmed_at_end=bool(boxes), final_front_cm=st["front_cm"], final_source=st["front_source"],
                          final_sonar_cm=st["sonar_cm"], nearest_lidar=st["nearest_lidar_deg_cm"],
                          final_box=[round(v) for v in max(boxes, key=lambda b: b[3])] if boxes else None)
            say(f"END: arrived={report.get('arrived', False)} confirmed={bool(boxes)} sonar={st['sonar_cm']} cm")
            break
        say("  not found this round")
        if rnd == 0:
            say("turn around (180)"); car("/turn_deg?deg=180", 90)
        elif rnd < rounds - 1:
            say("wander 20 s"); car("/wander?seconds=20&speed=35"); car("/wait?seconds=40", 60)
    report["log"] = log; report["dir"] = os.path.abspath(out)
    json.dump(report, open(os.path.join(out, "report.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in report.items() if k != "log"}, indent=1))


if __name__ == "__main__":
    main(" ".join(sys.argv[1:]) or "orange box")
