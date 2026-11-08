#!/usr/bin/env python3
"""Laptop-side client for the car server.
  bot.py snap [name]                 stable snapshot (waits until camera is still)
  bot.py drive L R T [name]          drive, settle, stable snapshot
  bot.py pair L R T [name]           stable before + drive + stable after, side-by-side sheet
"""
import io, json, os, sys, time, urllib.request
import numpy as np
from PIL import Image, ImageDraw

B = os.environ.get("CAR_URL", "http://raspberrypi.local:8000")
D = os.environ.get("CAR_OUT", "captures")
os.makedirs(D, exist_ok=True)


def get(path, timeout=15):
    return urllib.request.urlopen(B + path, timeout=timeout).read()


def small(data):
    return np.asarray(Image.open(io.BytesIO(data)).convert("L").resize((96, 54)), dtype=np.float32)


def snap_stable(settle=0.6, max_s=4.0, thresh=2.5):
    """Wait `settle`, then grab frames until two in a row match (camera not moving)."""
    time.sleep(settle)
    t0 = time.time()
    prev = get("/snap"); p = small(prev)
    while time.time() - t0 < max_s:
        time.sleep(0.25)
        cur = get("/snap"); c = small(cur)
        diff = float(np.abs(c - p).mean())
        if diff < thresh:
            return cur, round(diff, 2)
        prev, p = cur, c
    return prev, None          # never settled; return last frame


def drive(l, r, t):
    return json.loads(get(f"/drive?l={l}&r={r}&t={t}"))


def save(data, name):
    path = f"{D}/{name}.jpg"
    open(path, "wb").write(data)
    return path


def main(a):
    cmd = a[0]
    if cmd == "snap":
        data, d = snap_stable(settle=0.1)
        print("IMG", save(data, a[1] if len(a) > 1 else "snap"), "still_diff", d)
    elif cmd == "drive":
        res = drive(a[1], a[2], a[3])
        print("drive", {k: res.get(k) for k in ("left", "right", "moved_s", "stopped_because", "distance_cm_end")})
        data, d = snap_stable()
        print("IMG", save(data, a[4] if len(a) > 4 else "drive"), "still_diff", d)
    elif cmd == "pair":
        b, _ = snap_stable(settle=0.1)
        res = drive(a[1], a[2], a[3])
        print("drive", {k: res.get(k) for k in ("left", "right", "moved_s", "stopped_because")})
        c, d = snap_stable()
        A, C = Image.open(io.BytesIO(b)), Image.open(io.BytesIO(c))
        s = Image.new("RGB", (A.width * 2 + 10, A.height + 40), "black")
        s.paste(A, (0, 40)); s.paste(C, (A.width + 10, 40))
        dr = ImageDraw.Draw(s)
        dr.text((10, 10), "BEFORE", fill="white")
        dr.text((A.width + 20, 10), f"AFTER drive(l={a[1]}, r={a[2]}, t={a[3]})", fill="white")
        name = a[4] if len(a) > 4 else "pair"
        s.resize((s.width // 2, s.height // 2)).save(f"{D}/{name}.jpg")
        print("IMG", f"{D}/{name}.jpg", "still_diff", d)
    print("sense", get("/sense").decode())


if __name__ == "__main__":
    main(sys.argv[1:])
