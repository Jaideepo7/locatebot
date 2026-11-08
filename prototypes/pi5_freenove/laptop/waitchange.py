#!/usr/bin/env python3
"""Block until the camera view changes (person moved), then wait for it to settle.
Saves the settled frame to look/f_N.jpg and prints its path + sensors. Max wait arg (s)."""
import io, json, os, sys, time, glob, urllib.request
import numpy as np
from PIL import Image

D = os.environ.get("CAR_OUT", "captures")
os.makedirs(D, exist_ok=True)
B = os.environ.get("CAR_URL", "http://raspberrypi.local:8000")
maxwait = float(sys.argv[1]) if len(sys.argv) > 1 else 30


def grab():
    data = urllib.request.urlopen(B + "/snap", timeout=10).read()
    g = np.asarray(Image.open(io.BytesIO(data)).convert("L").resize((96, 54)), dtype=np.float32)
    return data, g


ref_data, ref = grab()
t0 = time.time(); changed = False
prev = ref
while time.time() - t0 < maxwait:
    time.sleep(0.4)
    data, g = grab()
    if not changed and np.abs(g - ref).mean() > 6:
        changed = True; tc = time.time()
    if changed and np.abs(g - prev).mean() < 3 and time.time() - tc > 0.8:
        break                                   # movement happened and has settled
    prev = g
n = len(glob.glob(D + "/f_*.jpg"))
path = f"{D}/f_{n}.jpg"
open(path, "wb").write(data)
sense = urllib.request.urlopen(B + "/sense", timeout=10).read().decode()
print(json.dumps({"changed": changed, "waited_s": round(time.time() - t0, 1)}))
print("sense", sense)
print("IMG", path)
