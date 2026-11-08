#!/usr/bin/env python3
"""Decide whether the car's left/right labels match the camera's frame.
Snap, pivot with left side +, right side - (should turn RIGHT), snap, phase-correlate.
If the scene shifts LEFT in the image (dx < 0), the car turned right -> labels correct."""
import io, json, os, sys, urllib.request
import numpy as np
from PIL import Image

BASE = os.environ.get("CAR_URL", "http://raspberrypi.local:8000")
OUT = sys.argv[1] if len(sys.argv) > 1 else "."


def snap(name):
    data = urllib.request.urlopen(BASE + "/snap", timeout=15).read()
    open(f"{OUT}/{name}.jpg", "wb").write(data)
    g = np.asarray(Image.open(io.BytesIO(data)).convert("L"), dtype=np.float32)
    return g - g.mean()


def shift(a, b):
    """dx of b relative to a (positive = content moved right)."""
    win = np.outer(np.hanning(a.shape[0]), np.hanning(a.shape[1]))
    A, B = np.fft.fft2(a * win), np.fft.fft2(b * win)
    R = A.conj() * B
    r = np.fft.ifft2(R / (np.abs(R) + 1e-9)).real
    y, x = np.unravel_index(np.argmax(r), r.shape)
    if x > a.shape[1] // 2: x -= a.shape[1]
    if y > a.shape[0] // 2: y -= a.shape[0]
    return int(x), int(y), float(r.max())


a = snap("lr_before")
res = json.loads(urllib.request.urlopen(BASE + "/drive?l=55&r=-55&t=0.3", timeout=15).read())
import time; time.sleep(0.4)
b = snap("lr_after")
dx, dy, peak = shift(a, b)
print(json.dumps({"drive": res, "dx_px": dx, "dy_px": dy, "peak": round(peak, 3)}))
if abs(dx) < 8:
    print("VERDICT: too little rotation to tell")
elif dx < 0:
    print("VERDICT: scene moved LEFT -> car turned RIGHT -> labels CORRECT")
else:
    print("VERDICT: scene moved RIGHT -> car turned LEFT -> labels SWAPPED")
