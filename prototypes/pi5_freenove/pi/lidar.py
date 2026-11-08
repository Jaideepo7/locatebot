#!/usr/bin/env python3
"""Minimal RPLIDAR C1 driver (standard scan mode) + top-down plot.
   python3 lidar.py plot [out.png]   -> grab ~1 s, save a top-down image, print summary JSON
"""
import json, math, os, sys, time, serial

PORT, BAUD = "/dev/ttyUSB0", 460800


class RPLidar:
    def __init__(self, port=PORT, baud=BAUD):
        self.s = serial.Serial(port, baud, timeout=0.5)
        self.s.dtr = False

    def start(self):
        self.s.write(b"\xa5\x25"); time.sleep(0.05); self.s.reset_input_buffer()
        self.s.write(b"\xa5\x20")                      # SCAN
        desc = self.s.read(7)
        if not desc.startswith(b"\xa5\x5a"):
            raise RuntimeError(f"bad scan descriptor {desc.hex()}")

    def stop(self):
        self.s.write(b"\xa5\x25"); time.sleep(0.05); self.s.reset_input_buffer()

    def nodes(self):
        """Yield (new_rotation, quality, angle_deg, dist_mm)."""
        buf = b""
        while True:
            buf += self.s.read(500)
            while len(buf) >= 5:
                b0, b1 = buf[0], buf[1]
                s_flag, ns_flag, check = b0 & 1, (b0 >> 1) & 1, b1 & 1
                if s_flag == ns_flag or not check:      # resync one byte at a time
                    buf = buf[1:]; continue
                q = b0 >> 2
                ang = ((b1 >> 1) | (buf[2] << 7)) / 64.0
                dist = (buf[3] | (buf[4] << 8)) / 4.0
                buf = buf[5:]
                yield bool(s_flag), q, ang, dist

    def scans(self):
        """Yield full rotations as lists of (angle_deg, dist_mm) with dist>0."""
        cur = []
        for new, q, a, d in self.nodes():
            if new and cur:
                yield cur; cur = []
            if d > 0:
                cur.append((a, d))

    def close(self):
        self.stop(); self.s.close()


def plot(scan, path, max_m=4.0, size=700):
    from PIL import Image, ImageDraw
    img = Image.new("RGB", (size, size), (15, 15, 20)); d = ImageDraw.Draw(img)
    c = size / 2; k = c / (max_m * 1000)
    for r in range(1, int(max_m) + 1):                  # 1 m rings
        d.ellipse([c - r * 1000 * k, c - r * 1000 * k, c + r * 1000 * k, c + r * 1000 * k], outline=(50, 50, 60))
        d.text((c + 4, c - r * 1000 * k + 2), f"{r} m", fill=(90, 90, 100))
    for deg, col in ((0, (255, 80, 80)), (90, (80, 160, 255))):
        x = c + math.sin(math.radians(deg)) * c * 0.95; y = c - math.cos(math.radians(deg)) * c * 0.95
        d.line([c, c, x, y], fill=col); d.text((x - 20, y), f"{deg} deg", fill=col)
    for a, mm in scan:
        x = c + math.sin(math.radians(a)) * mm * k; y = c - math.cos(math.radians(a)) * mm * k
        if 0 <= x < size and 0 <= y < size:
            d.rectangle([x - 1, y - 1, x + 1, y + 1], fill=(120, 255, 140))
    d.rectangle([c - 6, c - 6, c + 6, c + 6], outline=(255, 255, 0))
    img.save(path)


if __name__ == "__main__":
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser("~/car/lidar.png")
    lid = RPLidar(); lid.start()
    t0 = time.time(); scans = []
    try:
        for sc in lid.scans():
            scans.append(sc)
            if len(scans) >= 6:
                break
    finally:
        lid.close()
    dt = time.time() - t0
    best = max(scans[1:], key=len)                     # skip first partial rotation
    plot(best, out)
    ds = sorted(d for _, d in best)
    near = min(best, key=lambda p: p[1])
    print(json.dumps({"rotations": len(scans), "secs": round(dt, 2), "hz": round((len(scans) - 1) / dt, 1),
                      "points_per_rotation": len(best), "nearest": {"angle_deg": round(near[0], 1), "mm": near[1]},
                      "median_mm": ds[len(ds) // 2], "image": out}))
