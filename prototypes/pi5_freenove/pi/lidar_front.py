#!/usr/bin/env python3
"""Find close objects (<80 cm) in a few LiDAR rotations and list them as angle clusters."""
import json, time
from lidar import RPLidar

lid = RPLidar(); lid.start()
pts = []; n = 0
try:
    for sc in lid.scans():
        n += 1
        if n <= 2:            # let the motor reach speed
            continue
        pts += [(a, d) for a, d in sc if 50 < d < 800]
        if n >= 8:
            break
finally:
    lid.close()

pts.sort()
clusters, cur = [], []
for a, d in pts:
    if cur and a - cur[-1][0] > 4:
        clusters.append(cur); cur = []
    cur.append((a, d))
if cur:
    clusters.append(cur)
# merge wrap-around (cluster near 360 with cluster near 0)
if len(clusters) > 1 and clusters[0][0][0] < 4 and clusters[-1][-1][0] > 356:
    clusters[0] = [(a - 360, d) for a, d in clusters[-1]] + clusters[0]; clusters.pop()
out = []
for c in clusters:
    if len(c) < 3:
        continue
    out.append({"center_deg": round(sum(a for a, _ in c) / len(c) % 360, 1),
                "span_deg": round(c[-1][0] - c[0][0], 1),
                "min_cm": round(min(d for _, d in c) / 10, 1), "points": len(c)})
out.sort(key=lambda x: x["min_cm"])
print(json.dumps(out, indent=1))
