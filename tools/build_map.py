#!/usr/bin/env python3
"""Build a session map file from a recorded scan log, or from a synthetic demo run.

  python tools/build_map.py --log run.lbscan --out maps/session1
  python tools/build_map.py --demo --out maps/demo      # no hardware needed (synthetic)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from locatebot.lidar.mapping import Mapper, build_map_from_log, demo_room, demo_trajectory  # noqa: E402
from locatebot.lidar.sim import simulate_scan  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--log", type=Path, help="recorded .lbscan file")
    src.add_argument("--demo", action="store_true", help="synthetic room + loop trajectory")
    ap.add_argument("--out", type=Path, required=True, help="output stem; writes .lbmap, .pgm, .yaml")
    ap.add_argument("--size", type=float, default=20.0, help="map side length in metres")
    ap.add_argument("--resolution", type=float, default=0.05, help="metres per cell")
    args = ap.parse_args()

    if args.log:
        summary = build_map_from_log(args.log, args.out, size_m=args.size, resolution_m=args.resolution)
    else:
        mapper = Mapper(size_m=args.size, resolution_m=args.resolution)
        rng = np.random.default_rng(0)
        room = demo_room()
        for i, pose in enumerate(demo_trajectory()):
            mapper.add_scan(simulate_scan(room, pose, noise_std_m=0.01, rng=rng, stamp_s=i * 0.1, seq=i, dropout=0.02))
        paths = mapper.save(args.out, meta={"source": "synthetic demo"})
        summary = {"scans": mapper.n_scans, "integrated": mapper.n_integrated, "rejected": mapper.n_rejected,
                   "files": {k: str(v) for k, v in paths.items()}, "synthetic": True}
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
