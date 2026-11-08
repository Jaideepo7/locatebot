#!/usr/bin/env python3
"""Replay a recorded scan log against a saved map and report how localization behaves.

  python tools/relocalize.py --map maps/room1.lbmap --log recordings/room1_drive.lbscan
  python tools/relocalize.py --map maps/room1.lbmap --log run.lbscan --guess 0.5,-0.2,30
  python tools/relocalize.py --map maps/room1.lbmap --log run.lbscan --global --every 1

Without --guess the localizer starts from the map's home pose (where mapping started); with
--global it starts with no guess at all (search over every FREE cell and heading). Prints the
pose track every --every scans, every lost / ambiguous / recovery / odometry-fallback event,
and a summary (scores, pose spread, timing). The map file is never modified; --out writes the
slowly updated map to a new stem.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from locatebot.lidar.localize import LocalizerConfig, LocalizingMapper  # noqa: E402
from locatebot.lidar.scanlog import read_scan_log  # noqa: E402
from locatebot.lidar.types import Pose2D  # noqa: E402


def parse_pose(text: str) -> Pose2D:
    try:
        x, y, deg = (float(v) for v in text.split(","))
    except ValueError as e:
        raise argparse.ArgumentTypeError("pose must be x,y,deg") from e
    return Pose2D(x, y, math.radians(deg))


def run(map_path, log_path, guess: Pose2D | None, use_global: bool, every: int, out=None,
        config: LocalizerConfig | None = None, quiet: bool = False) -> dict:
    lm = LocalizingMapper.from_file(map_path, guess=guess, config=config)
    if guess is None and not use_global:
        lm.localizer.request_init(lm.home)
    elif use_global:
        lm.localizer.request_init(None)
    say = (lambda *a: None) if quiet else print
    times, scores, poses, events = [], [], [], []
    prev = None
    for i, scan in enumerate(read_scan_log(log_path)):
        t0 = time.perf_counter()
        r = lm.add_scan(scan)
        times.append(time.perf_counter() - t0)
        state = "lost" if r.lost else "ambiguous" if r.ambiguous else r.source
        if r.accepted:
            scores.append(r.score)
            poses.append((r.pose.x, r.pose.y, r.pose.theta))
        if state != prev and (prev is not None or state != "map"):
            ev = {"scan": i, "stamp_s": round(scan.stamp_s, 3), "state": state, "score": round(r.score, 3)}
            if r.ambiguous and r.runner_up is not None:
                ev["runner_up"] = [round(r.runner_up.x, 2), round(r.runner_up.y, 2),
                                   round(math.degrees(r.runner_up.theta), 1), round(r.runner_up_score, 3)]
            events.append(ev)
            say(f"event  scan {i:5d}  {state:10s} score {r.score:.3f}"
                + (f"  runner-up {ev['runner_up']}" if "runner_up" in ev else ""))
        prev = state
        if every and i % every == 0:
            p = r.pose
            say(f"scan {i:5d}  t {scan.stamp_s:10.3f}  pose ({p.x:7.3f}, {p.y:7.3f}, {math.degrees(p.theta):7.2f} deg)"
                f"  score {r.score:.3f}  {state}  {times[-1] * 1e3:6.1f} ms")
    summary = {"scans": lm.n_scans, "accepted": len(poses), "integrated": lm.n_integrated,
               "rejected": lm.n_rejected, "lost_events": lm.localizer.n_lost_events,
               "recoveries": lm.localizer.n_recoveries, "field_rebuilds": lm.n_rebuilds,
               "events": events}
    if scores:
        a = np.array(poses)
        summary.update({
            "score_mean": round(float(np.mean(scores)), 3), "score_min": round(float(np.min(scores)), 3),
            "pose_first": [round(v, 3) for v in a[0]], "pose_last": [round(v, 3) for v in a[-1]],
            "pose_std_xy_m": [round(float(v), 4) for v in a[:, :2].std(axis=0)],
            "pose_std_theta_deg": round(float(np.degrees(a[:, 2].std())), 3),
        })
    if times:
        t = np.array(times[1:] or times) * 1e3  # the first scan includes the initialization
        summary.update({"init_ms": round(times[0] * 1e3, 1), "update_ms_p50": round(float(np.median(t)), 1),
                        "update_ms_p95": round(float(np.percentile(t, 95)), 1),
                        "update_ms_max": round(float(t.max()), 1)})
    if out is not None:
        summary["files"] = {k: str(v) for k, v in lm.save(out, meta={"relocalize_log": str(log_path)}).items()}
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", type=Path, required=True)
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--guess", type=parse_pose, help="initial pose guess x,y,deg (map frame)")
    ap.add_argument("--global", dest="use_global", action="store_true", help="no guess: global search")
    ap.add_argument("--every", type=int, default=10, help="print the pose every N scans (0: events only)")
    ap.add_argument("--out", type=Path, help="write the slowly updated map to this stem")
    args = ap.parse_args(argv)
    summary = run(args.map, args.log, args.guess, args.use_global, args.every, args.out)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
