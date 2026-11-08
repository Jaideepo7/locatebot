#!/usr/bin/env python3
"""Run LocateAnything on photos with the model loaded once, and time every call.

  python tools/locate.py photos/desk.jpg="tape measure,marker" photos/room.jpg=chair \
      --mode hybrid --repeat 2 --out results/run1.json

Each argument is IMAGE=ITEM[,ITEM...]; every item is its own call (one item per prompt).
With --repeat N each call runs N times: the first time shows warm-up cost, the rest steady state.
The JSON report has load time, per-call times, raw boxes and boxes after post-processing.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from locatebot.vision.engine import MODES, EngineError, LocateEngine, find_library  # noqa: E402
from locatebot.vision.postprocess import build_prompt, clean_detections, jpeg_size  # noqa: E402


def parse_job(arg: str) -> tuple[Path, list[str]]:
    image, sep, items = arg.partition("=")
    names = [s.strip() for s in items.split(",") if s.strip()]
    if not sep or not names:
        raise ValueError(f"expected IMAGE=ITEM[,ITEM...], got {arg!r}")
    return Path(image), names


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("jobs", nargs="+", help="IMAGE=ITEM[,ITEM...]")
    ap.add_argument("--model", type=Path, default=ROOT / "models" / "locate-anything-q4_k.gguf")
    ap.add_argument("--lib", type=Path, help="liblocate_anything.so (default: search third_party build)")
    ap.add_argument("--mode", choices=sorted(MODES), default="hybrid")
    ap.add_argument("--threads", type=int, default=0, help="CPU threads (0 = engine default)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--out", type=Path, help="write the JSON report here")
    args = ap.parse_args(argv)

    jobs = [parse_job(j) for j in args.jobs]
    lib = args.lib or find_library(ROOT)

    t0 = time.perf_counter()
    engine = LocateEngine(args.model, lib_path=lib, n_threads=args.threads)
    load_s = time.perf_counter() - t0
    print(f"model loaded in {load_s:.1f} s ({args.mode} mode)", flush=True)

    runs, failed = [], False
    with engine:
        for image, items in jobs:
            data = image.read_bytes()
            width, height = jpeg_size(data)
            for item in items:
                run = {"image": str(image), "item": item, "width": width, "height": height, "times_s": []}
                try:
                    for _ in range(max(1, args.repeat)):
                        t = time.perf_counter()
                        dets = engine.locate(data, build_prompt(item), mode=args.mode)
                        run["times_s"].append(round(time.perf_counter() - t, 3))
                    clean = clean_detections(dets, width, height)
                    run["raw"] = [asdict(d) for d in dets]
                    run["clean"] = [asdict(d) for d in clean]
                    summary = ", ".join(f"[{', '.join(f'{v:.0f}' for v in d.box)}]" for d in clean) or "nothing"
                    print(f"{image.name:24} {item:20} {run['times_s']} s  -> {summary}", flush=True)
                except EngineError as e:
                    run["error"] = str(e)
                    failed = True
                    print(f"{image.name:24} {item:20} ERROR {e}", flush=True)
                runs.append(run)

    report = {"model": str(args.model), "mode": args.mode, "load_s": round(load_s, 3), "runs": runs}
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
