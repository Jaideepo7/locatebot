#!/usr/bin/env python3
"""Record RPLIDAR scans to an .lbscan file.

Example: python tools/record_scan.py --port /dev/ttyUSB0 --out room.lbscan --seconds 30
Ctrl+C stops the motor and closes the file cleanly.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from locatebot.lidar.rplidar import RPLidar  # noqa: E402
from locatebot.lidar.scanlog import ScanLogWriter  # noqa: E402


def _default_factory(port: str, baudrate: int, mount_offset_rad: float) -> RPLidar:
    return RPLidar(port, baudrate=baudrate, mount_offset_rad=mount_offset_rad)


def main(argv: list[str] | None = None, lidar_factory=_default_factory) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800, help="460800 for C1, 115200 for A1M8")
    ap.add_argument("--out", required=True, help="output .lbscan path")
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long (default: until Ctrl+C)")
    ap.add_argument("--mount-offset-deg", type=float, default=0.0, help="sensor zero relative to robot forward, CCW")
    args = ap.parse_args(argv)

    lidar = lidar_factory(args.port, args.baud, math.radians(args.mount_offset_deg))
    count = 0
    rc = 0
    t0 = time.monotonic()
    try:
        status, code = lidar.get_health()
        print(f"health: status={status} error_code={code}")
        print(f"info: {lidar.get_info()}")
        with ScanLogWriter(args.out) as writer:
            try:
                for scan in lidar.iter_scans():
                    writer.write(scan)
                    count += 1
                    if args.seconds is not None and time.monotonic() - t0 >= args.seconds:
                        break
            except KeyboardInterrupt:
                print("interrupted")
            except TimeoutError as e:
                print(f"error: {e}")
                rc = 1
    finally:
        try:
            lidar.stop()
        finally:
            lidar.close()
    elapsed = max(time.monotonic() - t0, 1e-9)
    print(f"scans: {count} in {elapsed:.1f} s ({count / elapsed:.2f} scans/s) -> {args.out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
