#!/usr/bin/env python3
"""Print decoded link messages from a publisher (bench test for the Pi <-> Orin link).

Examples:
  python tools/link_echo.py tcp://<pi-ip>:5555 --topic scan --topic pi_status
  python tools/link_echo.py tcp://<orin-ip>:5556 --newest drive
Ctrl+C closes the socket and exits.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from locatebot.lidar.types import Scan  # noqa: E402
from locatebot.link.messages import Frame  # noqa: E402
from locatebot.link.transport import Subscriber  # noqa: E402

ALL_TOPICS = ["scan", "frame", "pi_status", "drive", "estop", "orin_status"]


def describe(msg) -> str:
    if isinstance(msg, Scan):
        return f"scan seq={msg.seq} stamp_s={msg.stamp_s:.3f} n={len(msg)} valid={int(msg.valid_mask().sum())}"
    if isinstance(msg, Frame):
        return f"frame seq={msg.seq} capture_s={msg.capture_s:.3f} bytes={len(msg.jpeg)}"
    return repr(msg)


def main(argv: list[str] | None = None, out=print) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("endpoint", help="publisher endpoint, e.g. tcp://<host>:5555")
    ap.add_argument("--topic", action="append", choices=ALL_TOPICS, help="topic to show (repeatable; default all)")
    ap.add_argument("--newest", action="append", default=[], choices=ALL_TOPICS,
                    help="show only the newest message of this topic per poll (repeatable)")
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long (default: until Ctrl+C)")
    a = ap.parse_args(argv)
    topics = a.topic or ALL_TOPICS
    end = None if a.seconds is None else time.monotonic() + a.seconds
    with Subscriber(a.endpoint, sorted(set(topics) | set(a.newest)), newest_only=a.newest) as sub:
        try:
            while end is None or time.monotonic() < end:
                for msg in sub.poll(0.2):
                    out(f"{time.monotonic():.3f} {describe(msg)}")
        except KeyboardInterrupt:
            pass
        out(f"received={sub.received} malformed={sub.malformed} superseded={sub.superseded}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
