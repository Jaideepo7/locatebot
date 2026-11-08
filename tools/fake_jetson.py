#!/usr/bin/env python3
"""Pretend to be the Jetson: connect to the backend and play a scripted mission (no robot needed).

Runs the real JetsonAgent with ScriptedSink instead of LocateAnything, on a laptop or on the Jetson.
Credentials come from the environment (JETSON_DEVICE_ID, JETSON_AUTH_TOKEN), never the command line.

Examples:
  python tools/fake_jetson.py --url ws://127.0.0.1:8000/ws/robot --script happy
  python tools/fake_jetson.py --url ws://127.0.0.1:8000/ws/robot --script stall --drop-after 4
  python tools/fake_jetson.py --url ws://127.0.0.1:8000/ws/robot --silent
Scripts: happy, not_found, stall, reject. Ctrl+C to stop.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from locatebot.dispatch.agent import JetsonAgent  # noqa: E402
from locatebot.dispatch.sink import SCRIPTS, ScriptedSink  # noqa: E402
from locatebot.dispatch.ws_client import run_client  # noqa: E402


class LoggingSink(ScriptedSink):
    def start(self, mission_id, target_description, report):
        logging.info("MISSION %s: find %r -> would go to LocateAnything", mission_id, target_description)
        super().start(mission_id, target_description, report)

    def cancel(self, mission_id):
        logging.info("CANCEL %s", mission_id)
        super().cancel(mission_id)

    def on_link_lost(self, mission_id):
        logging.warning("LINK LOST during %s: stopping (local safety policy)", mission_id)
        super().on_link_lost(mission_id)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--url", required=True, help="ws:// or wss:// URL of /ws/robot")
    ap.add_argument("--script", choices=sorted(SCRIPTS), default="happy")
    ap.add_argument("--drop-after", type=float, default=None, help="drop the first connection after N seconds")
    ap.add_argument("--silent", action="store_true", help="connect but never answer (tests the ACK timeout)")
    args = ap.parse_args(argv)
    device, token = os.environ.get("JETSON_DEVICE_ID", ""), os.environ.get("JETSON_AUTH_TOKEN", "")
    if not device or not token:
        print("set JETSON_DEVICE_ID and JETSON_AUTH_TOKEN (same values as the backend)", file=sys.stderr)
        return 2
    if args.url.startswith("ws://") and not any(h in args.url for h in ("127.0.0.1", "localhost")):
        print("warning: ws:// sends the token unencrypted; use wss:// over the internet", file=sys.stderr)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    for noisy in ("websockets", "asyncio"):
        logging.getLogger(noisy).setLevel(logging.WARNING)
    sink = LoggingSink(SCRIPTS[args.script])

    class LoggingAgent(JetsonAgent):
        def _send(self, frame):
            logging.info("-> %s", frame)
            super()._send(frame)

    try:
        asyncio.run(run_client(args.url, device, token, lambda send: LoggingAgent(sink, send=send), poll=sink.poll,
                               drop_after_s=args.drop_after, ignore_frames=args.silent))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
