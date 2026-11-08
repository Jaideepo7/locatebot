#!/usr/bin/env python3
"""Store-guide agent: chat in the terminal or serve POST /message for the Photon bridge.

Needs internet (NVIDIA-hosted Nemotron). Reads NVIDIA_API_KEY and STORE_CODE from the
environment or from --env-file (default .env); values are never printed.

Robot: mock only for now (no motion). --frame <jpeg> is what the mock camera sees.
Locator: --locator fake (one box in the middle of the frame, for flow tests) or
         --locator locate-anything (LocateAnything engine on this machine's GPU).

Examples
  python tools/store_agent.py --chat
  python tools/store_agent.py --serve --port 8099
     curl -s localhost:8099/message -d '{"user": "u1", "text": "do you have paper towels?"}'
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from locatebot.agent.llm import NemotronClient  # noqa: E402
from locatebot.store.agent import StoreAgent, StoreConfig  # noqa: E402
from locatebot.store.catalog import MemoryCatalog  # noqa: E402
from locatebot.store.robot import Box, FakeLocator, MockRobot  # noqa: E402
from locatebot.store.verify import NemotronVerifier  # noqa: E402

MAX_BODY = 16 * 1024


def load_env_file(path: Path) -> None:
    """KEY=value lines into os.environ (existing variables win). Never prints values."""
    if not path.is_file():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


class LocateAnythingLocator:
    """LocateAnything engine (locatebot.vision) -> normalized boxes."""

    def __init__(self, model: Path, lib: Path | None):
        from locatebot.vision.engine import LocateEngine, find_library
        self.engine = LocateEngine(model, lib_path=lib or find_library(ROOT))

    def locate(self, jpeg: bytes, query: str) -> list[Box]:
        from locatebot.vision.postprocess import build_prompt, clean_detections, jpeg_size
        w, h = jpeg_size(jpeg)
        dets = clean_detections(self.engine.locate(jpeg, build_prompt(query)), w, h)
        out = []
        for d in dets:
            x1, y1, x2, y2 = d.box
            x1, x2 = max(0.0, x1 / w), min(1.0, x2 / w)
            y1, y2 = max(0.0, y1 / h), min(1.0, y2 / h)
            if x1 < x2 and y1 < y2:
                out.append(Box(x1, y1, x2, y2))
        return out


def build_agent(args) -> StoreAgent:
    code = args.store_code or os.environ.get("STORE_CODE", "")
    if not code:
        sys.exit("set STORE_CODE in .env or pass --store-code")
    llm = NemotronClient(model=args.model) if args.model else NemotronClient()
    frame = args.frame.read_bytes() if args.frame else None
    if args.locator == "locate-anything":
        locator = LocateAnythingLocator(args.la_model, args.la_lib)
    else:
        locator = FakeLocator([Box(0.35, 0.35, 0.65, 0.65, 0.5)])
    return StoreAgent(llm, MemoryCatalog.from_json(args.catalog), MockRobot(frame=frame), locator,
                      NemotronVerifier(llm), StoreConfig(store_code=code))


def serve(agent: StoreAgent, host: str, port: int) -> None:
    class H(BaseHTTPRequestHandler):
        def _send(self, code: int, obj: dict) -> None:
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path != "/message":
                return self._send(404, {"error": "not found"})
            n = int(self.headers.get("Content-Length") or 0)
            if not 0 < n <= MAX_BODY:
                return self._send(413 if n else 400, {"error": "bad body size"})
            try:
                req = json.loads(self.rfile.read(n))
                user, text = req["user"], req["text"]
                if not (isinstance(user, str) and isinstance(text, str) and user and text.strip()):
                    raise ValueError
            except (ValueError, KeyError, TypeError):
                return self._send(400, {"error": "expected JSON {user, text}"})
            self._send(200, {"reply": agent.handle_text(user, text[:2000])})

    httpd = ThreadingHTTPServer((host, port), H)
    print(f"store agent: POST http://{host}:{httpd.server_address[1]}/message", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--chat", action="store_true", help="talk to the agent in this terminal")
    mode.add_argument("--serve", action="store_true", help="HTTP POST /message {user, text} -> {reply}")
    ap.add_argument("--env-file", type=Path, default=ROOT / ".env")
    ap.add_argument("--catalog", type=Path, default=ROOT / "data" / "mock_store.json")
    ap.add_argument("--store-code")
    ap.add_argument("--model", help="Nemotron model slug")
    ap.add_argument("--frame", type=Path, help="JPEG the mock robot's camera returns (objects only, no people)")
    ap.add_argument("--locator", choices=["fake", "locate-anything"], default="fake")
    ap.add_argument("--la-model", type=Path, default=ROOT / "models" / "locate-anything-q4_k.gguf")
    ap.add_argument("--la-lib", type=Path)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    load_env_file(args.env_file)
    agent = build_agent(args)
    if args.serve:
        serve(agent, args.host, args.port)
        return 0
    print("type a message (Ctrl+D to quit)")
    for line in sys.stdin:
        if line.strip():
            print("bot:", agent.handle_text("terminal", line.rstrip("\n")), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
