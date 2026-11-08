#!/usr/bin/env python3
"""Run the Pi's Telegram remote-control service (Photon inbound + replies + local operator API).

Loads `.env` (git-ignored; see `.env.example`) without overriding variables already set, then:
  - inbound: Photon WebSocket delivery (default, no public URL needed) or the webhook route
    (`PHOTON_INBOUND=webhook`, needs a public HTTPS tunnel to --host/--port);
  - the local HTTP server (`/healthz`, `/api/photon/webhook`, `/api/missions*`) on --host:--port;
  - the reply outbox sender and the mission timeouts (background thread);
  - the robot link: `--robot fake` (scripted stand-in for the Orin) or `--robot zmq --orin-host <addr>`
    (missions over the ZeroMQ link: Pi PUB on --pi-bind:5555, Orin PUB <addr>:5556).
The robot keeps working without internet: inbound reconnects quietly and replies stay queued.

Examples:
  .venv/bin/python tools/remote_run.py                                  # .env, WebSocket inbound
  .venv/bin/python tools/remote_run.py --console                        # print replies, don't send
  PHOTON_INBOUND=webhook .venv/bin/python tools/remote_run.py --console --robot fake --fake-script happy
  .venv/bin/python tools/remote_run.py --robot zmq --orin-host <orin-address>     # the car
Ctrl+C or SIGTERM stops cleanly. --debug logs RSS every 60 s.
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
from pathlib import Path
from typing import Dict, Mapping, MutableMapping, Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from locatebot.remote.config import ConfigError, Settings  # noqa: E402
from locatebot.remote.robot import SCRIPTS, FakeRobotLink  # noqa: E402
from locatebot.remote.server import make_server  # noqa: E402
from locatebot.remote.service import RemoteService  # noqa: E402

log = logging.getLogger("remote_run")
RSS_EVERY_S = 60.0


def parse_env_file(path: Path) -> Dict[str, str]:
    """Minimal .env reader: KEY=VALUE lines, # comments, optional quotes. No interpolation."""
    out: Dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        if line.startswith("export "):
            line = line[len("export "):]
        key, _, value = line.partition("=")
        key, value = key.strip(), value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        elif " #" in value:
            value = value.split(" #", 1)[0].rstrip()
        if key:
            out[key] = value
    return out


def load_env(path: Optional[Path], env: MutableMapping[str, str]) -> None:
    if path is None or not path.exists():
        return
    for k, v in parse_env_file(path).items():
        env.setdefault(k, v)


def rss_mb() -> Optional[float]:
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return None


def build(args, env: Mapping[str, str]):
    settings = Settings.from_env(env)
    if args.robot == "fake":
        link = FakeRobotLink(args.fake_script)
    else:
        from locatebot.link.messages import PI_PUB_PORT
        from locatebot.link.transport import bind_endpoint, orin_pub_endpoint
        from locatebot.remote.robot_zmq import ZmqRobotLink
        try:
            orin = orin_pub_endpoint(args.orin_host)  # or $LOCATEBOT_ORIN_HOST
        except ValueError as e:
            raise ConfigError(f"--robot zmq: {e}") from None
        link = ZmqRobotLink(orin, pub_endpoint=bind_endpoint(PI_PUB_PORT, args.pi_bind))
    service = RemoteService(settings, link=link)  # PHOTON_MODE=console (or --console) prints replies
    server = make_server(service, args.host, args.port)
    return service, server


def run(service: RemoteService, server, stop: threading.Event) -> None:
    host, port = server.server_address[:2]
    http = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.2}, name="remote-http",
                            daemon=True)
    service.start()
    http.start()
    print(f"remote service listening on http://{host}:{port} (inbound={service.settings.photon_inbound}, "
          f"robot={type(service.link).__name__})", flush=True)
    try:
        while not stop.wait(RSS_EVERY_S):
            if log.isEnabledFor(logging.DEBUG):
                log.debug("rss %.1f MB, health %s", rss_mb() or -1, service.health())
    finally:
        server.shutdown()
        server.server_close()
        service.stop()
        log.info("remote service stopped")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--env-file", default=str(ROOT / ".env"), help="KEY=VALUE file (default: repo .env)")
    ap.add_argument("--host", default="127.0.0.1", help="operator/webhook HTTP bind host (default 127.0.0.1)")
    ap.add_argument("--port", type=int, default=8070)
    ap.add_argument("--robot", choices=["fake", "zmq"], default="fake", help="robot link")
    ap.add_argument("--orin-host", help="--robot zmq: Orin address (or $LOCATEBOT_ORIN_HOST)")
    ap.add_argument("--pi-bind", default="*", help="--robot zmq: interface for the Pi PUB socket (port 5555)")
    ap.add_argument("--fake-script", choices=sorted(SCRIPTS), default="happy")
    ap.add_argument("--console", action="store_true", help="print replies instead of sending them")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("locatebot.remote.photon_ws.wire").setLevel(logging.INFO)
    env = dict(os.environ)
    load_env(Path(args.env_file) if args.env_file else None, env)
    if args.console:
        env["PHOTON_MODE"] = "console"
    try:
        service, server = build(args, env)
    except ConfigError as e:
        print(e, file=sys.stderr)
        return 2
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    run(service, server, stop)
    return 0


if __name__ == "__main__":
    sys.exit(main())
