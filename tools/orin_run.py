#!/usr/bin/env python3
"""Run the Orin side: live mapping, skills, optional LLM brain and vision, operator page.

Sources (pick one):
  --sim                       simulated room + car, closed loop (demo / no hardware)
  --lidar-port /dev/ttyUSB0   LiDAR plugged into the Orin (bench): maps live; drive is a dry run
  --pi-host <pi-address>      the car: scans/frames from the Pi, drive commands back (ZeroMQ)

LLM (optional):
  --llm none     operator only: send skills from the page (default)
  --llm claude   Anthropic API, key from the ANTHROPIC_API_KEY env var (needs internet)
  --llm local    OpenAI-compatible server, e.g. Ollama: --llm-url http://127.0.0.1:11434/v1 --llm-model <m>
  --llm nemotron NVIDIA API (OpenAI-compatible, https://integrate.api.nvidia.com/v1), key from the
                 NVIDIA_API_KEY env var, model id from build.nvidia.com: --llm-model <id> (required)
  --mode approve (default): the operator approves each LLM call on the page; --mode auto runs them.

Vision (optional, needs camera frames, i.e. --pi-host): --vision [--model ... --lib ...]

Saved map (optional): --map maps/<name>.lbmap [--start-pose x,y,deg] [--save-map]
  localizes in the saved map instead of starting a new one (no --start-pose: global search),
  updates it slowly and on exit writes maps/<name>.v2.lbmap (--save-map: overwrites <name>.lbmap).

Examples
  python tools/orin_run.py --sim --port 8090
  python tools/orin_run.py --lidar-port /dev/ttyUSB0 --host 0.0.0.0
  python tools/orin_run.py --pi-host 10.0.0.2 --vision --llm claude --mode approve --host 0.0.0.0
Then open http://<orin-address>:8090/ . Ctrl+C stops everything and sends a zero drive.
"""
from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import threading
import time
import uuid
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from locatebot.lidar.types import Pose2D  # noqa: E402
from locatebot.mission.types import VehicleConfig  # noqa: E402
from locatebot.orin.app import AppConfig, OrinApp, PeerClock, make_server  # noqa: E402
from locatebot.orin.live_map import LiveMapper  # noqa: E402
from locatebot.orin.runtime import OrinRuntime  # noqa: E402
from locatebot.orin.skills import SkillExecutor  # noqa: E402

log = logging.getLogger("orin_run")


# ---------------------------------------------------------------------- sources
class SimSource:
    """Simulated 6 x 5 m room with a table and two items; the car follows the drive commands."""

    def __init__(self, vehicle: VehicleConfig):
        from locatebot.lidar.sim import Room
        from locatebot.nav.sim import BicycleSim
        self.room = (Room.box(6.0, 5.0).with_box(2.6, 2.0, 3.4, 2.8)
                     .with_box(3.67, 2.32, 3.83, 2.48).with_box(5.22, 4.42, 5.38, 4.58))
        self.car = BicycleSim(vehicle, Pose2D(1.0, 1.0, 0.2), dt=0.05)
        self.lock = threading.Lock()
        self.last = time.monotonic()

    def drive(self, throttle: float, steering: float) -> None:
        with self.lock:
            now = time.monotonic()
            dt, self.last = min(0.2, now - self.last), now
            self.car.dt = max(1e-3, dt)
            self.car.step(throttle, steering)

    def run(self, app: OrinApp, stop: threading.Event) -> None:
        from locatebot.lidar.sim import simulate_scan
        rng = np.random.default_rng(0)
        seq = 0
        while not stop.is_set():
            with self.lock:
                pose = self.car.pose
            app.on_scan(simulate_scan(self.room, pose, n_beams=508, noise_std_m=0.005, rng=rng,
                                      stamp_s=time.monotonic(), seq=seq, dropout=0.2))
            seq += 1
            stop.wait(0.1)


def lidar_source(app: OrinApp, stop: threading.Event, port: str, baud: int, mount_offset_rad: float) -> None:
    from locatebot.lidar.rplidar import RPLidar
    while not stop.is_set():
        try:
            with RPLidar(port, baudrate=baud, mount_offset_rad=mount_offset_rad) as lidar:
                status, code = lidar.get_health()
                log.info("lidar health %s/%s, info %s", status, code, lidar.get_info())
                for scan in lidar.iter_scans():
                    app.on_scan(scan)
                    if stop.is_set():
                        break
        except Exception as e:  # unplugged, stalled, permission: retry
            log.warning("lidar error: %s; retrying in 1 s", e)
            stop.wait(1.0)


def pi_source(app: OrinApp, stop: threading.Event, host: str, clock: PeerClock) -> None:
    from locatebot.lidar.types import Scan
    from locatebot.link.messages import Frame, PiStatus, Range
    from locatebot.link.transport import Subscriber, pi_pub_endpoint
    topics = ["scan", "frame", "pi_status", "range", "mission_request", "mission_cancel"]  # missions: never newest-only
    with Subscriber(pi_pub_endpoint(host), topics, newest_only=("frame",)) as sub:
        while not stop.is_set():
            for msg in sub.poll(0.2):
                if app.runtime.missions is not None and app.runtime.missions.on_message(msg):
                    continue
                if isinstance(msg, Scan):
                    clock.observe(msg.stamp_s)
                    app.on_scan(msg)
                elif isinstance(msg, Frame):
                    app.on_frame(msg.jpeg, msg.capture_s)
                elif isinstance(msg, PiStatus):
                    app.runtime.set_safety_reason(msg.safety_reason)
                elif isinstance(msg, Range):  # M3: ultrasonic -> glass_or_low layer (saved-map mode)
                    app.runtime.on_range(msg)


# Random per process start; OrinStatus carries it so the Pi resets its drive seq filter after a restart.
BOOT_ID = uuid.uuid4().hex


class PiDriveSink:
    """Publishes drive commands (and Orin status at 1 Hz) on the Orin's PUB socket."""

    def __init__(self, runtime: OrinRuntime):
        from locatebot.link.transport import Publisher, bind_endpoint
        self.pub = Publisher(bind_endpoint(5556))
        self.runtime = runtime
        self.seq = 0
        self.last_status = 0.0
        self.boot_id = BOOT_ID
        self._queue = deque()  # MissionStatus from the control thread; only this sink's thread sends

    def queue(self, msg) -> None:
        """Thread-safe: hand a message (e.g. MissionStatus) to the sender thread."""
        self._queue.append(msg)

    def __call__(self, throttle: float, steering: float) -> None:
        from locatebot.link.messages import DriveCmd, OrinStatus
        self.seq += 1
        self.pub.send(DriveCmd(float(throttle), float(steering), self.seq, time.monotonic()))
        while self._queue:
            self.pub.send(self._queue.popleft())
        now = time.monotonic()
        if now - self.last_status >= 1.0:
            self.last_status = now
            v = self.runtime.vision.status() if self.runtime.vision is not None else None
            self.pub.send(OrinStatus(now, bool(v and v.get("alive")), v.get("last_latency_s") if v else None,
                                     self.runtime.executor.status.state.value, None, None, self.boot_id))

    def close(self) -> None:
        self.pub.close()


class DryRunSink:
    """No motors: log the requested drive once a second when it is non-zero."""

    def __init__(self):
        self.last = 0.0

    def __call__(self, throttle: float, steering: float) -> None:
        now = time.monotonic()
        if (throttle or steering) and now - self.last >= 1.0:
            self.last = now
            log.info("dry run: would drive throttle %.2f steering %.2f", throttle, steering)


# ---------------------------------------------------------------------- wiring
NVIDIA_API_URL = "https://integrate.api.nvidia.com/v1"


def make_llm(args):
    if args.llm == "none":
        return None
    from locatebot.agent.llm import ClaudeClient, OpenAICompatClient
    if args.llm == "claude":
        return ClaudeClient(model=args.llm_model) if args.llm_model else ClaudeClient()
    if args.llm == "nemotron":
        if not args.llm_model:
            raise SystemExit("--llm nemotron needs --llm-model <model id from build.nvidia.com>")
        return OpenAICompatClient(base_url=args.llm_url or NVIDIA_API_URL, model=args.llm_model,
                                  api_key_env="NVIDIA_API_KEY")
    return OpenAICompatClient(base_url=args.llm_url or "http://127.0.0.1:11434/v1",
                              model=args.llm_model or "qwen2.5:7b-instruct")


def parse_pose(text: str) -> Pose2D:
    x, y, deg = (float(v) for v in text.split(","))
    return Pose2D(x, y, math.radians(deg))


def make_mapper(ap, args) -> LiveMapper:
    if args.map is None:
        if args.start_pose or args.save_map:
            ap.error("--start-pose/--save-map need --map")
        return LiveMapper(size_m=args.map_size)
    try:
        guess = parse_pose(args.start_pose) if args.start_pose else None
    except ValueError:
        ap.error("--start-pose must be x,y,deg")
    lm = LiveMapper.from_saved(args.map, guess=guess)
    log.info("localizing in %s (%s)", args.map, "global search" if guess is None else f"guess {guess}")
    return lm


def wifi_source(mapper: LiveMapper, stop: threading.Event, clock, iface: str | None = None) -> None:
    """M3: 1 Hz Wi-Fi RSSI into the wifi_rssi layer (saved-map mode). No Wi-Fi tool: disabled."""
    from locatebot.orin.wifi import WifiReader
    reader = WifiReader(iface)
    if not reader.enabled:
        log.info("wifi layer disabled: no /proc/net/wireless interface and no iw")
        return
    log.info("wifi layer: %s via %s", reader.iface, reader.mode)
    while not stop.wait(1.0):
        mapper.add_wifi(reader.read(), clock())


def map_stem(path: Path) -> Path:
    name = path.name[:-len(".lbmap")] if path.name.endswith(".lbmap") else path.name
    return path.with_name(name)


def save_localized_map(args, mapper: LiveMapper) -> None:
    """--map sessions: keep the slowly updated map (never when it never localized)."""
    if args.map is None:
        return
    st = mapper.stats()
    if not st.get("integrated"):
        print("map not saved: never localized", flush=True)
        return
    stem = map_stem(args.map)
    stem = stem if args.save_map else stem.with_name(stem.name + ".v2")
    paths = mapper.save(stem, meta={"saved_by": "orin_run"})
    print(f"updated map: {paths['lbmap']}", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sim", action="store_true")
    src.add_argument("--lidar-port")
    src.add_argument("--pi-host")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--mount-offset-deg", type=float, default=0.0)
    ap.add_argument("--llm", choices=["none", "claude", "local", "nemotron"], default="none")
    ap.add_argument("--llm-model")
    ap.add_argument("--llm-url", help="override the server URL (local: Ollama default; nemotron: NVIDIA API)")
    ap.add_argument("--mode", choices=["approve", "auto"], default="approve")
    ap.add_argument("--vision", action="store_true", help="run LocateAnything on frames from the Pi")
    ap.add_argument("--model", type=Path, default=ROOT / "models" / "locate-anything-q4_k.gguf")
    ap.add_argument("--lib", type=Path)
    ap.add_argument("--hfov-deg", type=float, default=70.0, help="camera horizontal field of view")
    ap.add_argument("--map-size", type=float, default=20.0)
    ap.add_argument("--map", type=Path, help="saved .lbmap to localize in (instead of a new map)")
    ap.add_argument("--start-pose", help="with --map: where the car is in that map, x,y,deg")
    ap.add_argument("--save-map", action="store_true", help="with --map: overwrite it on exit")
    ap.add_argument("--wifi-iface", help="with --map: Wi-Fi interface for the coverage layer (default: auto)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--maps-dir", default=str(ROOT / "maps"))
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    vehicle = VehicleConfig()
    clock = PeerClock() if args.pi_host else time.monotonic
    runtime = OrinRuntime(make_mapper(ap, args), SkillExecutor(vehicle), clock=clock)

    llm = make_llm(args)
    if llm is not None:
        from locatebot.agent.brain import Brain
        runtime.brain = Brain(llm, mode=args.mode, on_skill=runtime.on_skill, clock=clock)

    if args.vision:
        if not args.pi_host:
            ap.error("--vision needs camera frames from the Pi (--pi-host)")
        from locatebot.vision.engine import LocateEngine, find_library
        from locatebot.vision.service import VisionService
        engine = LocateEngine(args.model, lib_path=args.lib or find_library(ROOT))
        runtime.vision = VisionService(engine, on_result=runtime.on_vision_result,
                                       hfov_rad=math.radians(args.hfov_deg))
        runtime.vision.start()

    sim = SimSource(vehicle) if args.sim else None
    sink = sim.drive if sim else (PiDriveSink(runtime) if args.pi_host else DryRunSink())
    app = OrinApp(runtime, sink, clock=clock, config=AppConfig(maps_dir=args.maps_dir))
    if args.pi_host:  # Telegram/operator missions from the Pi (docs/remote/README.md)
        from locatebot.orin.missions import MissionBridge
        runtime.missions = MissionBridge(runtime, sink.queue)

    stop = threading.Event()
    if sim:
        target, targs = sim.run, (app, stop)
    elif args.lidar_port:
        target, targs = lidar_source, (app, stop, args.lidar_port, args.baud, math.radians(args.mount_offset_deg))
    else:
        target, targs = pi_source, (app, stop, args.pi_host, clock)
    source = threading.Thread(target=target, args=targs, name="source", daemon=True)

    server = make_server(app, args.host, args.port)
    web = threading.Thread(target=server.serve_forever, name="http", daemon=True)

    def shutdown(*_):
        stop.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    source.start()
    if args.map is not None and not args.sim:
        threading.Thread(target=wifi_source, args=(runtime.mapper, stop, clock, args.wifi_iface),
                         name="wifi", daemon=True).start()
    app.start()
    web.start()
    host, port = server.server_address[:2]
    print(f"operator page: http://{host}:{port}/  (llm={args.llm}, mode={args.mode})", flush=True)
    t_end = None if args.seconds is None else time.monotonic() + args.seconds
    try:
        while not stop.is_set():
            if t_end is not None and time.monotonic() >= t_end:
                break
            stop.wait(0.2)
    finally:
        stop.set()
        app.stop()
        server.shutdown()
        server.server_close()
        if runtime.vision is not None:
            runtime.vision.stop(timeout=2.0)
        if hasattr(sink, "close"):
            sink.close()
        save_localized_map(args, runtime.mapper)
        st = app.status()
        print(f"stopped: {st['map']['integrated']} scans mapped, skill {st['skill']['state']}"
              f" ({st['skill']['detail'] or '-'}), max step {st['app']['max_step_ms']} ms", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
