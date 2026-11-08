#!/usr/bin/env python3
"""Pi control loop + local manual-drive page (step 1: manual drive + E-stop).

Bench (no hardware):   python tools/pi_drive.py --sim
With the real LiDAR:   python tools/pi_drive.py --lidar-port /dev/ttyUSB0 --host <pi-lan-address>

The motor is always a FakeMotor until the real motor/steering driver exists (see
docs/pi/README.md). Ctrl+C (or SIGTERM) stops the motor, the LiDAR and the server.
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from locatebot.lidar.sim import Room, simulate_scan  # noqa: E402
from locatebot.lidar.types import Pose2D  # noqa: E402
from locatebot.pi.control import ControlConfig, PiController  # noqa: E402
from locatebot.pi.motor import FakeMotor  # noqa: E402
from locatebot.pi.server import make_server  # noqa: E402
from locatebot.safety.supervisor import SafetyConfig, SafetySupervisor  # noqa: E402


def sim_scans(ctl: PiController, stop: threading.Event, obstacle_m: float | None, rate_hz: float = 10.0) -> None:
    """Synthetic scans of a static 6 x 4 m room; the simulated car never moves."""
    pose = Pose2D(1.0, 2.0, 0.0)
    room = Room.box(6.0, 4.0)
    if obstacle_m is not None:
        room = room.with_box(pose.x + obstacle_m, pose.y - 0.1, pose.x + obstacle_m + 0.1, pose.y + 0.1)
    rng = np.random.default_rng(0)
    seq = 0
    while not stop.is_set():
        ctl.update_scan(simulate_scan(room, pose, n_beams=720, noise_std_m=0.005, rng=rng,
                                      stamp_s=time.monotonic(), seq=seq))
        seq += 1
        stop.wait(1.0 / rate_hz)


def lidar_scans(ctl: PiController, stop: threading.Event, port: str, baud: int, mount_offset_rad: float) -> None:
    """Feed real RPLIDAR scans (newest wins). On any error, log, close and retry; while the
    LiDAR is down the supervisor stops the car on scan staleness."""
    from locatebot.lidar.rplidar import RPLidar

    while not stop.is_set():
        try:
            with RPLidar(port, baudrate=baud, mount_offset_rad=mount_offset_rad) as lidar:
                scans = lidar.iter_scans(scan_timeout_s=2.0)
                try:
                    for scan in scans:
                        ctl.update_scan(scan)
                        if stop.is_set():
                            break
                finally:
                    scans.close()  # sends STOP to the sensor
        except Exception as e:  # noqa: BLE001 - keep retrying; the car is stopped meanwhile
            print(f"lidar error: {type(e).__name__}: {e}; retrying in 1 s", file=sys.stderr, flush=True)
            stop.wait(1.0)


def main(argv: list[str] | None = None, motor=None, stop: threading.Event | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sim", action="store_true", help="synthetic scans, no hardware")
    src.add_argument("--lidar-port", help="RPLIDAR serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=460800, help="460800 for C1, 115200 for A1M8")
    ap.add_argument("--mount-offset-deg", type=float, default=0.0, help="sensor zero relative to robot forward, CCW")
    ap.add_argument("--sim-obstacle-m", type=float, default=None, help="--sim: put a box this far ahead")
    ap.add_argument("--host", default="127.0.0.1", help="page bind address (the Pi's LAN address at the event)")
    ap.add_argument("--port", type=int, default=8080, help="page port (0 = any free port)")
    ap.add_argument("--rate", type=float, default=20.0, help="control loop rate, Hz")
    ap.add_argument("--speed-limit", type=float, default=0.3, help="initial manual throttle scale, (0, 1]")
    ap.add_argument("--seconds", type=float, default=None, help="exit after this long (default: until Ctrl+C)")
    ap.add_argument("--quiet", action="store_true", help="no periodic status line")
    args = ap.parse_args(argv)
    if not (math.isfinite(args.rate) and 1.0 <= args.rate <= 200.0):
        ap.error("--rate must be in [1, 200] Hz")

    stop = stop or threading.Event()
    motor = motor if motor is not None else FakeMotor()
    ctl = PiController(SafetySupervisor(SafetyConfig()), motor, ControlConfig(speed_limit=args.speed_limit))
    server = make_server(ctl, args.host, args.port)
    threads = [threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1},
                                name="http", daemon=True)]
    if args.sim:
        threads.append(threading.Thread(target=sim_scans, args=(ctl, stop, args.sim_obstacle_m),
                                        name="scans", daemon=True))
    else:
        threads.append(threading.Thread(target=lidar_scans, name="scans", daemon=True,
                                        args=(ctl, stop, args.lidar_port, args.baud,
                                              math.radians(args.mount_offset_deg))))

    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, lambda *_: stop.set())

    host, port = server.server_address[:2]
    print(f"manual drive page: http://{host}:{port}/  (motor: {type(motor).__name__})", flush=True)
    period = 1.0 / args.rate
    t_end = None if args.seconds is None else time.monotonic() + args.seconds
    try:
        for th in threads:
            th.start()
        next_t = time.monotonic()
        last_print = 0.0
        while not stop.is_set():
            ctl.tick()
            now = time.monotonic()
            if t_end is not None and now >= t_end:
                break
            if not args.quiet and now - last_print >= 2.0:
                st = ctl.status()
                age = st["scan_age_s"]
                print(f"estop={st['estop']} src={st['active_source']} reason={st['reason']} "
                      f"out=({st['throttle']:.2f},{st['steering']:.2f}) scans/s={st['scans_per_s']:.1f} "
                      f"scan_age={'-' if age is None else f'{age:.2f}'}", flush=True)
                last_print = now
            next_t += period
            if next_t < now - period:  # fell behind (slow tick): don't burst to catch up
                next_t = now
            stop.wait(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ctl.close()  # motor stop first
        server.shutdown()
        server.server_close()
        for th in threads:
            if th.is_alive():
                th.join(timeout=3.0)
        print("stopped: motor stopped, server closed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
