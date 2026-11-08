# Pi runtime (`locatebot/pi/`)

**Status:** step 1 (manual drive + E-stop) built and tested in software only: synthetic scans, `FakeMotor`, loopback HTTP. **Nothing here has driven a real motor. Physical safety is unverified.**

## Design
Every control tick (20 Hz in `tools/pi_drive.py`):

```text
newest scan ─┐
manual (page) ─┼─▶ PiController picks one command ─▶ SafetySupervisor.evaluate ─▶ MotorDriver.set
mission (Orin) ┘   (E-stop > safety stop > manual > mission)
```

- **Re-stamp on receipt.** Manual and mission commands get the Pi's monotonic time when they arrive; the Orin's `sent_s` is never compared with Pi time. The supervisor stops the car once a command is older than `command_timeout_s` (0.3 s): that is hold-to-drive.
- **Manual takeover.** A manual command keeps manual in control for `manual_hold_s` (1 s) after it arrives. Mission commands arriving in that window are ignored (counted as `mission_ignored`). A stale manual command means *stop*, not *hand back*. After the hold, only mission commands received after the last manual command can drive.
- **Mission replay.** `seq <= last_seq` is dropped (`mission_dropped`). Call `reset_mission_seq()` when the Orin's status shows it restarted; that also drops the held mission command.
- **E-stop.** `engage_estop(source)` latches from anywhere and calls `motor.stop()` at once (no wait for the next tick). A network `EStopCmd(engage=False)` is ignored (`network_release_ignored`). Only `release_estop_local()` releases, and it clears every held command, so the car stays still until a new command arrives.
- **Fail closed.** Any exception in `tick()` (clock, supervisor, motor) calls `motor.stop()` and returns a stop with reason `fault`; the error is in `status()["fault"]`. If `motor.stop()` raises too, the error is recorded but not propagated. After `close()`, ticks don't touch the motor.
- **Speed limit.** Manual throttle is multiplied by the speed limit (default 0.3, slider on the page). The supervisor's `max_speed` still caps everything. Mission commands are not scaled.

## API
- `motor.py`: `MotorDriver` protocol (`set(throttle, steering)`, `stop()`, `close()`), `FakeMotor` (records `outputs`, `stop_count`, `closed`, `last`).
- `control.py`: `ControlConfig(manual_hold_s=1.0, speed_limit=0.3, scan_rate_window_s=2.0)`; `PiController(supervisor, motor, config, clock=time.monotonic)` with `update_scan`, `submit_manual`, `submit_mission(DriveCmd) -> bool`, `reset_mission_seq`, `set_speed_limit`, `engage_estop(source)`, `handle_estop(EStopCmd)`, `release_estop_local`, `tick() -> SafetyDecision`, `close`, `status() -> dict`, `pi_status(camera_ok, cpu_temp_c) -> PiStatus`. A fault in `tick` reports the supervisor's `Reason.FAULT` (`"fault"`); `PiReason.CLOSED` (`"closed"`) is the controller's own reason after `close()`, same shape as `Reason`.
- `server.py`: `make_server(controller, host="127.0.0.1", port=8080)` returns a `ThreadingHTTPServer`. Endpoints: `GET /` (one self-contained page, no external assets, CSP blocks any), `GET /status`, `POST /drive {throttle, steering}`, `POST /estop`, `POST /release {"confirm": true}`, `POST /speed {limit}`. `/drive`, `/release` and `/speed` need `Content-Type: application/json` (blocks cross-origin form/text posts from other tabs). A rejected `/drive` replaces the held manual command with a stop.
- The page: big E-STOP fixed at the top (Space also engages), hold buttons, WASD/arrow keys, a touch joystick, speed slider, status panel polled every 300 ms. While a control is held it posts `/drive` every 100 ms; on release, blur, `visibilitychange` (hidden) or `pagehide` it posts a zero command. If the page dies, the server-side timeout stops the car.

## Run on the bench
```bash
.venv/bin/python tools/pi_drive.py --sim                       # synthetic room, open http://127.0.0.1:8080/
.venv/bin/python tools/pi_drive.py --sim --sim-obstacle-m 0.3  # obstacle ahead: driving shows "obstacle"
.venv/bin/python tools/pi_drive.py --lidar-port /dev/ttyUSB0 --host <pi-lan-address>
```
`--host` defaults to `127.0.0.1`; at the event set it to the Pi's own address on the team router. There is no login: anyone who can reach the port can drive. Never put real addresses in the repo. The LiDAR thread keeps the newest scan, retries after any error, and while it is down the supervisor stops the car (`scan_stale`). Ctrl+C or SIGTERM stops the motor first, then the server and LiDAR.

## Needs the real car
- **Motor driver: placeholder.** The motor/steering interface (ESC + servo PWM, H-bridge or motor HAT) is unknown, so only `FakeMotor` exists and `pi_drive.py` always uses it. The real driver must clamp to [-1, 1], make `stop()` neutral + centred, and rely on a hardware failsafe (ESC signal-loss failsafe or watchdog), because a hung Python process can't stop anything.
- The page's JavaScript has only been syntax-checked, not run in a browser here. Test it on the laptop and phone that will be used.
- Bench tests with the wheels off the ground and a physical kill switch in hand: hold-to-drive stops within the timeout when you release; closing the tab, switching apps and pulling Wi-Fi stop the car; E-stop from the page stops at once; release does not move the car; unplugging the LiDAR stops the car; killing `pi_drive.py` (Ctrl+C, SIGTERM, `kill -9`) stops it (`kill -9` needs the hardware failsafe).
- Measure the loop's real tick rate and latency on the Pi 4 (supervisor evaluation on ~10 000-point C1 scans).
- Safety distances and timeouts are placeholders until measured (see `docs/safety/README.md`).
