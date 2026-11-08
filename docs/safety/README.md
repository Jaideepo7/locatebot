# Safety supervisor (Pi)

**Status:** decision logic built and tested on synthetic scans. Not yet wired to a motor driver or run on the car.

`locatebot/safety/supervisor.py`: `SafetySupervisor(SafetyConfig).evaluate(command, scan, now) -> SafetyDecision`.
Call it every control tick on the Pi with the newest scan and drive command (both stamped with the Pi's monotonic clock); send only the returned throttle/steering to the motors.

## Policy
- First failing check wins, in order: e-stop (latched) → clock → scan missing/stale/future → command missing/stale/future → invalid command → reverse → forward coverage → obstacle.
- Every stop is throttle 0, steering 0, with a distinct `Reason`.
- Unknown is never free: below `min_coverage` (angular bins with a valid return over `coverage_fov_rad`) the car stops. When moving, every 2° bin of the corridor ahead (its angular half-width at `stop_distance_m`, for each checked steering direction) must also have a return (`corridor_min_coverage`), so a small blind wedge (black, glass or absorbing object) can't hide behind high global coverage. `max_steer_rad` is validated so the steered corridor stays inside `coverage_fov_rad`.
- Non-numeric command values are `INVALID_COMMAND`, never an exception. The Pi control loop must still treat any exception around `evaluate` as a stop.
- The forward corridor is checked straight ahead and rotated towards the steering direction, so turning can't dodge an obstacle.
- Clearance ≤ `stop_distance_m` stops; up to `slow_distance_m` the throttle cap rises linearly to `max_speed`. Requested speed is never raised.
- Reverse is always blocked (no rear policy yet). Zero throttle on a healthy scan is allowed (steer in place).

## Reasons not produced by `evaluate`
`Reason` is the one enum for every stop reason the Pi reports (`PiStatus.safety_reason` sends its `.value`). Two members are appended for callers outside `evaluate`; existing values never change:
- `Reason.FAULT` (`"fault"`): the Pi control loop (`locatebot/pi/control.py`, `PiController.tick`) caught an exception and stopped the motor.
- `Reason.ULTRASONIC` (`"ultrasonic"`): reserved for the front ultrasonic fusion (ROADMAP T1.2); nothing produces it yet.

## Corridor coverage needs dense scans
The corridor gate needs a valid return in every ~2° bin, so a LiDAR with beam spacing coarser than about 1.9° can never pass it. The RPLIDAR C1 (~0.07° spacing) is far above that, but glossy or black surfaces, or open space beyond 12 m straight ahead, can empty a bin and stop the car. Measure how often `LOW_COVERAGE` fires on clean scans on the real car; if it is too often, add hysteresis (N consecutive bad scans) for the coverage stop only. Never relax the obstacle stop.

## Before trusting it on the car
The defaults (0.40 m stop, 1.0 m slow, 0.5 s scan / 0.3 s command timeouts, 0.15 m half-width) are placeholders.
1. Measure braking distance at the capped speed and set `stop_distance_m` from it plus sensing latency and margin.
2. Measure the car's width and set `half_width_m`.
3. Unplug / freeze the LiDAR and confirm the motors stop, with a physical kill switch in hand.
4. Test turns towards soft obstacles at low speed.
5. Record the `LOW_COVERAGE` rate while driving slowly in the demo room.

Limits: 2D scan misses obstacles above/below the LiDAR plane, glass and drop-offs; the rotated corridor approximates the swept arc; a driver that invents fresh timestamps can't be detected from scan age alone.
