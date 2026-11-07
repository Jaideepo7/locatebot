# Shared contracts — proposed v1

**Documentation only: these APIs are proposed, not implemented here.**

## Units and validation
Metres, metres/second, radians positive left, forward bearing zero. Portable timestamps are seconds on one monotonic clock. Capture time survives inference unchanged. Never refresh a stale observation by replacing its timestamp with completion time. Explicit adapters translate device/ROS time and handle clock resets.

Immutable records proposed in `rc_robot/contracts.py`:
| Record | Fields |
|---|---|
| MotionCommand | speed_mps: float; steering_rad: float; stamp_s: float |
| LidarScan | ranges_m: tuple[float, ...]; angle_min_rad: float; angle_increment_rad: float; stamp_s: float; sequence: int |
| TargetObservation | target_id: str; bearing_rad: float; confidence: float; stamp_s: float |
| SafetyDecision | command: MotionCommand; allowed: bool; reason: str |

Records are data, not permission to actuate. Every consumer validates its boundary. Constructors should not conceal unit conversions. No target distance/map position is implied by TargetObservation. Configuration errors raise explicit exceptions; runtime invalid inputs fail closed or are rejected with diagnostics. Never conceal inference errors as successful detections.

## Safety API
`SafetySupervisor(max_speed_mps=0.3, max_steering_rad=0.45, scan_timeout_s=0.25, command_timeout_s=0.25, stop_distance_m=0.6)`.
`evaluate(command: MotionCommand, scan: LidarScan | None, now: float) -> SafetyDecision`.
Positive finite configuration required. Reject reverse, malformed/nonfinite command, future/expired data, empty/invalid scan and insufficient forward coverage. Clamp otherwise valid positive speed/steering. Obstacle at or inside the conservative forward threshold stops. Reasons must distinguish invalid command, stale command, scan fault, stale scan, obstacle and permitted command. Exact reason strings are frozen by the integrator before code.

First synthetic geometry: samples must cover [-pi/2,+pi/2], have finite positive angle increment and finite positive ranges; gaps between rays remain a documented blind-space limitation. Missing return data is not assumed free. Real driver range/quality conventions are a later adapter responsibility. The front-envelope threshold is not a vehicle swept-path model.

## Control API
`FakeVehicle.apply(command: MotionCommand) -> None`; `.history` records applied fake outputs; nonfinite values are rejected.
`WatchdogController(vehicle, timeout_s=0.25, max_speed_mps=0.3, max_steering_rad=0.45)`.
`submit(command: MotionCommand, now: float) -> None`; `tick(now: float) -> MotionCommand`.
Startup zero. Invalid/future/expired/reverse/out-of-limit submissions produce zero instead of retaining an old moving command. Age uses original command stamp, not submission time. Tick applies zero after expiry. This software loop is not independent of process death.

## Perception API
`LatestObservationStore.publish(observation: TargetObservation) -> bool`.
`latest(now: float, max_age_s: float) -> TargetObservation | None`.
`VisionWorker(infer, store)`; `start()`, `submit(frame)`, `close(timeout_s=1.0) -> bool`; `.last_error`.
Publish requires nonempty ID, finite bearing/time, confidence in [0,1]. Reject older/equal stamp. Invalid query time/age cannot yield a target. Inference callable returns an observation or None with preserved capture timestamp. One pending latest frame; submit/read never wait on inference. Exception details become diagnostics. Close disallows later publication and reports whether shutdown completed; an arbitrary hung inference cannot safely be force-cancelled by the thread API. Start/submit/close lifecycle behaviour must be pinned by tests before implementation.

## Mission API
`MissionPlanner(max_speed_mps=0.2, max_steering_rad=0.4, target_timeout_s=1.0, min_confidence=0.5)`.
`step(target: TargetObservation | None, now: float, enabled: bool=True) -> MotionCommand`; `.state`.
States: IDLE when disabled; SEARCHING when no valid target; TRACKING for a valid target. Initial SEARCHING is stationary; physical exploration is deferred. Valid target produces bounded forward intent and bounded steering. Old/future/nonfinite/low-confidence target produces zero. Repeated steps using a still-fresh cached observation remain nonzero without waiting for a camera update. No arrival/map/depth claims.

## Defaults and ambiguity policy
All numbers above are synthetic-test settings, not safe physical values. Timeout boundary: age <= limit is fresh; age > limit is expired; future stamps are invalid. Steering at the configured absolute limit is valid. Confidence equal to minimum is acceptable. Positive speed above safety maximum is clamped by the supervisor; the downstream controller rejects commands that still exceed its independent limit. Clock rollback invalidates state; integrator owns reset policy before real adapters.

## Arbitration and contract changes
Physical cutoff/MCU watchdog > safety constraints > bounded/fresh command > navigation/mission intent > semantic preference. One command owner only. Integrator owns contract updates and shared tests; lanes propose changes with consumer impact and regression tests, never rename fields independently. ROS topic/message selection waits for actual stack compatibility. Do not confuse yaw rate with steering angle.
