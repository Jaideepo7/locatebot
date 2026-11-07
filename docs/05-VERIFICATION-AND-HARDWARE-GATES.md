# Verification, acceptance and hardware gates

**All checks below are planned, not executed by this documentation package.**

## Software evidence rules
Write tests before behaviour, observe the expected failure, implement minimum behaviour and run both lane and full suites. Proposed portable command: `python3 -m unittest discover -s tests -v`. Test success claims require actual output and exit code on the exact reviewed commit. Record interpreter/version. A clean compile or agent self-report does not substitute for behavioural tests.

## Foundation gate
- Immutable contract fields/units/capture timestamps pinned by tests.
- Shared baseline committed and clean in every lane checkout.
- No external network/hardware dependency for portable tests.
- Root ownership rules and ignored worktrees documented.

## Per-lane gate
Use lane-specific failure tests plus full suite. Independently review spec compliance and code quality. Report exact base/head commits, actual commands, results, unimplemented features and limitations. Boundary cases: invalid/future/nonfinite time, exact timeout threshold, stale data, reverse/limits, missing scan coverage, asynchronous lifecycle.

## Cross-lane acceptance scenarios
| Scenario | Expected result |
|---|---|
| Fresh target + clear synthetic scan | Bounded forward command reaches fake vehicle |
| Inference blocked using Event, cached target still fresh | Mission/safety/control complete without awaiting inference |
| Target becomes too old | Required-target policy stops; no stale semantic motion |
| Obstacle enters front envelope | Safety outputs zero regardless of target/model state |
| LiDAR missing/frozen/expired | Safety outputs zero |
| Command expires or is invalid | Controller outputs zero, not old motion |
| Inference raises | Diagnostic recorded; control path remains responsive |
| Worker closes during inference | No late publication; bounded close reports outcome |
| Out-of-order inference result | Cannot replace a newer observation |

Use deterministic clocks and synchronization, not fragile sleep-based timing tests. Passing synthetic tests proves software behaviour under those inputs, not physical obstacle coverage or actual real-time latency.

## Real sensor gate
Identify driver/version and documented no-return conventions; verify forward/left/right mounting and units; compare known physical distances; record capture timestamps/quality/sequence; unplug and freeze data deliberately. Driver must not refresh a timestamp for old scan contents. Verify blind zones and coverage. Preserve replay evidence and test raw-to-portable conversion.

## Motor-enablement gate — all mandatory
- [ ] Exact ESC/servo/electrical interface identified; no guessed neutral/brake values.
- [ ] Vehicle footprint, wheelbase, steering limits and turning radius measured.
- [ ] Independent MCU timeout verified on host crash and cable removal.
- [ ] Physical cutoff accessible and tested.
- [ ] Wheels-raised startup, shutdown and communications-loss checks complete.
- [ ] Voltage/current/connector/ground/noise checks complete.
- [ ] Sensor freshness and required field-of-view checks complete.
- [ ] Measured low-speed braking plus sensing/control latency used for stopping envelope.
- [ ] Restricted area, soft obstacles and human supervision arranged.
- [ ] Reverse disabled until rear sensing/recovery checks pass.
A Python watchdog called by the host is not an independent fail-safe. Never test by approaching a person.

## Navigation gate
Odometry and calibrated transforms verified; localization health gates motion; car-compatible planner/controller respects turning limits; unreachable goals and blocked paths produce explicit outcomes. Target has validated spatial position/uncertainty and reachable stand-off pose. No object-arrival claim from camera bearing alone.

## Model gate
Record actual model/backend/version/license source, device/software, input size, load success, memory use, latency measurements and target-quality evaluation. No invented model responses or published performance estimates presented as measurements. Slow inference must not block motion control.

## Release gate
Reviewed integration commit; full suite and synthetic demo pass; physical checks recorded separately if hardware enabled; known limitations listed; no secrets or private recordings included. Keep a last-passing milestone and rollback path. No remote release/push without destination and explicit authority.
