> **Superseded:** the safety supervisor is built in `locatebot/safety/supervisor.py`; see `docs/safety/README.md`. This file is the original plan and its paths are outdated.

# Lane A — LiDAR validation and safety supervisor

**Status: planned only. Branch: feat/safety. Human/agent owner: unassigned.**
Read architecture and shared contracts first. Allowed future files: rc_robot/safety.py, tests/test_safety.py, docs/lanes/safety.md. Do not modify contracts, actuation, inference or mission code.

## Deliverable
A deterministic, fail-closed SafetySupervisor.evaluate(command, scan, now) returning SafetyDecision. It constrains motion intent; it does not classify objects, map the room or guarantee physical collision avoidance.

## S1 — Reject unhealthy commands and scans
- [ ] Define tests for missing/empty scan, NaN/infinite/nonpositive range, invalid angle increment, insufficient forward coverage and invalid sequence metadata.
- [ ] Define tests for future/stale/nonfinite command and scan timestamps; exact timeout boundary follows shared contracts.
- [ ] Observe missing-feature failures before implementation.
- [ ] Implement boundary validation and distinct diagnostic stop reasons.
- [ ] Run lane and full suites, review, commit this complete slice.
Acceptance: every invalid required input produces zero output; malformed data never silently becomes free space. Do not turn host/device time mismatch into false freshness.

## S2 — Conservative obstacle gate and bounds
- [ ] Test clear synthetic forward scan permits bounded positive motion.
- [ ] Test obstacle exactly at threshold and below threshold stops.
- [ ] Test forward-left/right and front-edge directions; requested steering must not bypass the conservative envelope.
- [ ] Test reverse rejected, positive speed/steering clamped, invalid config rejected.
- [ ] Implement conservative forward half-plane policy with documented finite ray coverage limits.
- [ ] Run all tests and commit a tested slice.
Acceptance: safety result is independent of AI speed and always has explicit reason. A stopped command is stamped consistently and has zero speed; steering stop policy is frozen before implementation.

## S3 — Fault/replay evidence
- [ ] Add deterministic advancing-time tests showing old/frozen capture time expires.
- [ ] Record that a driver inventing fresh timestamps cannot be detected from capture age alone; add driver sequence/health task to the hardware backlog.
- [ ] Provide simple synthetic scenarios for integrator; label all inputs synthetic.

## Deferred physical work
Actual sensor driver validity conventions, drop-off/height hazards, measured braking, steering swept footprint, stopping envelope including sensing/control latency, dynamic obstacles, rear coverage and hysteresis. These must not be represented by a fixed threshold safety claim.

## Suggested next real-sensor tasks
1. Identify LiDAR driver and validate forward/left/right mounting in a viewer.
2. Measure known distances and record raw scan/quality/time fields.
3. Unplug/freeze the sensor and verify stop with independently controlled actuation.
4. Measure braking at approved low speed and derive conservative envelope with margin.
5. Test turns against soft obstacles in a restricted area with a physical cutoff.

## Test names to include
- test_clear_scan_allows_bounded_forward_command
- test_close_or_threshold_obstacle_stops
- test_missing_forward_coverage_stops
- test_invalid_scan_data_stops
- test_future_or_expired_inputs_stop
- test_reverse_is_rejected
- test_turn_does_not_bypass_forward_obstacle

## Handoff evidence template
Task/slice; baseline/head SHA; exact lane/full-suite commands and results; independent review; limitations; next task. No code or commits exist in this documentation folder.
