# Lane B — fake vehicle, command limits and watchdog

**Status: planned only. Branch: feat/control. Owner: unassigned.**
Allowed future files: rc_robot/control.py, tests/test_control.py, docs/lanes/control.md. Consume shared MotionCommand, never bypass SafetyDecision orchestration. No real ESC/servo adapter in the first batch.

## Deliverable
FakeVehicle.apply/history plus WatchdogController.submit/tick, as specified in shared contracts. Fake output must make every applied command observable. It is a test harness, not a dynamics simulator.

## C1 — Fake output boundary
- [ ] Write tests recording actual submitted commands in order and rejecting nonfinite fields.
- [ ] Observe red, implement minimum fake output, run lane/full suites.
- [ ] Confirm construction produces no movement and imports open no hardware devices.
- [ ] Commit complete fake-boundary slice after review.

## C2 — Software watchdog and independent bounds
- [ ] Test startup tick returns/applies zero.
- [ ] Test fresh command applies within limits.
- [ ] Test expired original command timestamp stops even when submitted recently.
- [ ] Test future/nonfinite time, reverse, invalid command and out-of-limit inputs produce zero, not retained old motion.
- [ ] Test exact timeout boundary, repeated ticks and resumed fresh commands.
- [ ] Test invalid timeout/speed/steering configuration rejected.
- [ ] Observe expected failures; implement explicit state and bounded output.
- [ ] Run full suite, review, commit only owned files.
Acceptance: stale intent cannot survive by repeatedly resubmitting it. No silent clamping of commands outside the downstream controller's independent limits; the supervisor is responsible for producing bounded commands first.

## C3 — Physical fail-safe design, documentation only initially
Document MCU serial/protocol requirements: sequence ID, command age/heartbeat, bounded fields, neutral/brake behaviour, timeout and restart state. Keep the protocol design distinct from actually flashing hardware. On host crash/USB disconnect the MCU must stop output without needing another Python tick.

## Critical limitations
A software watchdog called by the same host cannot stop a frozen or dead host. Physical safety requires independently enforced timeout plus accessible cutoff. ESC neutral is not universally zero PWM and braking/reverse semantics differ. Do not guess pulse widths or electrical connections.

## Later hardware acceptance
- Exact ESC/servo models and signal/electrical levels verified.
- Neutral/brake/reverse calibrated with wheels raised.
- Restart and communications loss never energize propulsion.
- Manual dead-man and physical cutoff exercised.
- Actual stopping distances measured in restricted low-speed area.
- Power/grounding/noise checks complete before connecting controller outputs.

## Test names
- test_startup_is_zero
- test_fake_history_records_applied_commands
- test_original_command_age_drives_timeout
- test_invalid_submission_clears_previous_motion
- test_reverse_and_out_of_limit_are_rejected
- test_future_or_nonfinite_time_stops

## Handoff
Provide exact commits, real lane/full-suite results, reviewer findings, declared absence of hardware output and next MCU/calibration task. No coding is authorized by this file alone.
