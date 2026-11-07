# Lane D — mission state and bounded target intent

**Status: planned only. Branch: feat/mission. Owner: unassigned.**
Allowed future files: rc_robot/mission.py, tests/test_mission.py, docs/lanes/mission.md. Consume shared observations and produce MotionCommand. Do not command hardware or override safety.

## Deliverable
Pure MissionPlanner.step(target, now, enabled=True) with explicit state and bounded command intent. No inference calls, sleeps, frame waits or hidden I/O inside step.

## M1 — State and invalid-target behaviour
- [ ] Test disabled yields IDLE and zero command.
- [ ] Test absent/invalid/low-confidence target yields SEARCHING and zero command.
- [ ] Test future/expired target cannot generate motion.
- [ ] Test nonfinite time/config validation and exact freshness/confidence boundaries.
- [ ] Observe red; implement deterministic state decision, run all tests, review and commit.
SEARCHING is stationary in the first portable core. Autonomous exploration is a separate future navigation capability, not a spinning-in-place action for a car.

## M2 — Fresh-target intent
- [ ] Test valid target yields TRACKING and bounded positive intent.
- [ ] Test left/right bearing signs and steering limits.
- [ ] Reuse one fresh observation across repeated control ticks; confirm commands continue without a new AI response.
- [ ] Advance time past expiry and verify zero; distinguish safe expiry stop from waiting on every model cycle.
- [ ] Observe red; implement minimal bounded intent, run lane/full suites, review and commit.
Acceptance: pure repeatable function of supplied state/time; output timestamp is current command time; target capture time remains unchanged. Bearing tracking is not route planning or proof the target can be reached.

## M3 — Real mission manager, later
Expand states only after navigation exists: IDLE → SEARCHING → TARGET_CONFIRMED → PLANNING → NAVIGATING → FINAL_APPROACH → ARRIVED. Any moving state may transition to BLOCKED, LOST_LOCALIZATION, SENSOR_FAULT or EMERGENCY_STOP. Define entry conditions, permitted motion, timeouts and recovery before implementing each transition.

## M4 — Known destinations before semantic search
First navigate to a map goal, then AprilTag station ID, then a named object with validated depth/localization. Generate a reachable stand-off pose rather than driving into the marker/object. Handle multiple matches, target loss, unreachable goal and blocked path explicitly. Car steering needs turning radius; do not assume differential-drive rotate-in-place behaviour.

## Test names
- test_disabled_is_idle_and_zero
- test_absent_or_invalid_target_is_searching
- test_valid_target_tracks_with_bounded_steering
- test_repeated_ticks_use_still_fresh_target
- test_required_target_expiry_stops
- test_future_timestamp_never_creates_motion

## Handoff
Commit/test/review evidence, state transition table, exact deferred navigation/depth/arrival capabilities, and next known-goal task. No code is authorized by this document alone.
