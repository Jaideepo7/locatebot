# Future agent handoff templates — do not launch yet

Current instruction: Markdown only. These are copyable task briefs for when the user explicitly resumes coding, not evidence of dispatched agents.

## Common worker brief
You own one bounded slice in an isolated worktree of the future locate-bot repository. Read architecture, shared contracts and your lane document. Record baseline SHA. Modify only permitted lane files. Use standard-library Python and unittest for the portable core. Write behaviour tests first and observe expected red, implement minimum behaviour, run lane and full suites, review your diff and commit scoped files. Do not install/download models, access credentials, actuate hardware, spawn nested agents, change contracts, merge, push or publish. If a shared API is missing or contradictory, report a proposed contract change to the coordinator instead of creating a local incompatible version. Report actual results and explicit limits; never fabricate outputs.

## Worker A — safety
Branch feat/safety. Read lanes/SAFETY.md and 02-SHARED-CONTRACTS.md. Implement one reviewed slice at a time: unhealthy-input rejection, conservative forward obstacle gate, then replay/fault evidence. Allowed future files: rc_robot/safety.py, tests/test_safety.py, docs/lanes/safety.md. Output no actuator code or physical collision-safety claims.

## Worker B — control
Branch feat/control. Read lanes/CONTROL.md and shared contracts. Build fake output then software watchdog/limits. Allowed future files: rc_robot/control.py, tests/test_control.py, docs/lanes/control.md. Explicitly document that independent MCU watchdog/calibration remain unimplemented. No GPIO/PWM/serial writes in the portable core.

## Worker C — perception
Branch feat/perception. Read lanes/PERCEPTION.md and shared contracts. Implement latest observation store then bounded asynchronous worker with Event-based tests. Allowed future files: rc_robot/perception.py, tests/test_perception.py, docs/lanes/perception.md. Preserve capture time; no inference lock on control; no actual model integration or performance claims in the first slice.

## Worker D — mission
Branch feat/mission. Read lanes/MISSION.md and shared contracts. Implement deterministic state/invalid-target behaviour then fresh-target bounded intent. Allowed future files: rc_robot/mission.py, tests/test_mission.py, docs/lanes/mission.md. No blocking calls, obstacle bypass, hardware output or fabricated metric target goal.

## Worker completion report
- Status: TESTED / BLOCKED / NEEDS_REVIEW, never self-approved INTEGRATED.
- Task ID, worktree, baseline and head SHA.
- Files changed and behaviours implemented.
- Observed red-test reason; exact lane/full-suite commands and outputs.
- Contract/dependency impact.
- Limitations and next concrete task.
- Reviewer-ready scoped diff reference.

## Independent reviewer brief
Read exact task brief, common contracts, baseline-to-head diff and test report. Review both spec compliance and code quality. Focus on fail-closed defaults, timestamp semantics, queue bounds, lock scope, shutdown races, out-of-scope edits and misleading safety/model claims. Identify reproducible findings with file/line and expected behaviour. Do not accept author self-review as independent approval; do not approve missing evidence. Report compliance and quality verdicts separately.

## Integrator brief
Keep shared contracts/root configuration stable. Review each complete candidate slice, verify exact commits and scope, merge into integration one at a time, rerun suite after each merge. Add cross-lane acceptance scenarios from verification document; stop promotion on failures. Maintain progress board with observed evidence, not guessed percentages. Promote only a reviewed passing milestone. Do not push without explicit authorization.

## Stopping conditions
Missing authority, security-sensitive operation, destructive/irreversible operation, hardware actuation, paid API fallback, contract contradiction blocking safe progress, or requested user pause. Preserve report/task state and ask the coordinator; never work around the boundary silently.
