# Branch, ownership and incremental merge plan

## Recommended future repository
One `locate-bot` repository, four development branches. Separate teammate clones or Git worktrees isolate files/index state; never let multiple agents edit one checkout. This document creates none of them.

| Branch | Owner | Exclusive files |
|---|---|---|
| main | Release integrator | Reviewed milestones only |
| integration | Integrator | Candidate merges, shared contracts, root config and integration tests |
| feat/safety | Agent/person A | rc_robot/safety.py; tests/test_safety.py; docs/lanes/safety.md |
| feat/control | Agent/person B | rc_robot/control.py; tests/test_control.py; docs/lanes/control.md |
| feat/perception | Agent/person C | rc_robot/perception.py; tests/test_perception.py; docs/lanes/perception.md |
| feat/mission | Agent/person D | rc_robot/mission.py; tests/test_mission.py; docs/lanes/mission.md |

## Setup after explicit coding approval
1. Confirm remote owner, visibility, access and account authentication; never guess a publishing destination.
2. Integrator prepares foundation contracts, package, test command, instructions and ignored worktree directory.
3. Review foundation, record baseline SHA, then branch every lane from that same SHA.
4. Verify clean baseline in every checkout.
5. Assign one lane brief, one permitted file list, and one tested slice per agent run.
6. Keep root CI/dependency/shared-interface edits integrator-owned.

## Checkpoint policy
For every slice: write behaviour test, observe expected red, implement minimum behaviour, run lane and full suite, review scope, commit owned files. Suggested future subjects:
- feat(safety): reject stale scans and unsafe forward commands
- feat(control): stop fake output on command timeout
- feat(perception): add latest-only async inference handoff
- feat(mission): produce bounded intent from fresh targets
Suggested subjects are not observed commits.

## Merge-ready gate
Require base/head SHAs, clean worktree, scoped diff, actual named test results, full-suite pass, independent spec/quality review, known limitations and remaining tasks. Worker DONE is only a report. Integrator checks the exact diff and reruns tests after merging into integration. Failed cross-lane tests block promotion even if lane suites passed individually.

## Partial completion
Merge complete, independently testable behaviour slices, never “50% of a branch.” A valid stale-scan rejection slice may land while swept-footprint work stays pending. Split mixed complete/incomplete commits and retest before review. Do not cherry-pick a slice that secretly depends on missing unmerged code. Leave unfinished work unmerged rather than hiding gaps behind placeholders.

## Integration order and dependencies
Foundation first. The four lanes depend only on contracts, not each other's unmerged internals. Preferred first integration order: safety, control, perception, mission. Integrator then wires all four and adds blocked-inference/sensor-fault acceptance tests. Promote to main only after final independent review and full suite/demo evidence. Preserve active branches and worktrees for teammates.

## Conflict and rollback
Ownership collisions return to the coordinator for splitting/reassignment. Resolve shared contract changes centrally, not blindly with ours/theirs. Reproduce regression with a test and fix/revert the offending slice without force-pushing shared history. Main stays on the last passing milestone. Remote pushes and releases require explicit user authority.

## PR/handoff template
- Task/lane and exact base/head SHAs.
- Implemented behaviours; explicitly deferred behaviours.
- Test commands and real output summaries.
- Contract/dependency changes and consumers.
- Safety impact, limitations and hardware evidence boundary.
- Reviewer findings/resolutions.
- Integrator verification and merge SHA, filled after observation only.
