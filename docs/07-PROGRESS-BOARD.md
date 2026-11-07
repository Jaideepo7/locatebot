# Progress board — documentation-only baseline

## Current state
Documentation prepared. Architecture decisions await review. Implementation is paused by user instruction. No coding agents running, no subscriptions authenticated here, no hardware evidence and no remote repository configured by this documentation task.

Branches `integration`, `feat/safety`, `feat/control`, `feat/perception` and `feat/mission` exist at the docs commit. Start with the master prompt in `08-LAUNCH-GUIDE.md`.

## Planned tasks
| ID | Owner lane | Deliverable | Depends on | State |
|---|---|---|---|---|
| D0 | User/integrator | Approve architecture and scope | Documentation | Awaiting review |
| D1 | Team | Hardware/software inventory | None | Backlog |
| F0 | Integrator | Approved contracts, package/test foundation | D0 + coding approval | Not started in this folder |
| S1 | Safety | Invalid/stale input rejection | F0 | Backlog |
| S2 | Safety | Conservative obstacle gate/bounds | S1 | Backlog |
| C1 | Control | Fake vehicle boundary | F0 | Backlog |
| C2 | Control | Command limits/software watchdog | C1 | Backlog |
| P1 | Perception | Latest observation store | F0 | Backlog |
| P2 | Perception | Bounded async inference adapter | P1 | Backlog |
| M1 | Mission | State/invalid-target handling | F0 | Backlog |
| M2 | Mission | Fresh-target bounded intent | M1 | Backlog |
| I1 | Integrator/reviewer | Review lane slices/integration tests | S2+C2+P2+M2 | Backlog |
| H1 | Team | Actual sensor replay/health | D1 + approved hardware work | Backlog |
| H2 | Control/safety | MCU watchdog/calibration/cutoff | H1 + physical gates | Backlog |
| N1 | Mission | Odometry/known-goal navigation | H2 + localization | Backlog |
| V1 | Perception | Actual model benchmark/localization | D1 + explicit approval | Backlog |

## Status rules
BACKLOG → IN_PROGRESS → TESTED → REVIEWED → INTEGRATED. BLOCKED always names a reason/owner. A percentage is optional planning context, not a merge gate. A complete status needs exact evidence. No automatic future progress is implied by this board.

## Evidence ledger template
| Task | Agent/person | Base/head SHA | Tests and actual result | Review | Integration SHA | Remaining limits |
|---|---|---|---|---|---|---|
| None executed here | — | — | — | — | — | Documentation only |

## Open decisions
- Actual remote/repository owner and team member access.
- LiDAR, camera, MCU, ESC and installed Jetson software versions.
- Tag-based first target versus semantic object target.
- Measured safe operating envelope and physical emergency cutoff.
- Current supported coding-tool authentication/model entitlements per account.
- User approval before any implementation resumes.
