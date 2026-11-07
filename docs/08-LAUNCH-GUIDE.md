# Launch guide — who runs what, where

Start here once the docs are reviewed. Coding begins only when the integrator runs the master prompt below.

## Branches
| Branch | Purpose | Who commits |
|---|---|---|
| `main` | Reviewed milestones only | Integrator, by merging `integration` |
| `integration` | Shared foundation, candidate merges, cross-lane tests | Integrator only |
| `feat/safety` | Lane A: LiDAR validation and safety supervisor | Safety owner |
| `feat/control` | Lane B: fake vehicle, limits, watchdog | Control owner |
| `feat/perception` | Lane C: asynchronous target perception | Perception owner |
| `feat/mission` | Lane D: mission state and bounded intent | Mission owner |

All six branches start at the same docs commit. The lanes do **not** start coding until the integrator's foundation commit (shared contracts, package layout, test command) lands on `integration` and each lane merges it in.

## Who runs what, where
| Seat | Account | Tool / model | Machine | Checkout | Branch |
|---|---|---|---|---|---|
| Integrator | Claude Max ($200) | Claude Code, Opus | Max owner's own machine | fresh clone | `integration` |
| Lane A — Safety | Claude Pro #1 | Claude Code, Sonnet (Opus for reviews if allowance permits) | Owner's own machine | fresh clone | `feat/safety` |
| Lane B — Control | Claude Pro #2 | Claude Code, Sonnet | Owner's own machine | fresh clone | `feat/control` |
| Lane C — Perception | Claude Pro #3 | Claude Code, Sonnet | Owner's own machine | fresh clone | `feat/perception` |
| Lane D — Mission | ChatGPT Plus | Codex CLI | Owner's own machine | fresh clone | `feat/mission` |
| Reviewer | Claude Max (fresh session) or ChatGPT Plus (Codex `/review`) | Whichever model did **not** write the slice | Integrator's machine | review worktree | read-only |
| Jetson | none yet | — | Jetson Orin Nano Super | — | Not used until hardware gate H1 |

Why this split: the integrator touches every lane and needs the largest allowance, so it gets Max. Lanes are small and well specified, so Pro is enough. Codex on the mission lane, and as a reviewer, gives a second vendor's view on Claude-written code. Pick models from what each tool currently offers; these are suggestions, not pinned versions.

Seats follow the accounts, not the people: whoever holds the Claude Max subscription is the integrator, and Jaideep (repo owner) takes whichever lane matches his own account. Every seat signs in with its **own** account. Never copy credentials between machines or into the repo. Subscriptions don't include API credit; don't enable paid API fallback without agreeing it first.

### If one person runs several lanes on one laptop
Use a separate worktree per lane so agents never share a checkout:
```bash
cd ~/Projects/locatebot
git worktree add ../locatebot-safety feat/safety
git worktree add ../locatebot-control feat/control
git worktree add ../locatebot-perception feat/perception
git worktree add ../locatebot-mission feat/mission
```
Run one agent per worktree in its own terminal. Two different Claude accounts on one machine need separate config directories (for example `CLAUDE_CONFIG_DIR=~/.claude-pro2 claude`); check that this works in your installed version before relying on it.

## Step-by-step
1. **Publish (Jaideep, once):**
   ```bash
   cd ~/Projects/locatebot
   git push -u origin main integration feat/safety feat/control feat/perception feat/mission
   ```
   Add teammates as collaborators on `github.com/Jaideepo7/locatebot`. The integrator needs write access to push `integration` and `main`; consider protecting `main` so only the integrator merges into it.
2. **Teammates clone their lane:**
   ```bash
   git clone https://github.com/Jaideepo7/locatebot.git && cd locatebot
   git switch feat/<lane>
   ```
3. **Integrator (Max owner) clones the repo, runs the master prompt** (below) on `integration`, reviews the foundation and pushes `integration`.
4. **Each lane syncs the foundation, then runs its lane prompt:**
   ```bash
   git fetch origin && git merge origin/integration
   python3 -m unittest discover -s tests -v   # baseline must pass before starting
   ```
5. **Lanes push tested slices** to their own branch and tell the integrator the commit SHA and test output.
6. **Integrator reviews and merges** one slice at a time into `integration`, reruns the full suite after each merge, and promotes to `main` only when the whole suite and the cross-lane checks pass (see `03-BRANCH-AND-MERGE-PLAN.md` and `05-VERIFICATION-AND-HARDWARE-GATES.md`).

## Master prompt — integrator (Claude Code, Claude Max, branch `integration`)
The Max owner runs this on their own machine:
```bash
git clone https://github.com/Jaideepo7/locatebot.git && cd locatebot
git switch integration
claude   # signed in with the Claude Max account
```
Then paste:

```text
You are the integrator for locatebot, a Jetson Orin Nano Super RC car that uses LiDAR for
obstacle safety and an asynchronous vision model to find targets. Read AGENTS.md and every
file in docs/ (start with docs/README.md) before doing anything.

Your job this session: create the foundation commit on the `integration` branch that the
four lanes (feat/safety, feat/control, feat/perception, feat/mission) will build on.

1. Confirm you are on `integration` with a clean tree. Stop if not.
2. Using test-driven development, add only the integrator-owned files:
   - rc_robot/__init__.py and rc_robot/contracts.py: the four immutable dataclasses exactly
     as specified in docs/02-SHARED-CONTRACTS.md (MotionCommand, LidarScan,
     TargetObservation, SafetyDecision). Units and field names verbatim.
   - tests/__init__.py and tests/test_contracts.py: immutability, field names, preserved
     capture timestamps. Write the test first and show it failing before implementing.
   - pyproject.toml (Python >=3.10, no runtime dependencies) and a .gitignore covering
     __pycache__, .venv, .worktrees and private recordings.
   - Empty-module stubs are NOT allowed; lanes create their own modules.
3. Freeze the exact SafetyDecision.reason strings and the stop-command steering policy that
   docs/02-SHARED-CONTRACTS.md leaves to the integrator; record them in that doc.
4. Run `python3 -m unittest discover -s tests -v` and show the real output.
5. Commit with a conventional message (feat(contracts): ...). Do not push, do not touch
   any feat/* branch, do not write lane modules, do not install packages, do not touch
   hardware.
6. Report: commit SHA, test output, anything in the docs you found contradictory, and the
   exact command each lane should run to pick up the foundation.

Rules for the rest of the project: lanes own their files (see docs/03-BRANCH-AND-MERGE-PLAN.md);
you own contracts, root config, tests/test_integration.py and all merges. A lane reporting
DONE is a claim: verify its SHA, rerun the full suite, and get an independent review before
merging. Vision must never block the control loop. Safety is the final gate on motion.
```

## Lane prompt — fill in `<LANE>` (Claude Code or Codex, on `feat/<lane>`)
Lanes: `safety` → `docs/lanes/SAFETY.md`, `control` → `docs/lanes/CONTROL.md`, `perception` → `docs/lanes/PERCEPTION.md`, `mission` → `docs/lanes/MISSION.md`.

```text
You are the <LANE> lane worker for locatebot. Read AGENTS.md, docs/README.md,
docs/01-ARCHITECTURE.md, docs/02-SHARED-CONTRACTS.md and docs/lanes/<LANE>.md first.

Branch: feat/<lane>. It must already contain the integrator's foundation commit
(rc_robot/contracts.py). If that file is missing, stop and say so.

Work through the slices in docs/lanes/<LANE>.md in order, one slice at a time:
1. Write the listed tests first; run them and show they fail for the right reason.
2. Implement the minimum code to make them pass, using only the Python standard library.
3. Run `python3 -m unittest discover -s tests -v` (the whole suite, not just yours).
4. Tick the slice's checkboxes in docs/lanes/<LANE>.md and add the real test result.
5. Commit only your lane's files with a message like feat(<lane>): <behaviour>.
Stop after each slice and report: commit SHA, test output summary, limitations, next slice.

You may edit only: rc_robot/<lane>.py, tests/test_<lane>.py, docs/lanes/<LANE>.md.
Never edit rc_robot/contracts.py or another lane's files; if the contract looks wrong,
describe the change you need and stop. No pushing, merging, package installs, model
downloads, hardware access or nested agents. Never claim physical safety from tests.
```

## Reviewer prompt (fresh session, different model from the author)
```text
Review the locatebot change between <BASE_SHA> and <HEAD_SHA> on <branch>. Read
docs/02-SHARED-CONTRACTS.md and the matching docs/lanes/<LANE>.md, then `git diff <BASE>..<HEAD>`.
Give two separate verdicts: (1) spec compliance, (2) code quality. Focus on fail-closed
defaults, timestamp freshness/future/nonfinite handling, queue bounds and lock scope,
shutdown races, edits outside the lane's allowed files, and any claim the tests don't
support. Cite file:line for every finding. Do not edit files.
```

## Not yet
- Nothing runs on the Jetson until hardware gate H1 in `07-PROGRESS-BOARD.md`.
- No LocateAnything download or quantization during the portable-core phase.
- No pushes from agents; humans push their own branches.
