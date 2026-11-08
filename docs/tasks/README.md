# Agent task briefs: R (remote control via Photon/Telegram) and M (smart map)

Each `*.md` here is a complete prompt for one agent. Give the agent the file's contents (or tell it: "Read `docs/tasks/<ID>.md` in /home/jaideep/Projects/locatebot and do it"). Background: `docs/ROADMAP.md` (sections R and M), `docs/PROJECT.md`, `AGENTS.md`.

## Waves (run each wave's tasks in parallel; a wave needs the previous one merged)
| Wave | Tasks | Needs first |
|---|---|---|
| **0** | **Step C contracts** (integrator, not an agent): `MissionRequest/MissionCancel/MissionStatus/Range` + `OrinStatus.boot_id` in `locatebot/link/messages.py`; `WorldState.goal`, `WorldState.people`, `PersonView` in `locatebot/mission/types.py`; `remote` extra in `pyproject.toml` | - |
| **1** | R1 (port Photon messaging core to the Pi), M1 (localize in a saved map), R4 (goal input for the LLM; needs wave 0) | R1, M1: nothing. R4: wave 0 |
| **2** | R2 (Photon WebSocket inbound + Pi service), M2 (static/dynamic layers, change detection, `.lbmap` v2) | R2: R1. M2: M1 |
| **3** | R3 (mission bridge Pi <-> Orin over ZeroMQ), M3 (people tracking + ultrasonic/glass + Wi-Fi layers) | R3: R1, R2, R4, wave 0. M3: M2, wave 0 |
| **4** | R5 (live Photon + Telegram test, people) | R2, R3, internet decision D5 |

## Rules every agent follows (repeated in each brief)
- One git worktree per task under `.worktrees/`, one branch, never edit the main checkout or another worktree.
- Integrator-owned files are read-only for you: `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `locatebot/mission/types.py`, `locatebot/link/messages.py`, `pyproject.toml`, `docs/PROJECT.md`, `AGENTS.md`. Need a change? Describe it in your report.
- Test first; run the **whole** suite from the worktree root with `/home/jaideep/Projects/locatebot/.venv/bin/python -m pytest -q`.
- No hardware, no network in tests (127.0.0.1 servers are fine). Never put keys, tokens, real IPs or personal data in the repo.
- Commit on your branch with conventional messages ending `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Do not push, merge or rebase other branches.
- Report: commit SHAs, the real final pytest line, public API, deviations, limitations, what needs hardware.
