# Agent task briefs: R (remote control, retired) and M (smart map)

Each `*.md` here is a complete prompt for one agent. Give the agent the file's contents (or tell it: "Read `docs/tasks/<ID>.md` in /home/jaideep/Projects/locatebot and do it"). Background: `docs/ROADMAP.md` (sections R and M), `docs/PROJECT.md`, `AGENTS.md`.

**R tasks retired (2026-11-08).** The team chose the Node **Spectrum** bot in `my-spectrum-app/` as the only Telegram path; it calls the Orin operator HTTP API directly. The Python Photon/Telegram service on the Pi (`locatebot/remote/`, built by R1–R3) was a duplicate and has been removed, together with the R1, R2, R3 and R5 briefs (they are in git history). What stays: R4 (goal input for the LLM, done, `R4.md`) and the Orin half of R3, the mission bridge (`locatebot/orin/missions.py` + the mission messages in `locatebot/link/messages.py`), kept for a future Pi-side mission sender. Do not hand out R1–R3 or R5 again.

## Waves (run each wave's tasks in parallel; a wave needs the previous one merged)
| Wave | Tasks | Needs first |
|---|---|---|
| **0** | **Step C contracts** (integrator, not an agent): `MissionRequest/MissionCancel/MissionStatus/Range` + `OrinStatus.boot_id` in `locatebot/link/messages.py`; `WorldState.goal`, `WorldState.people`, `PersonView` in `locatebot/mission/types.py` | - |
| **1** | M1 (localize in a saved map), R4 (goal input for the LLM; needs wave 0; done) | M1: nothing. R4: wave 0 |
| **2** | M2 (static/dynamic layers, change detection, `.lbmap` v2) | M1 |
| **3** | M3 (people tracking + ultrasonic/glass + Wi-Fi layers) | M2, wave 0 |

## Rules every agent follows (repeated in each brief)
- One git worktree per task under `.worktrees/`, one branch, never edit the main checkout or another worktree.
- Integrator-owned files are read-only for you: `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `locatebot/mission/types.py`, `locatebot/link/messages.py`, `pyproject.toml`, `docs/PROJECT.md`, `AGENTS.md`. Need a change? Describe it in your report.
- Test first; run the **whole** suite from the worktree root with `/home/jaideep/Projects/locatebot/.venv/bin/python -m pytest -q`.
- No hardware, no network in tests (127.0.0.1 servers are fine). Never put keys, tokens, real IPs or personal data in the repo.
- Commit on your branch with conventional messages ending `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Do not push, merge or rebase other branches.
- Report: commit SHAs, the real final pytest line, public API, deviations, limitations, what needs hardware.
