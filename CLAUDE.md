# CLAUDE.md

@AGENTS.md

<!-- omh:agent-instructions:begin -->
## Notes for Claude Code
- `AGENTS.md` (imported above) is the single source of project rules, shared with Codex and other agents. Change rules there, not here.
- Start every session with `docs/PROJECT.md` for current architecture and status.
- Before any change, run `.venv/bin/python -m pytest -q` once to confirm a clean baseline.
- For parallel work, use one git worktree per lane under `.worktrees/` (git-ignored), never two agents in one checkout.
- Hardware-facing code (RPLIDAR, motors, camera, Orin GPU) can't be exercised here. Say what still needs a hardware test in your report instead of claiming it works.
<!-- omh:agent-instructions:end -->
