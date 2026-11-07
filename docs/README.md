# Locate Bot — documentation package

**Scope: Markdown only. Coding is paused. No coding subagents have been launched.**

Plan for a Jetson Orin Nano Super RC-car robot combining LiDAR obstacle constraints with asynchronous camera-based target identification, scoped for a 24-hour hackathon.

## Reading order
1. [Project blueprint](00-PROJECT-BLUEPRINT.md): goals, build sequence, schedule, hardware worksheet.
2. [Architecture](01-ARCHITECTURE.md): data flow, boundaries, alternatives and safety limits.
3. [Shared contracts](02-SHARED-CONTRACTS.md): proposed exact interfaces and conventions.
4. [Branches and merging](03-BRANCH-AND-MERGE-PLAN.md): four isolated lanes and incremental merge policy.
5. [Subscriptions and team](04-TEAM-AND-SUBSCRIPTIONS.md): account allocation and execution boundaries.
6. [Safety lane](lanes/SAFETY.md).
7. [Control lane](lanes/CONTROL.md).
8. [Perception lane](lanes/PERCEPTION.md).
9. [Mission lane](lanes/MISSION.md).
10. [Verification and hardware gates](05-VERIFICATION-AND-HARDWARE-GATES.md).
11. [Agent handoffs](06-AGENT-HANDOFFS.md): prompts for later, not running agents.
12. [Progress board](07-PROGRESS-BOARD.md): unstarted implementation tasks and evidence templates.
13. [Launch guide](08-LAUNCH-GUIDE.md): who runs what, where, plus the master and lane prompts.

## Decisions to review
Proposed: one future `locate-bot` repository with four feature branches, not four divergent repositories. Shared contracts and integration belong to one coordinator; each lane owns its files and tests. All numerical defaults are synthetic-test settings, not physical calibration.

Vision must never block driving. Keep using still-valid navigation/target state between model updates. Stop for unsafe conditions or expired required state, not simply because inference is running.

This folder contains documentation only: no project source, tests or account credentials.

Before coding resumes: review these design choices, fill the hardware inventory, confirm the repository destination, and explicitly approve starting implementation.
