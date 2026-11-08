# Nav-lite (`locatebot/nav/`)

A small, ROS-free navigation layer for the car: costmap, planner, pure-pursuit follower and
frontier exploration. It turns an `OccupancyGrid` and a pose into normalised
`(throttle, steering)` commands. It only imports shared contracts
(`locatebot.mission.types`, `locatebot.lidar.*`), numpy and the standard library.

Everything here has been tested on synthetic maps and an ideal bicycle model only. Nothing has
run on the real car. The Pi's LiDAR safety stop stays the final gate on motion.

## The car it plans for
- Car-like (Ackermann). It **cannot reverse** (the Pi safety layer blocks reverse) and cannot
  turn on the spot. Minimum turning radius: `VehicleConfig.min_turn_radius_m`
  (= wheelbase / tan(max_steer), 0.45 m with the placeholder values).
- Pose = rear-axle centre in the map frame. +x forward, +y left, angles CCW, metres.
- Throttle mapping (an assumption, also used by `sim.py`):
  `speed = throttle / max_throttle * max_speed_mps`. This only holds if
  `VehicleConfig.max_throttle == SafetyConfig.max_speed`.

## Modules

| Module | What it does |
|---|---|
| `costmap.py` | Classifies the grid. Only FREE cells can be traversable. OCCUPIED cells, and by default UNKNOWN and off-grid cells, are inflated by `half_width_m + margin_m`. A soft cost grows near obstacles. `lethal` marks cells where the car body could touch something. |
| `planner.py` | 8-connected A* (octile heuristic, cost = length × (1 + soft cost)), cost-aware shortcutting, even resampling. Handles the start pose's heading, test-drives each path, and refuses to stop in dead ends. |
| `pursuit.py` | Pure pursuit with a stateful progress index. Reports `following` / `reached` / `needs_replan`. |
| `explore.py` | Frontier detection, clustering and goal choice. A stateful `FrontierExplorer`. `explored_fraction`. |
| `sim.py` | Kinematic bicycle model, plus throttle↔speed helpers, for closed-loop tests. |

## API

```python
from locatebot.nav.costmap import CostmapConfig, Costmap, build_costmap, costmap_from_classes
cm = build_costmap(grid: OccupancyGrid, vehicle: VehicleConfig | None = None,
                   config: CostmapConfig | None = None) -> Costmap
# Costmap fields: classes, traversable, lethal, clearance_m, occ_clearance_m, cost (inf = blocked),
#   resolution_m, origin_xy, vehicle, config; world_to_cell(xy) -> (col, row), cell_to_world,
#   in_bounds, is_traversable_xy(xy), clearance_at(xy)

from locatebot.nav.planner import (plan, plan_approach, PlannedPath, path_clear, check_tracking,
                                   can_turn_around, has_forward_exit, astar, reachable_steps)
plan(cm, start_xy, goal_xy, *, start_theta=None, allow_escape=True, escape_radius_m=0.5,
     smooth=True, spacing_m=0.1, ensure_exit=True, max_trim_m=1.0, validate=True,
     max_repairs=3) -> PlannedPath | None
plan_approach(cm, start_xy, item_xy, standoff_m, *, start_theta=None, ..., max_trim_m=0.5,
              max_candidates=10) -> PlannedPath | None
# PlannedPath is a list[(x, y)] with .goal_xy, .trimmed_m and .detail
path_clear(cm, pts, near_m=0.3) -> bool          # re-check the path ahead against a newer map
check_tracking(cm, path, start_pose) -> (ok, fail_xy | None, detail)
can_turn_around(cm, xy, theta, turn_deg=150, max_expansions=800, strict=True) -> bool

from locatebot.nav.pursuit import PurePursuit, PursuitConfig, PursuitCommand, PursuitStatus
pp = PurePursuit(vehicle, PursuitConfig()); pp.set_path(path)
cmd = pp.update(pose, speed_mps=None, clearance_m=None) -> PursuitCommand
# cmd.throttle in [0, max_throttle], cmd.steering in [-1, 1] (+ = left, 1 = max_steer_rad),
# cmd.status, cmd.detail, cmd.target_xy, cmd.cross_track_m, cmd.remaining_m, cmd.lookahead_m
pp.remaining_path(); pp.progress_m

from locatebot.nav.explore import (FrontierExplorer, ExploreConfig, Frontier, find_frontiers,
                                   frontier_mask, cluster_frontiers, explored_fraction)
ex = FrontierExplorer(ExploreConfig(), retry_distance_m=1.0, max_failures=2)
fr = ex.next_goal(cm, pose) -> Frontier | None      # None = exploration finished
ex.mark_visited(goal_xy); ex.mark_failed(goal_xy, from_xy=(x, y)); ex.goal_still_useful(cm, fr)
explored_fraction(grid_or_classes) -> float

from locatebot.nav.sim import BicycleSim
sim = BicycleSim(vehicle, pose, dt=0.05, speed_tau_s=0.0, steer_rate_rad_s=None)
sim.step(throttle, steering) -> Pose2D
```

## How the planner copes with a forward-only car
1. **Grid route.** A* on the costmap gives a route that ignores heading.
2. **Start heading** (`start_theta` given). If the route's first ~1.35 m is not
   `start_feasible`, a small Hybrid-A*-style search runs over forward constant-curvature motions.
   "Not feasible" means a point behind the car, or a point beyond one turning radius that lies
   inside a full-lock circle. The search uses 0.3 m primitives at 1.05 × the minimum radius,
   guided by an exact lazy-Dijkstra cost-to-go. It finds a manoeuvre after which the grid route is
   feasible, such as a loop round furniture. If no forward manoeuvre exists, the result is `None`.
3. **Tracking check** (`validate`). The path is test-driven with pure pursuit on the bicycle
   model. If the simulated car fails, for example at a 90° bend at the end of a narrow slot, a
   0.35 m disc there is blocked and the route is replanned (up to 3 times).
4. **Exit check** (`ensure_exit`). The car must never stop where it cannot turn around. The end
   is pulled back in 0.15 m steps, up to `max_trim_m`, until `can_turn_around` passes with the
   arrival heading ±0.3 rad. That check needs a ≥150° forward turn on traversable cells. If no
   end point passes, the result is `None`. So a goal 1.5 m into a dead-end slot is refused.
   `plan_approach` keeps the trimmed end within `standoff_m` by trying several candidate cells.
5. **Start escape.** If the car starts inside the inflation band (parked near a wall), the path
   first heads to the nearest traversable cell, and the lattice may use body-clear cells for its
   first 0.5 m.

## What an executor (skill runner) must do
`run_exploration` in `tests/nav/test_explore.py` is the reference loop.
- Build a costmap and plan with `start_theta=pose.theta`. Call `pp.update` at control rate.
- At about 2 Hz, rebuild the costmap and call `path_clear(cm, pp.remaining_path())`. If it
  fails, stop and replan. Thin walls are sometimes mapped FREE until the car is close: grazing
  LiDAR rays erode them (see Limitations).
- On `needs_replan`, replan from the current pose. Give up on a goal after a few attempts.
  Call `mark_failed(goal, from_xy=pose)`. That goal is then retried only from somewhere else, and
  dropped after 2 failures.
- Abandon a path early (for example, when the frontier has already been seen) only if
  `can_turn_around` holds at the current pose.
- If `plan` returns `None` and `can_turn_around` is False, the car is stuck without reversing.
  Report it to the agent or operator.

## Exploration and `explored_fraction`
- Frontier = a FREE cell with an UNKNOWN 8-neighbour and no OCCUPIED cell within 2 cells. The
  rule drops false frontiers in front of small holes in walls (cells missed at grazing angles).
- `explored_fraction = known / (known + U)`. Known = FREE + OCCUPIED. U = UNKNOWN cells
  4-connected (through UNKNOWN) to a frontier cell. Space sealed off by walls doesn't count. A
  closed, fully mapped room gives 1.0. A map with no FREE cell gives 0.0. Early on, unknown space
  outside open walls counts, so the value starts low and jumps as the room closes.

## Test results (synthetic, `tests/nav/`, ~11 s)
- Unit tests cover: inflation radius, exact distance transform, soft cost, A* optimality, no path
  through unknown, approach to a standoff, start escape, heading-aware start, dead-end refusal,
  tracking check, path re-check, pursuit steering sign and limits, reached / behind / off-path
  status, bicycle turning radius, and the frontier and blacklist logic.
- Closed loop: plan, then pure pursuit, then the bicycle model, in a 6×4 m room with a box. Runs
  from three starts (one facing away from the goal) and one item approach. The car never enters a
  lethal cell and keeps ≥ `half_width_m` from every wall.
- Exploration from scratch, in `demo_room()` and in a two-room layout with a 1.5 m door. The map
  is built from `simulate_scan` with ground-truth poses: 508 beams, 27 % dropout, matching the
  C1 below. In 4 test cases the explored fraction goes from < 0.5 to > 0.95, the run terminates,
  and the car is never closer than `half_width_m` to an obstacle.
- An offline sweep of 16 start poses gave 15 runs at ≥ 0.97 explored, no collisions and none
  stuck. One run ended at 0.59: its last frontier could only be reached through a dead-end slot,
  and that frontier connects to unknown space outside the room. See Limitations.

## Limitations
- **Synthetic only.** The bicycle model is ideal (no slip, no servo lag unless `speed_tau_s` /
  `steer_rate_rad_s` are set, perfect pose). Real tracking error is larger.
- **No reverse.** Some places stay unreachable on purpose: dead-end slots, and the gap behind
  furniture next to a wall. The car needs about 1.35 m of traversable width to turn around with
  margin, or 1.2 m body-clear. A future reverse mode would change `can_turn_around`, the lattice
  and the tracking check.
- **Thin walls can vanish from the map.** Grazing rays free wall cells, and a wall lying exactly
  on a cell boundary is the worst case. One test run planned through such a wall until the car
  got close. The 2 Hz `path_clear` re-check catches this. The LiDAR lane should consider
  limiting free-space updates at grazing incidence.
- `explored_fraction` can stay low when one unresolved frontier touches unknown space leaking out
  through a wall gap. A stall in the fraction is a better "done" signal than a threshold.
- The planner is pure Python. Planning takes ~10–50 ms on x86 when the grid route is feasible.
  Heading manoeuvres, tracking checks and exit checks add ~0.05–0.5 s. It hasn't been profiled on
  the Orin.
- 2D only. Obstacles above or below the scan plane are invisible here (see the LiDAR README).

## Tune or measure on the real car
- `VehicleConfig`: measure the wheelbase, real max steering angle (left and right),
  half-width + margin, and speed at `max_throttle`. Check that `max_throttle` equals
  `SafetyConfig.max_speed`.
- Steering sign and linearity: does `steering = +1` turn left at `max_steer_rad`? Is the servo
  linear? Is there a deadband?
- `PursuitConfig`: lookahead (0.3–0.8 m, 1 s × speed), `max_cross_track_m` (0.25),
  `goal_tolerance_m` (0.15), `min_throttle_frac` (0.35: the car must not stall at that throttle).
- `CostmapConfig`: `margin_m` (0.05), `soft_range_m` / `soft_weight`, and classify thresholds
  against real map noise.
- `ExploreConfig`: `min_frontier_cells`, `reach_radius_m`, `min_goal_dist_m`.
- LiDAR: the RPLIDAR C1 gives ~508 points per revolution (~0.7° spacing) at 10 Hz. About 370 are
  valid in a cluttered room. Range noise is ~0.6 mm. The sim uses 1 cm noise and 27 % dropout.
  Check that thin chair legs and glass are mapped. Re-check `frontier_mask`'s 2-cell hole filter
  against real wall holes.
- Control-loop rate and latency (pose → command → motor), and the replan rate (2 Hz assumed).
- Runtime of `plan` on the Orin, next to the vision worker.
