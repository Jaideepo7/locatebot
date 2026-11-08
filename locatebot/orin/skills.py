"""Skill executor (Orin): turns one ``SkillCall`` into drive commands with Nav-lite.

The LLM (or a fallback routine, or the operator) picks a skill; ``SkillExecutor`` runs it one
control tick at a time and reports a ``SkillStatus``. It only produces *requested* throttle and
steering: the Pi's LiDAR safety supervisor still has the final say on every command.

Skills (see ``locatebot.mission.types.SkillName``):
- ``stop``: zero output, succeeds at once.
- ``go_to``: to a map point (``x``, ``y``) or to the best track of a named item, stopping
  ``standoff_m`` short of it (items sit on or against obstacles).
- ``return_home``: ``go_to`` the session's start pose.
- ``explore``: frontier exploration until no reachable frontier is left.
- ``search``: explore while vision looks for ``items`` (exposed as ``search_items``); succeeds
  when every item has a confirmed track, fails listing what is missing if exploring ends first.
- ``look_around``: one slow full-lock loop so the camera sweeps the surroundings (the car can't
  turn on the spot); fails if the loop doesn't fit in known free space.
- ``follow``: keep about ``follow_distance_m`` behind a moving item. Vision fixes arrive every
  few seconds; between them the target is tracked as the LiDAR cluster nearest its last
  position. Fails once nothing has confirmed the target for ``follow_lost_s``.

``tick`` never raises: any internal error stops the car and fails the skill.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

from locatebot.lidar.detect import clean_scan, cluster_scan, corridor_clearance
from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.types import Pose2D, Scan, wrap_angle
from locatebot.localize.items import ItemTracker
from locatebot.mission.types import SkillCall, SkillName, SkillState, SkillStatus, VehicleConfig
from locatebot.nav.costmap import Costmap, CostmapConfig, build_costmap
from locatebot.nav.explore import ExploreConfig, FrontierExplorer, explored_fraction
from locatebot.nav.planner import path_clear, plan, plan_approach, reachable_mask, start_cell
from locatebot.nav.pursuit import PursuitConfig, PursuitStatus, PurePursuit

log = logging.getLogger(__name__)

_TERMINAL = (SkillState.SUCCEEDED, SkillState.FAILED, SkillState.CANCELLED)


@dataclass(frozen=True)
class ExecutorConfig:
    standoff_m: float = 0.6            # go_to(item): stop this far from the item
    point_standoff_m: float = 0.3      # go_to(x, y) / return_home when the point itself is blocked
    replan_interval_s: float = 1.0     # re-check the path against the growing map this often
    max_replans: int = 6               # consecutive failed plans / NEEDS_REPLAN before giving up
    no_map_timeout_s: float = 5.0      # fail if no map arrives within this
    follow_distance_m: float = 1.0
    follow_lost_s: float = 6.0         # fail when the target hasn't been confirmed for this long
    follow_gate_m: float = 0.5         # LiDAR cluster must be this close to the predicted target
    follow_person_gate_m: float = 1.0  # a person track must be this close to the last target
    look_around_throttle_frac: float = 0.5   # of VehicleConfig.max_throttle
    look_around_timeout_s: float = 60.0
    explore_plan_attempts: int = 3     # frontier goals tried per tick before waiting
    patrol_spacing_m: float = 1.5      # search: waypoints spread over the reachable floor
    max_patrol_points: int = 12
    sweep_throttle_frac: float = 0.3   # search: slow loop so consecutive vision frames overlap
    search_timeout_s: float = 600.0    # search gives up (listing found/missing) after this
    min_explored_frac: float = 0.0     # explore succeeds only above this (informational)


@dataclass(frozen=True)
class TickInput:
    now: float
    pose: Pose2D
    grid: OccupancyGrid | None
    scan: Scan | None
    tracker: ItemTracker | None
    home: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    people: tuple = ()   # lidar.people.PersonTrack (M3): follow(person) prefers these to raw clusters


@dataclass(frozen=True)
class DriveOutput:
    throttle: float = 0.0
    steering: float = 0.0


_ZERO = DriveOutput()


class _SkillFailed(Exception):
    pass


class SkillExecutor:
    def __init__(self, vehicle: VehicleConfig | None = None, config: ExecutorConfig | None = None,
                 costmap_config: CostmapConfig | None = None, pursuit_config: PursuitConfig | None = None,
                 explore_config: ExploreConfig | None = None) -> None:
        self.vehicle = vehicle or VehicleConfig()
        self.config = config or ExecutorConfig()
        self.costmap_config = costmap_config
        self.pursuit_config = pursuit_config
        self.explore_config = explore_config
        self._status = SkillStatus()
        self.explored_frac: float | None = None
        self._reset_motion()

    # ------------------------------------------------------------------ public API
    @property
    def status(self) -> SkillStatus:
        return self._status

    @property
    def search_items(self) -> tuple[str, ...]:
        """Items vision should look for right now (empty unless a search/follow/go_to is running)."""
        call = self._status.call
        if call is None or self._status.state != SkillState.RUNNING:
            return ()
        if call.name == SkillName.SEARCH:
            return tuple(call.args.get("items", ()))
        if call.name in (SkillName.FOLLOW, SkillName.GO_TO) and isinstance(call.args.get("item"), str):
            return (call.args["item"],)
        return ()

    def start(self, call: SkillCall, now: float) -> SkillStatus:
        """Start ``call``, cancelling whatever was running."""
        self._reset_motion()
        self._started_s = now
        self._status = SkillStatus(SkillState.RUNNING, call, "starting")
        return self._status

    def cancel(self, detail: str = "cancelled") -> None:
        if self._status.state == SkillState.RUNNING:
            self._status = SkillStatus(SkillState.CANCELLED, self._status.call, detail)
        self._reset_motion()

    def tick(self, inp: TickInput) -> DriveOutput:
        """One control step. Never raises; returns zero output unless a skill is driving."""
        if self._status.state != SkillState.RUNNING or self._status.call is None:
            return _ZERO
        try:
            out = self._dispatch(self._status.call, inp)
        except _SkillFailed as e:
            self._finish(SkillState.FAILED, str(e))
            return _ZERO
        except Exception as e:  # never let a bug leave the car driving
            log.exception("skill %s crashed", self._status.call.name.value)
            self._finish(SkillState.FAILED, f"internal error: {e}")
            return _ZERO
        if self._status.state in _TERMINAL:
            return _ZERO
        return DriveOutput(float(np.clip(out.throttle, 0.0, self.vehicle.max_throttle)),
                           float(np.clip(out.steering, -1.0, 1.0)))

    # ------------------------------------------------------------------ plumbing
    def _reset_motion(self) -> None:
        self._pursuit = PurePursuit(self.vehicle, self.pursuit_config)
        self._path = None
        self._goal: tuple[float, float] | None = None
        self._goal_label = ""
        self._goal_standoff: float | None = None
        self._last_check_s = -math.inf
        self._fails = 0
        self._started_s = 0.0
        self._explorer = FrontierExplorer(self.explore_config)
        self._frontier = None
        self._turned = 0.0
        self._last_theta: float | None = None
        self._target: tuple[float, float] | None = None
        self._target_seen_s = -math.inf
        self._target_fix_s = -math.inf
        self._costmap: Costmap | None = None
        self._costmap_s = -math.inf
        self._phase = "explore"           # search: explore -> patrol
        self._patrol: list[tuple[tuple[float, float], float | None, str]] | None = None
        self._investigated: set[str] = set()
        self._sweep: dict | None = None   # active camera sweep during a search

    def _finish(self, state: SkillState, detail: str) -> None:
        self._status = SkillStatus(state, self._status.call, detail)

    def _running(self, detail: str) -> None:
        self._status = SkillStatus(SkillState.RUNNING, self._status.call, detail)

    def _need_map(self, inp: TickInput) -> bool:
        if inp.grid is not None:
            return False
        if inp.now - self._started_s > self.config.no_map_timeout_s:
            raise _SkillFailed("no map yet: drive around to build one first")
        self._running("waiting for map")
        return True

    def _map_settled(self, inp: TickInput, cm: Costmap, radius_m: float = 1.0, min_cells: int = 40) -> bool:
        """True once enough free space around the car is mapped to make decisions on.

        A fresh grid has no FREE cells until a few scans have been integrated; deciding then
        would read as "nothing to explore" or "no room to turn". Waits up to no_map_timeout_s."""
        from locatebot.lidar.grid import FREE
        c, r = cm.world_to_cell((inp.pose.x, inp.pose.y))
        k = int(math.ceil(radius_m / cm.resolution_m))
        win = cm.classes[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        if int((win == FREE).sum()) >= min_cells:
            return True
        if inp.now - self._started_s > self.config.no_map_timeout_s:
            raise _SkillFailed("the map around the car is still empty: is the LiDAR running?")
        self._running("waiting for the map to fill in")
        return False

    def _costmap_for(self, inp: TickInput, fresh: bool = False) -> Costmap:
        if fresh or self._costmap is None or inp.now - self._costmap_s >= self.config.replan_interval_s:
            self._costmap = build_costmap(inp.grid, self.vehicle, self.costmap_config)
            self._costmap_s = inp.now
        return self._costmap

    def _dispatch(self, call: SkillCall, inp: TickInput) -> DriveOutput:
        name = call.name
        if name == SkillName.STOP:
            self._finish(SkillState.SUCCEEDED, "stopped")
            return _ZERO
        if name == SkillName.GO_TO:
            return self._go_to(call, inp)
        if name == SkillName.RETURN_HOME:
            if self._need_map(inp):
                return _ZERO
            return self._drive_to(inp, (inp.home.x, inp.home.y), "home", self.config.point_standoff_m,
                                  approach_if_blocked=True)
        if name == SkillName.EXPLORE:
            return self._explore(inp, items=())
        if name == SkillName.SEARCH:
            items = tuple(call.args.get("items") or ())
            if not items:
                raise _SkillFailed("search needs at least one item")
            return self._explore(inp, items=items)
        if name == SkillName.LOOK_AROUND:
            return self._look_around(inp)
        if name == SkillName.FOLLOW:
            return self._follow(call, inp)
        raise _SkillFailed(f"unknown skill {name}")

    # ------------------------------------------------------------------ driving to a goal
    def _clearance(self, inp: TickInput) -> float | None:
        if inp.scan is None:
            return None
        c = corridor_clearance(clean_scan(inp.scan), self.vehicle.half_width_m)
        return c if math.isfinite(c) else None

    def _plan(self, inp: TickInput, cm: Costmap, goal, standoff: float | None, approach_if_blocked: bool):
        start = (inp.pose.x, inp.pose.y)
        if standoff is None:
            return plan(cm, start, goal, start_theta=inp.pose.theta)
        if approach_if_blocked and cm.is_traversable_xy(goal):
            return plan(cm, start, goal, start_theta=inp.pose.theta)
        return plan_approach(cm, start, goal, standoff, start_theta=inp.pose.theta)

    def _drive_to(self, inp: TickInput, goal, label: str, standoff: float | None,
                  approach_if_blocked: bool = False, finish: bool = True) -> DriveOutput:
        """Plan/follow to ``goal``. Returns zero output and sets SUCCEEDED (if ``finish``) on
        arrival; raises _SkillFailed after too many failed plans. ``self._arrived`` tells callers."""
        self._arrived = False
        goal = (float(goal[0]), float(goal[1]))
        moved_goal = self._goal is None or math.hypot(goal[0] - self._goal[0], goal[1] - self._goal[1]) > 0.25
        recheck = inp.now - self._last_check_s >= self.config.replan_interval_s
        if self._path is None and not moved_goal and not recheck:
            return _ZERO  # last plan failed: wait a replan interval (the map may grow) before retrying
        if moved_goal or self._path is None or recheck:
            cm = self._costmap_for(inp, fresh=moved_goal or self._path is None)
            if moved_goal or self._path is None or not path_clear(cm, self._remaining_path()):
                self._last_check_s = inp.now
                path = self._plan(inp, cm, goal, standoff, approach_if_blocked)
                self._goal, self._goal_label, self._goal_standoff = goal, label, standoff
                if path is None or len(path) < 2:
                    self._path = None
                    self._fails += 1
                    if self._fails > self.config.max_replans:
                        if not self._known(cm, goal):
                            raise _SkillFailed(f"{label} is in unmapped space: explore first")
                        raise _SkillFailed(f"no drivable path to {label}")
                    self._running(f"looking for a path to {label} ({self._fails})")
                    return _ZERO
                self._path = path
                self._pursuit = PurePursuit(self.vehicle, self.pursuit_config)
                self._pursuit.set_path(path)
            self._last_check_s = inp.now
        cmd = self._pursuit.update(inp.pose, clearance_m=self._clearance(inp))
        if cmd.status == PursuitStatus.REACHED:
            self._arrived = True
            self._path = None
            self._fails = 0
            if finish:
                self._finish(SkillState.SUCCEEDED, f"reached {label}")
            return _ZERO
        if cmd.status == PursuitStatus.NEEDS_REPLAN:
            self._path = None
            self._fails += 1
            if self._fails > self.config.max_replans:
                raise _SkillFailed(f"could not follow a path to {label}")
            self._running(f"replanning to {label}")
            return _ZERO
        self._fails = 0
        self._running(f"driving to {label}")
        return DriveOutput(cmd.throttle, cmd.steering)

    @staticmethod
    def _known(cm: Costmap, xy) -> bool:
        """True if any cell within 0.3 m of ``xy`` is mapped (free or occupied)."""
        from locatebot.lidar.grid import UNKNOWN
        c, r = cm.world_to_cell(xy)
        k = max(1, int(round(0.3 / cm.resolution_m)))
        win = cm.classes[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1]
        return bool(win.size) and bool((win != UNKNOWN).any())

    def _remaining_path(self):
        if self._path is None:
            return []
        return list(self._path)

    def _go_to(self, call: SkillCall, inp: TickInput) -> DriveOutput:
        item = call.args.get("item")
        if isinstance(item, str):
            track = inp.tracker.best(item) if inp.tracker is not None else None
            if track is None:
                raise _SkillFailed(f"haven't seen {item} yet: search for it first")
            if self._need_map(inp):
                return _ZERO
            return self._drive_to(inp, (track.x, track.y), item, self.config.standoff_m)
        if "x" in call.args and "y" in call.args:
            if self._need_map(inp):
                return _ZERO
            x, y = float(call.args["x"]), float(call.args["y"])
            return self._drive_to(inp, (x, y), f"({x:.1f}, {y:.1f})", self.config.point_standoff_m,
                                  approach_if_blocked=True)
        raise _SkillFailed("go_to needs an item or x and y")

    # ------------------------------------------------------------------ explore / search
    def _missing(self, inp: TickInput, items: tuple[str, ...]) -> list[str]:
        out = []
        for it in items:
            t = inp.tracker.best(it) if inp.tracker is not None else None
            if t is None or not t.confirmed:
                out.append(it)
        return out

    def _explore(self, inp: TickInput, items: tuple[str, ...]) -> DriveOutput:
        if items and not self._missing(inp, items):
            self._finish(SkillState.SUCCEEDED, "found all: " + ", ".join(items))
            return _ZERO
        if items and inp.now - self._started_s > self.config.search_timeout_s:
            missing = self._missing(inp, items)
            found = [i for i in items if i not in missing]
            raise _SkillFailed(f"search timed out after {self.config.search_timeout_s:.0f} s; not found: "
                               f"{', '.join(missing)}; found: {', '.join(found) or 'nothing'}")
        if self._need_map(inp):
            return _ZERO
        cm = self._costmap_for(inp)
        if not self._map_settled(inp, cm):
            self._costmap = None  # rebuild next tick as scans arrive
            return _ZERO
        self.explored_frac = explored_fraction(cm)
        if self._phase == "explore":
            out = self._frontier_step(inp, cm, items)
            if out is not None:
                return out
            if not items:
                return self._explore_done(inp, items)
            # The LiDAR map is complete, but the camera may not have looked everywhere yet.
            self._phase, self._patrol, self._path, self._goal = "patrol", None, None, None
        return self._search_patrol(inp, cm, items)

    def _frontier_step(self, inp: TickInput, cm: Costmap, items: tuple[str, ...]) -> DriveOutput | None:
        """Drive towards the current frontier; None once no reachable frontier is left."""
        pose = inp.pose
        if self._frontier is not None and not self._explorer.goal_still_useful(cm, self._frontier):
            self._explorer.mark_visited(self._frontier.goal_xy)
            self._frontier, self._path, self._goal = None, None, None
        for _ in range(self.config.explore_plan_attempts):
            if self._frontier is None:
                self._frontier = self._explorer.next_goal(cm, pose)
                self._path, self._goal, self._fails = None, None, 0
                if self._frontier is None:
                    return None
            try:
                out = self._drive_to(inp, self._frontier.goal_xy, "frontier", None, finish=False)
            except _SkillFailed:
                self._explorer.mark_failed(self._frontier.goal_xy, (pose.x, pose.y))
                self._frontier, self._path, self._goal = None, None, None
                continue
            if self._arrived:
                self._explorer.mark_visited(self._frontier.goal_xy)
                self._frontier, self._path, self._goal = None, None, None
                continue
            pct = f"{self.explored_frac:.0%}" if self.explored_frac is not None else "?"
            what = "searching for " + ", ".join(self._missing(inp, items)) if items else "exploring"
            self._running(f"{what} ({pct} explored)")
            return out
        self._running("choosing where to explore next")
        return _ZERO

    def _patrol_points(self, cm: Costmap, pose: Pose2D) -> list[tuple[float, float]]:
        """Waypoints about ``patrol_spacing_m`` apart over the reachable floor, nearest-first."""
        sc = start_cell(cm, (pose.x, pose.y))
        if sc is None:
            return []
        reach = reachable_mask(cm, sc)
        k = max(1, int(round(self.config.patrol_spacing_m / cm.resolution_m)))
        rows, cols = np.nonzero(reach)
        if rows.size == 0:
            return []
        # one representative cell per lattice square: the one farthest from obstacles
        keyed: dict[tuple[int, int], tuple[float, int, int]] = {}
        for r, c in zip(rows.tolist(), cols.tolist()):
            key = (r // k, c // k)
            clr = float(cm.occ_clearance_m[r, c])
            if key not in keyed or clr > keyed[key][0]:
                keyed[key] = (clr, r, c)
        pts = [cm.cell_to_world((c, r)) for clr, r, c in keyed.values() if clr >= 0.4]
        ordered, cur = [], (pose.x, pose.y)
        while pts and len(ordered) < self.config.max_patrol_points:
            i = min(range(len(pts)), key=lambda n: math.hypot(pts[n][0] - cur[0], pts[n][1] - cur[1]))
            cur = pts.pop(i)
            ordered.append((float(cur[0]), float(cur[1])))
        return ordered

    def _search_patrol(self, inp: TickInput, cm: Costmap, items: tuple[str, ...]) -> DriveOutput:
        missing = self._missing(inp, items)
        if self._sweep is not None:
            out = self._sweep_step(inp)
            if out is not None:
                self._running(f"searching for {', '.join(missing)}: looking around")
                return out
        if self._patrol is None:
            self._patrol = [(xy, None, "search point") for xy in self._patrol_points(cm, inp.pose)]
        # A single unconfirmed sighting: go and look again before anything else.
        for name in missing:
            t = inp.tracker.best(name) if inp.tracker is not None else None
            if t is not None and name not in self._investigated:
                self._investigated.add(name)
                self._patrol.insert(0, ((t.x, t.y), self.config.standoff_m + 0.4, name))
                self._path, self._goal = None, None
        while self._patrol:
            goal, standoff, label = self._patrol[0]
            try:
                out = self._drive_to(inp, goal, label, standoff, approach_if_blocked=True, finish=False)
            except _SkillFailed:
                self._patrol.pop(0)
                self._path, self._goal, self._fails = None, None, 0
                continue
            if self._arrived:
                self._patrol.pop(0)
                self._path, self._goal, self._fails = None, None, 0
                self._start_sweep(inp, cm)
                return _ZERO
            self._running(f"searching for {', '.join(missing)} ({len(self._patrol)} places left to check)")
            return out
        return self._explore_done(inp, items)

    def _start_sweep(self, inp: TickInput, cm: Costmap) -> None:
        side = next((sd for sd in (1.0, -1.0) if self._circle_fits(cm, inp.pose, sd)), None)
        self._sweep = None if side is None else {"side": side, "turned": 0.0, "theta": inp.pose.theta,
                                                  "t0": inp.now}

    def _sweep_step(self, inp: TickInput) -> DriveOutput | None:
        sw = self._sweep
        sw["turned"] += abs(wrap_angle(inp.pose.theta - sw["theta"]))
        sw["theta"] = inp.pose.theta
        if sw["turned"] >= 2 * math.pi or inp.now - sw["t0"] > self.config.look_around_timeout_s:
            self._sweep = None
            return None
        return DriveOutput(self.vehicle.max_throttle * self.config.sweep_throttle_frac, sw["side"])

    def _explore_done(self, inp: TickInput, items: tuple[str, ...]) -> DriveOutput:
        pct = f"{self.explored_frac:.0%}" if self.explored_frac is not None else "?"
        if not items:
            self._finish(SkillState.SUCCEEDED, f"explored {pct} of the reachable area")
            return _ZERO
        missing = self._missing(inp, items)
        found = [i for i in items if i not in missing]
        if not missing:
            self._finish(SkillState.SUCCEEDED, "found all: " + ", ".join(items))
        else:
            self._finish(SkillState.FAILED, f"explored {pct} and checked the room; not found: "
                                            f"{', '.join(missing)}; found: {', '.join(found) or 'nothing'}")
        return _ZERO

    # ------------------------------------------------------------------ look around
    def _circle_fits(self, cm: Costmap, pose: Pose2D, side: float) -> bool:
        r = self.vehicle.min_turn_radius_m * 1.1
        cx = pose.x - side * r * math.sin(pose.theta)
        cy = pose.y + side * r * math.cos(pose.theta)
        for k in range(36):
            a = 2 * math.pi * k / 36
            if not cm.is_traversable_xy((cx + r * math.cos(a), cy + r * math.sin(a))):
                return False
        return True

    def _look_around(self, inp: TickInput) -> DriveOutput:
        if self._need_map(inp):
            return _ZERO
        if self._last_theta is None:
            cm = self._costmap_for(inp, fresh=True)
            if not self._map_settled(inp, cm):
                return _ZERO
            side = next((s for s in (1.0, -1.0) if self._circle_fits(cm, inp.pose, s)), None)
            if side is None:
                raise _SkillFailed("not enough room here to turn a full circle")
            self._side = side
            self._last_theta = inp.pose.theta
        self._turned += abs(wrap_angle(inp.pose.theta - self._last_theta))
        self._last_theta = inp.pose.theta
        if self._turned >= 2 * math.pi - math.radians(10):
            self._finish(SkillState.SUCCEEDED, "looked all around")
            return _ZERO
        if inp.now - self._started_s > self.config.look_around_timeout_s:
            raise _SkillFailed("look_around took too long (blocked?)")
        self._running(f"looking around ({math.degrees(self._turned):.0f}°)")
        return DriveOutput(self.vehicle.max_throttle * self.config.look_around_throttle_frac, self._side)

    # ------------------------------------------------------------------ follow
    def _update_target(self, inp: TickInput, item: str) -> None:
        track = inp.tracker.best(item) if inp.tracker is not None else None
        if track is not None and track.last_seen_s > self._target_fix_s:
            self._target = (track.x, track.y)  # fresh vision fix wins
            self._target_fix_s = self._target_seen_s = track.last_seen_s
            return
        if item.strip().lower() in ("person", "people") and inp.people and self._follow_person(inp):
            return
        if self._target is None or inp.scan is None:
            return
        # Between vision fixes: follow the LiDAR cluster nearest the last known position.
        best, best_d = None, self.config.follow_gate_m
        for ob in cluster_scan(clean_scan(inp.scan)):
            if ob.width_m > 0.8:
                continue  # walls and furniture, not a person or a carried item
            wx, wy = inp.pose.transform_points(np.array([ob.centroid_xy]))[0]
            d = math.hypot(wx - self._target[0], wy - self._target[1])
            if d < best_d:
                best, best_d = (float(wx), float(wy)), d
        if best is not None:
            self._target = best
            self._target_seen_s = inp.now

    def _follow_person(self, inp: TickInput) -> bool:
        """follow(person) between vision fixes: the people tracker's track nearest the last target
        (or, before any fix, the nearest person). Tracks carry their own velocity and identity, so
        a chair leg or a wall corner near the person can't steal the target as a raw cluster can."""
        best, best_d = None, math.inf
        for t in inp.people:
            if not getattr(t, "is_person", True):
                continue
            if self._target is not None:
                d = math.hypot(t.x - self._target[0], t.y - self._target[1])
                if d > self.config.follow_person_gate_m:
                    continue
            else:
                d = math.hypot(t.x - inp.pose.x, t.y - inp.pose.y)
            if d < best_d:
                best, best_d = t, d
        if best is None:
            return False
        self._target = (float(best.x), float(best.y))
        self._target_seen_s = max(self._target_seen_s, min(inp.now, float(best.last_seen_s)))
        return True

    def _follow(self, call: SkillCall, inp: TickInput) -> DriveOutput:
        item = call.args.get("item")
        if not isinstance(item, str):
            raise _SkillFailed("follow needs an item")
        if self._need_map(inp):
            return _ZERO
        self._update_target(inp, item)
        if self._target is None:
            raise _SkillFailed(f"haven't seen {item} yet: search for it first")
        if inp.now - self._target_seen_s > self.config.follow_lost_s:
            raise _SkillFailed(f"lost {item}: not seen for {self.config.follow_lost_s:.0f} s")
        d = math.hypot(self._target[0] - inp.pose.x, self._target[1] - inp.pose.y)
        if d <= self.config.follow_distance_m + 0.2:
            self._path, self._goal = None, None
            self._running(f"holding {d:.1f} m from {item}")
            return _ZERO
        out = self._drive_to(inp, self._target, item, self.config.follow_distance_m, finish=False)
        if self._arrived:
            self._running(f"holding near {item}")
            return _ZERO
        self._running(f"following {item} ({d:.1f} m)")
        return out
