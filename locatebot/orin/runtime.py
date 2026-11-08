"""Orin runtime: wires live mapping, vision, item localization, the skill executor and the LLM brain.

Data flow (one ``OrinRuntime`` per robot session)::

    Pi scans  --on_scan-->  LiveMapper (pose + grid)
    Pi frames --on_frame--> VisionService --results--> localize (bearing + LiDAR range + pose)
                                                       --> ItemTracker
    control loop --step(now)--> SkillExecutor.tick --> requested (throttle, steering) --> Pi
                             \\-> WorldState --> Brain (LLM) --SkillCall--> queue --> executor

Threads: scans, frames, vision results, the LLM thread and the control loop may all be
different threads. Shared state is guarded by one lock, and skill calls from the brain only go
into a queue drained by ``step``: the brain calls ``on_skill`` while holding its own lock, so
calling into the executor directly from there could deadlock against ``step``.

The Pi's safety supervisor stays the final gate: everything here is a *request*.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable

from locatebot.lidar.types import Pose2D, Scan
from locatebot.localize.items import ItemTracker, LocalizeConfig, localize
from locatebot.mission.types import SkillCall, SkillName, SkillState, WorldState
from locatebot.vision.service import VisionResult

from .live_map import LiveMapper
from .skills import DriveOutput, SkillExecutor, TickInput
from .world import build_world_state

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RuntimeConfig:
    localize: LocalizeConfig = LocalizeConfig()
    min_sightings: int = 2           # sightings before an item track counts as confirmed
    person_check_s: float = 5.0      # M3: how long vision looks for "person" per tracker request
    person_window_rad: float = math.radians(5.0)  # M3: LiDAR range search half-width for people
    merge_radius_m: float = 0.5
    event_keep: int = 20


class OrinRuntime:
    def __init__(self, mapper: LiveMapper, executor: SkillExecutor, *, brain=None, vision=None,
                 config: RuntimeConfig | None = None, clock: Callable[[], float] = time.monotonic) -> None:
        self.mapper = mapper
        self.executor = executor
        self.brain = brain            # locatebot.agent.brain.Brain or None (operator-only)
        self.vision = vision          # locatebot.vision.service.VisionService or None
        self.config = config or RuntimeConfig()
        self.clock = clock
        self.tracker = ItemTracker(merge_radius_m=self.config.merge_radius_m,
                                   min_sightings=self.config.min_sightings)
        self._lock = threading.RLock()
        self._calls: deque[SkillCall] = deque()
        self._events: deque[tuple[float, str]] = deque(maxlen=self.config.event_keep)
        self._last_event = ""
        self._safety_reason = "ok"
        self._vision_items: tuple[str, ...] = ()
        self._last_state = self.executor.status.state
        self._confirmed: set[str] = set()
        self._stats = {"scans": 0, "frames": 0, "vision_results": 0, "fixes": 0, "no_range": 0, "steps": 0}
        self._person_check_until = -math.inf  # M3: low-priority vision "person" check window

    # ------------------------------------------------------------------ inputs (any thread)
    def on_scan(self, scan: Scan) -> None:
        self.mapper.add_scan(scan)
        with self._lock:
            self._stats["scans"] += 1

    def on_frame(self, jpeg: bytes, capture_s: float) -> None:
        with self._lock:
            self._stats["frames"] += 1
        if self.vision is not None:
            self.vision.submit_frame(jpeg, capture_s)

    def on_range(self, msg) -> None:
        """Ultrasonic ``link.messages.Range`` from the Pi (M3): glass / low obstacle layer."""
        self.mapper.on_range(msg.stamp_s, msg.range_m, msg.fov_rad)

    def on_vision_result(self, result: VisionResult) -> None:
        """VisionService callback: place each detection on the map with the LiDAR range."""
        pose = self.mapper.pose_at(result.capture_s)
        scan = self.mapper.scan_near(result.capture_s)
        with self._lock:
            self._stats["vision_results"] += 1
            if pose is None or scan is None:
                return
            is_person = result.item.strip().lower() in ("person", "people")
            # a box centre often falls between a person's legs: look a little wider for the range
            lcfg = (replace(self.config.localize, window_rad=max(self.config.localize.window_rad,
                                                                 self.config.person_window_rad))
                    if is_person else self.config.localize)
            for f in result.found:
                fix = localize(f.bearing_rad, scan, pose, lcfg, stamp_s=result.capture_s)
                if fix is None:
                    self._stats["no_range"] += 1  # no LiDAR return along the bearing: no position
                    continue
                if is_person:  # M3: confirms a LiDAR person track
                    self.mapper.people_camera_fix((fix.x, fix.y), result.capture_s)
                    if result.item not in self.executor.search_items:
                        continue  # our own low-priority check: not an item the user asked for
                track = self.tracker.update(result.item, fix)
                self._stats["fixes"] += 1
                if track.confirmed and result.item not in self._confirmed:
                    self._confirmed.add(result.item)
                    self._event(result.capture_s, f"found {result.item} at ({track.x:.1f}, {track.y:.1f})")

    def on_skill(self, call: SkillCall) -> None:
        """Brain callback (LLM thread, brain lock held): only enqueue."""
        with self._lock:
            self._calls.append(call)

    def submit_operator(self, call: SkillCall) -> None:
        """Operator command from a dashboard: goes through the brain when there is one so the
        brain knows (operator STOP pauses LLM decisions), else straight to the executor."""
        if self.brain is not None:
            self.brain.submit_operator(call)
        else:
            self.on_skill(replace(call, source="operator"))

    def set_safety_reason(self, reason: str) -> None:
        with self._lock:
            if reason != self._safety_reason and reason not in ("ok", "slowed"):
                self._event(self.clock(), f"Pi safety stop: {reason}")
            self._safety_reason = reason

    # ------------------------------------------------------------------ control loop
    def step(self, now: float | None = None) -> DriveOutput:
        """One control tick: run queued skill calls, tick the executor, feed the brain."""
        now = self.clock() if now is None else now
        with self._lock:
            self._stats["steps"] += 1
            while self._calls:
                call = self._calls.popleft()
                prev = self.executor.status
                if prev.state == SkillState.RUNNING and prev.call is not None:
                    self._event(now, f"{prev.call.name.value} interrupted by {call.name.value}")
                self.executor.start(call, now)
            inp = TickInput(now=now, pose=self.mapper.pose, grid=self.mapper.grid_snapshot(),
                            scan=self.mapper.latest_scan, tracker=self.tracker, home=self.mapper.home,
                            people=tuple(self.mapper.people()))
            out = self.executor.tick(inp)
            st = self.executor.status
            if st.state != self._last_state and st.state in (SkillState.SUCCEEDED, SkillState.FAILED,
                                                             SkillState.CANCELLED) and st.call is not None:
                self._event(now, f"{st.call.name.value} {st.state.value}: {st.detail}")
            self._last_state = st.state
            items = self.executor.search_items
            if not items and self.vision is not None:
                items = self._person_check(now)
            ws = self._world_state(now, inp)
        if self.vision is not None and items != self._vision_items:
            self._vision_items = items
            self.vision.set_items(list(items))
        if self.brain is not None:
            self.brain.on_world_state(ws)
        return out

    def _person_check(self, now: float) -> tuple[str, ...]:
        """M3: vision looks for "person" for ``person_check_s`` when the people tracker has a
        likely person without a recent camera confirmation, and only when no search runs."""
        if now < self._person_check_until:
            return ("person",)
        t = self.mapper.people_wants_camera(now)
        if t is None:
            return ()
        self.mapper.people_camera_requested(t.id, now)
        self._person_check_until = now + self.config.person_check_s
        return ("person",)

    def world_state(self, now: float | None = None) -> WorldState:
        now = self.clock() if now is None else now
        with self._lock:
            inp = TickInput(now=now, pose=self.mapper.pose, grid=None, scan=self.mapper.latest_scan,
                            tracker=self.tracker, home=self.mapper.home)
            return self._world_state(now, inp)

    def _world_state(self, now: float, inp: TickInput) -> WorldState:
        call = self.executor.status.call
        search = tuple(call.args.get("items", ())) if call is not None and call.name == SkillName.SEARCH else ()
        return build_world_state(
            now=now, pose=inp.pose, scan=inp.scan, tracker=self.tracker, skill=self.executor.status,
            search_items=search, explored_frac=self.executor.explored_frac,
            safety_reason=self._safety_reason, mission_state=self.executor.status.state.value,
            last_event=self._last_event, changes=self.mapper.change_blobs(),
            people=self.mapper.people(), notes=self.mapper.notes(now))

    def _event(self, t: float, text: str) -> None:
        self._events.append((t, text))
        self._last_event = text
        log.info("event: %s", text)

    # ------------------------------------------------------------------ status
    def status(self) -> dict[str, Any]:
        with self._lock:
            st = self.executor.status
            p = self.mapper.pose
            out = {
                "pose": {"x": round(p.x, 3), "y": round(p.y, 3), "theta_deg": round(math.degrees(p.theta), 1)},
                "skill": {"state": st.state.value, "call": None if st.call is None else
                          {"name": st.call.name.value, "args": dict(st.call.args), "source": st.call.source},
                          "detail": st.detail},
                "explored_frac": self.executor.explored_frac,
                "items": [{"name": n, "x": round(t.x, 2), "y": round(t.y, 2), "sightings": t.sightings,
                           "confirmed": t.confirmed} for n in self.tracker.items() for t in self.tracker.tracks(n)],
                "events": [{"t": round(t, 2), "text": e} for t, e in self._events],
                "safety_reason": self._safety_reason,
                "map": self.mapper.stats(),
                "stats": dict(self._stats),
            }
        if self.brain is not None:
            out["brain"] = self.brain.status()
        if self.vision is not None:
            out["vision"] = self.vision.status()
        return out
