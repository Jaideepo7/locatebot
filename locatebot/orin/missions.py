"""Mission bridge on the Orin: Telegram/operator missions from the Pi -> skills, status back.

The Pi (``locatebot/remote``) publishes ``MissionRequest`` / ``MissionCancel`` on the link; the
Orin's subscriber hands them to ``on_request`` / ``on_cancel`` (any thread, never blocks).
``tick(now, ws)`` runs from the control loop (``OrinRuntime.step``) and does all the work:

* **Validation** (ported from chanabyte's ``JetsonAgent``, feat/photon-dispatch): a request that
  is expired, future-dated beyond ``max_skew_s`` (5 s), has ``expires <= issued``, or arrives
  while another mission is active is answered ``FAILED`` with reason ``EXPIRED`` /
  ``CLOCK_SKEW`` / ``INVALID_EXPIRY`` / ``BUSY`` (the Pi's ``ZmqRobotLink`` turns a first-status
  ``FAILED`` with one of these reasons into a rejection). A duplicate id is never started twice:
  the last status for it is sent again. Mission times are Unix wall clock (``wall_clock``).
* **Start**: ``runtime.set_goal("find <target>", source="mission")``. With an LLM brain the brain
  acts (approve or auto, as configured; a brain paused by an earlier STOP is resumed). Without
  one (``--llm none``) the routine runs ``search([target])`` then, on success,
  ``go_to(item=target)`` through ``runtime.submit_operator``.
* **Status**, only from what the executor and tracker really show, each state once, in the
  allowed transition order (missing intermediate states are filled in only when they are true,
  e.g. ``TARGET_DETECTED`` before ``APPROACHING`` needs a track of the target):
  ``ACCEPTED`` on start; ``SEARCHING`` when a search/explore/look_around runs; ``TARGET_DETECTED``
  when a track of the target is confirmed (message has the map position); ``APPROACHING`` when
  ``go_to(item=target)`` runs; ``COMPLETED`` when it succeeds; ``FAILED`` with ``NOT_FOUND``
  (a search for the target ended without it; message = the executor's found/missing text),
  ``NO_PATH``, ``TIMEOUT`` (mission TTL), ``ROBOT_FAULT`` (executor internal error) or
  ``STOPPED`` (someone else stopped the robot mid-mission); ``CANCELLED`` after a cancel once
  the executor has stopped. A ``go_to(target)`` that loses the target sends the mission back to
  ``SEARCHING`` when the next search runs (routine mode re-searches by itself, at most twice).
* **Link loss** (no Pi scan for ``link_timeout_s`` = 2 s): operator STOP, the mission state is
  kept; when scans come back the current state is re-reported and the mission resumes (routine:
  the current step is issued again; brain: resumed). The Pi's own safety stop already stops the
  car when the Orin goes quiet; this is the Orin-side half.

Target matching is case-insensitive exact equality of the normalised description with the item
track name, which is the same text vision searches for: wording matters (see
``docs/vision/README.md``: short nouns such as "marker" work, "dry erase marker pen" may not).

Nothing here produces throttle or steering: the bridge only issues skill calls, and the Pi's
safety supervisor stays the final gate on motion.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from typing import Callable, Optional

from locatebot.link.messages import MAX_MESSAGE_CHARS, MissionCancel, MissionRequest, MissionStatus
from locatebot.mission.types import SkillCall, SkillName, SkillState, WorldState

log = logging.getLogger(__name__)

PENDING, ACCEPTED, SEARCHING, DETECTED, APPROACHING, COMPLETED, FAILED, CANCELLED = (
    "PENDING", "ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING", "COMPLETED", "FAILED", "CANCELLED")
TERMINAL = frozenset({COMPLETED, FAILED, CANCELLED})
_END = {FAILED, CANCELLED}
# Same table as locatebot/remote/protocol.py (tested equal); duplicated so the Orin does not import
# the Pi's remote-control package.
TRANSITIONS = {
    PENDING: {ACCEPTED} | _END,
    ACCEPTED: {SEARCHING} | _END,
    SEARCHING: {DETECTED} | _END,
    DETECTED: {APPROACHING, SEARCHING, COMPLETED} | _END,
    APPROACHING: {COMPLETED, SEARCHING} | _END,
}
REJECT_REASONS = ("BUSY", "EXPIRED", "CLOCK_SKEW", "INVALID_EXPIRY")
_SEARCHING_SKILLS = (SkillName.SEARCH, SkillName.EXPLORE, SkillName.LOOK_AROUND)
_PATH_FAILURES = ("no drivable path", "unmapped space", "could not follow a path")


def can_transition(old: str, new: str) -> bool:
    return new in TRANSITIONS.get(old, ())


def _one_line(text, limit: int = MAX_MESSAGE_CHARS) -> str:
    s = " ".join("".join(ch if ch.isprintable() else " " for ch in str(text)).split())
    return s if len(s) <= limit else s[: limit - 1] + "…"


@dataclass
class _Mission:
    mission_id: str
    target: str
    expires_s: float
    state: str = ACCEPTED
    step: str = "search"            # routine mode: "search" | "go_to"
    seq: int = 0                    # counter for unique call reasons
    call_reason: str = ""           # reason text of the routine call we issued last
    researches: int = 0
    lost_at: Optional[float] = None  # ws stamp when go_to lost the target
    cancel_at: Optional[float] = None
    stop_reasons: set = field(default_factory=set)  # reason texts of our own STOP calls
    pre_call: object = None         # the executor's call when the mission started (ignored)
    done_call: object = None        # last terminal (call, state) processed
    done_state: object = None
    link_down: bool = False
    status: Optional[MissionStatus] = None
    started_wall: float = 0.0
    history: list = field(default_factory=list)


class MissionBridge:
    def __init__(self, runtime, publish: Callable[[MissionStatus], None], clock: Callable[[], float] = time.time,
                 *, max_skew_s: float = 5.0, link_timeout_s: float = 2.0, max_researches: int = 2,
                 cancel_grace_s: float = 3.0, seen_capacity: int = 1000,
                 last_scan_s: Optional[Callable[[], Optional[float]]] = None) -> None:
        self.runtime = runtime
        self._publish = publish
        self._wall = clock                       # Unix wall clock: mission times are wall clock
        self.max_skew_s = max_skew_s
        self.link_timeout_s = link_timeout_s
        self.max_researches = max_researches
        self.cancel_grace_s = cancel_grace_s
        self._seen_capacity = seen_capacity
        self._last_scan_s = last_scan_s or self._mapper_last_scan
        self._lock = threading.Lock()
        self._inbox: deque = deque()
        self._seen: "OrderedDict[str, MissionStatus]" = OrderedDict()  # id -> last status sent
        self._active: Optional[_Mission] = None
        self._link_up = True
        self._stats = {"requests": 0, "rejected": 0, "duplicates": 0, "cancels": 0, "statuses": 0,
                       "link_losses": 0}

    # ------------------------------------------------------------------ inputs (any thread)
    def on_request(self, req: MissionRequest) -> None:
        with self._lock:
            self._inbox.append(req)

    def on_cancel(self, cancel: MissionCancel) -> None:
        with self._lock:
            self._inbox.append(cancel)

    def on_message(self, msg) -> bool:
        """Convenience for subscribers: route a decoded link message. True if it was a mission one."""
        if isinstance(msg, MissionRequest):
            self.on_request(msg)
        elif isinstance(msg, MissionCancel):
            self.on_cancel(msg)
        else:
            return False
        return True

    def status(self) -> dict:
        with self._lock:
            m = self._active
            return {"active": None if m is None else {
                        "mission_id": m.mission_id, "target": m.target, "state": m.state, "step": m.step,
                        "link_down": m.link_down, "expires_in_s": round(m.expires_s - self._wall(), 1)},
                    "link_up": self._link_up, **self._stats}

    # ------------------------------------------------------------------ control loop
    def tick(self, now: float, ws: Optional[WorldState] = None) -> None:
        """Called from the control loop after ``runtime.step``'s executor tick. Never blocks:
        all runtime calls are queue-style (``submit_operator``, ``set_goal``)."""
        if ws is None:
            ws = self.runtime.world_state(now)
        actions: list[Callable[[], None]] = []
        out: list[MissionStatus] = []
        with self._lock:
            while self._inbox:
                msg = self._inbox.popleft()
                if isinstance(msg, MissionRequest):
                    self._handle_request(msg, ws, actions, out)
                else:
                    self._handle_cancel(msg, ws, actions, out)
            self._check_link(now, ws, actions, out)
            m = self._active
            if m is not None:
                self._update(m, ws, actions, out)
        for fn in actions:
            try:
                fn()
            except Exception:
                log.exception("mission action failed")
        for st in out:
            try:
                self._publish(st)
            except Exception:
                log.exception("publishing %s failed", st)

    # ------------------------------------------------------------------ requests / cancels
    def _handle_request(self, req: MissionRequest, ws: WorldState, actions, out) -> None:
        self._stats["requests"] += 1
        mid = req.mission_id
        if mid in self._seen:  # replay or resend: same answer, never start twice
            self._stats["duplicates"] += 1
            out.append(self._seen[mid])
            return
        now_wall = self._wall()
        reason = None
        if not (math.isfinite(req.issued_s) and math.isfinite(req.expires_s)) or req.expires_s <= req.issued_s:
            reason = "INVALID_EXPIRY"
        elif req.issued_s - now_wall > self.max_skew_s:
            reason = "CLOCK_SKEW"
        elif now_wall >= req.expires_s:
            reason = "EXPIRED"
        elif self._active is not None:
            reason = "BUSY"
        if reason:
            self._stats["rejected"] += 1
            text = {"BUSY": f"busy with another mission ({self._active.target})" if self._active else "busy",
                    "EXPIRED": "the request expired before it arrived",
                    "CLOCK_SKEW": "the request is dated in the future: check the Pi and Orin clocks (NTP)",
                    "INVALID_EXPIRY": "the request expires before it was issued"}[reason]
            st = MissionStatus(mid, FAILED, _one_line(f"rejected: {text}"), reason)
            self._remember(mid, st)
            out.append(st)
            return
        m = _Mission(mid, req.target, req.expires_s, started_wall=now_wall)
        m.pre_call = ws.skill.call
        self._active = m
        brain = self.runtime.brain
        goal = f"find {req.target}"

        def start():
            try:
                self.runtime.set_goal(goal, source="mission")
            except ValueError:
                log.warning("mission %s: goal %r not accepted", mid, goal)
            if brain is not None and brain.status().get("paused"):
                brain.resume()

        actions.append(start)
        if brain is None:
            self._routine(m, SkillName.SEARCH, actions)
        self._emit(m, ACCEPTED, f"searching for {req.target}" if brain is None else f"goal: {goal}", "", out)
        log.info("mission %s started: find %r", mid, req.target)

    def _handle_cancel(self, msg: MissionCancel, ws: WorldState, actions, out) -> None:
        self._stats["cancels"] += 1
        m = self._active
        if m is None or m.mission_id != msg.mission_id:
            st = self._seen.get(msg.mission_id)
            if st is not None and st.state in TERMINAL:
                out.append(st)  # already over: tell the Pi again
            log.info("cancel for inactive mission %s ignored", msg.mission_id)
            return
        if m.cancel_at is None:
            m.cancel_at = ws.stamp_s
            self._stop(m, "cancel", actions)
            actions.append(self.runtime.clear_goal)

    # ------------------------------------------------------------------ link loss
    def _mapper_last_scan(self) -> Optional[float]:
        scan = self.runtime.mapper.latest_scan
        return None if scan is None else float(scan.stamp_s)

    def _check_link(self, now: float, ws: WorldState, actions, out) -> None:
        last = self._last_scan_s()
        up = last is not None and now - last <= self.link_timeout_s
        self._link_up = up
        m = self._active
        if m is None:
            return
        if not up and not m.link_down:
            self._stats["link_losses"] += 1
            m.link_down = True
            log.warning("mission %s: no scans from the Pi for %.1f s, stopping", m.mission_id,
                        self.link_timeout_s)
            self._stop(m, "link lost", actions)
        elif up and m.link_down:
            m.link_down = False
            log.info("mission %s: Pi link back, re-reporting %s", m.mission_id, m.state)
            if m.status is not None:
                out.append(m.status)
            if m.cancel_at is None:  # resume what we were doing
                if self.runtime.brain is not None:
                    actions.append(self.runtime.brain.resume)
                else:
                    self._routine(m, SkillName.GO_TO if m.step == "go_to" else SkillName.SEARCH, actions)

    # ------------------------------------------------------------------ the state machine
    def _update(self, m: _Mission, ws: WorldState, actions, out) -> None:
        sk = ws.skill
        call = sk.call
        if self._wall() >= m.expires_s:
            self._stop(m, "timeout", actions)
            actions.append(self.runtime.clear_goal)
            self._finish(m, FAILED, f"mission time limit reached while {m.state.lower().replace('_', ' ')}",
                         "TIMEOUT", out)
            return
        if m.cancel_at is not None:
            stopped = call is not None and call.name == SkillName.STOP and sk.state != SkillState.RUNNING \
                and call.reason in m.stop_reasons
            idle = sk.state != SkillState.RUNNING and ws.stamp_s - m.cancel_at >= self.cancel_grace_s
            if stopped or idle:
                self._finish(m, CANCELLED, "stopped on request", "", out)
            return
        if m.link_down or call is None or call is m.pre_call:
            self._detect(m, ws, out)
            return
        name = call.name
        mine = self._matches(call)
        if sk.state == SkillState.RUNNING:
            if name in _SEARCHING_SKILLS:
                if m.state == ACCEPTED or (m.state in (DETECTED, APPROACHING) and m.lost_at is not None):
                    self._emit(m, SEARCHING, f"searching for {m.target}", "", out)
            elif name == SkillName.GO_TO and mine and m.state != APPROACHING:
                self._detect(m, ws, out)
                self._to(m, APPROACHING, f"driving to {m.target}" + self._where(m, ws), ws, out)
            self._detect(m, ws, out)
            return
        # terminal skill state: process each (call, state) once
        if call is m.done_call and sk.state == m.done_state:
            self._detect(m, ws, out)
            return
        m.done_call, m.done_state = call, sk.state
        detail = _one_line(sk.detail, 200)
        if sk.state == SkillState.FAILED and detail.startswith("internal error"):
            self._stop(m, "fault", actions)
            actions.append(self.runtime.clear_goal)
            self._finish(m, FAILED, f"robot fault in {name.value}: {detail}", "ROBOT_FAULT", out)
            return
        if name == SkillName.STOP:
            if sk.state == SkillState.SUCCEEDED and call.reason not in m.stop_reasons:
                actions.append(self.runtime.clear_goal)
                why = _one_line(call.reason, 120) or call.source
                self._finish(m, FAILED, f"robot stopped by {call.source}: {why}", "STOPPED", out)
            return
        if name == SkillName.SEARCH and mine:
            self._detect(m, ws, out)
            if sk.state == SkillState.SUCCEEDED:
                if self.runtime.brain is None and m.step == "search":
                    self._routine(m, SkillName.GO_TO, actions)
            elif sk.state == SkillState.FAILED and m.state not in (DETECTED, APPROACHING):
                actions.append(self.runtime.clear_goal)
                self._finish(m, FAILED, detail or f"{m.target} not found", "NOT_FOUND", out)
            return
        if name == SkillName.GO_TO and mine:
            if sk.state == SkillState.SUCCEEDED:
                self._to(m, COMPLETED, f"arrived at {m.target}" + self._where(m, ws), ws, out)
                if m.state == COMPLETED:
                    actions.append(self.runtime.clear_goal)
                    self._active = None
            elif sk.state == SkillState.FAILED:
                if any(p in detail for p in _PATH_FAILURES):
                    actions.append(self.runtime.clear_goal)
                    self._finish(m, FAILED, detail, "NO_PATH", out)
                else:  # lost / not seen: back to searching
                    m.lost_at = ws.stamp_s
                    if self.runtime.brain is None:
                        if m.researches >= self.max_researches:
                            actions.append(self.runtime.clear_goal)
                            self._finish(m, FAILED, f"lost {m.target} and could not find it again: {detail}",
                                         "NOT_FOUND", out)
                            return
                        m.researches += 1
                        self._routine(m, SkillName.SEARCH, actions)
            return
        self._detect(m, ws, out)

    # ------------------------------------------------------------------ helpers (lock held)
    def _matches(self, call: SkillCall) -> bool:
        m = self._active
        t = m.target.casefold()
        if call.name == SkillName.SEARCH:
            return any(isinstance(i, str) and i.casefold() == t for i in call.args.get("items", ()))
        if call.name in (SkillName.GO_TO, SkillName.FOLLOW):
            item = call.args.get("item")
            return isinstance(item, str) and item.casefold() == t
        return False

    def _track(self, m: _Mission, ws: WorldState, confirmed: bool = False):
        t = m.target.casefold()
        views = [v for v in ws.items if v.name.casefold() == t and (v.confirmed or not confirmed)]
        if not views:
            return None
        return max(views, key=lambda v: (v.confirmed, v.sightings))

    def _where(self, m: _Mission, ws: WorldState) -> str:
        v = self._track(m, ws)
        return "" if v is None else f" at ({v.x:.1f}, {v.y:.1f}) m"

    def _detect(self, m: _Mission, ws: WorldState, out) -> None:
        """SEARCHING -> TARGET_DETECTED once the target's track is confirmed (after a loss: a
        sighting newer than the loss)."""
        if m.state not in (ACCEPTED, SEARCHING):
            return
        v = self._track(m, ws, confirmed=True)
        if v is None:
            return
        if m.lost_at is not None and ws.stamp_s - v.last_seen_age_s <= m.lost_at:
            return
        if m.state == ACCEPTED:
            self._emit(m, SEARCHING, f"{m.target} is already on the map", "", out)
        m.lost_at = None
        self._emit(m, DETECTED, f"found {m.target} at ({v.x:.1f}, {v.y:.1f}) m, "
                                f"{v.range_m:.1f} m from the robot", "", out)

    def _to(self, m: _Mission, state: str, message: str, ws: WorldState, out) -> None:
        """Go to ``state`` through the true intermediate states."""
        if m.state in (ACCEPTED, SEARCHING) and state in (APPROACHING, COMPLETED):
            v = self._track(m, ws)
            if v is None:
                return
            if m.state == ACCEPTED:
                self._emit(m, SEARCHING, f"{m.target} is already on the map", "", out)
            m.lost_at = None
            self._emit(m, DETECTED, f"found {m.target} at ({v.x:.1f}, {v.y:.1f}) m", "", out)
        self._emit(m, state, message, "", out)  # TARGET_DETECTED -> COMPLETED is allowed too

    def _emit(self, m: _Mission, state: str, message: str, reason: str, out) -> None:
        if state == m.state and m.status is not None:
            return
        if m.status is not None and not can_transition(m.state, state):
            log.debug("mission %s: %s -> %s not allowed", m.mission_id, m.state, state)
            return
        m.state = state
        st = MissionStatus(m.mission_id, state, _one_line(message), reason)
        m.status = st
        m.history.append(state)
        self._remember(m.mission_id, st)
        self._stats["statuses"] += 1
        out.append(st)
        log.info("mission %s: %s %s", m.mission_id, state, st.message)

    def _finish(self, m: _Mission, state: str, message: str, reason: str, out) -> None:
        """End the mission with FAILED or CANCELLED (allowed from every non-terminal state)."""
        self._emit(m, state, message, reason, out)
        if m.state in TERMINAL:
            self._active = None

    def _remember(self, mid: str, st: MissionStatus) -> None:
        self._seen[mid] = st
        self._seen.move_to_end(mid)
        while len(self._seen) > self._seen_capacity:
            self._seen.popitem(last=False)

    def _routine(self, m: _Mission, name: SkillName, actions) -> None:
        m.seq += 1
        if name == SkillName.SEARCH:
            m.step = "search"
            call = SkillCall(SkillName.SEARCH, {"items": (m.target,)},
                             reason=f"mission {m.mission_id[:8]} step {m.seq}: search for {m.target}")
        else:
            m.step = "go_to"
            call = SkillCall(SkillName.GO_TO, {"item": m.target},
                             reason=f"mission {m.mission_id[:8]} step {m.seq}: go to {m.target}")
        m.call_reason = call.reason
        actions.append(lambda c=call: self.runtime.submit_operator(c))

    def _stop(self, m: _Mission, why: str, actions) -> None:
        m.seq += 1
        call = SkillCall(SkillName.STOP, {}, reason=f"mission {m.mission_id[:8]} step {m.seq}: {why}")
        m.stop_reasons.add(call.reason)
        actions.append(lambda c=call: self.runtime.submit_operator(c))
