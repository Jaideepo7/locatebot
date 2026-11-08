"""The seam between `MissionService` (on the Pi) and whatever actually runs the mission.

`MissionService` talks to the robot only through a `RobotLink`: `send(msg: dict) -> bool` for the
two frames it may send (`MISSION_REQUEST`, `MISSION_CANCEL`), a `connected` property, and a
`listener` it registers itself as. The link calls back `listener.on_robot_frame(text)` with JSON
text frames (`MISSION_ACK`, `MISSION_STATUS`, `ERROR`, see `protocol.py`), and
`on_robot_connected()` / `on_robot_disconnected()` when reachability changes.
Links may also have `start()` / `stop()`; `RemoteService` calls them when present.

`FakeRobotLink` stands in for the Orin in tests and desk demos. It plays a scripted mission on a
timer (adapted from chanabyte's `ScriptedSink` + `JetsonAgent` on `feat/photon-dispatch`): it
answers a request with an ACK, rejects expired, future-dated, duplicate and concurrent missions,
confirms a cancel after a short delay, stops by itself when the link drops and re-reports its real
state on reconnect. It never invents progress the script does not contain. R3 adds `ZmqRobotLink`.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass
from typing import Callable, Deque, List, Optional, Protocol, Sequence, Union

from . import protocol as p
from .protocol import MissionState as S

log = logging.getLogger(__name__)


class RobotListener(Protocol):
    def on_robot_frame(self, frame: str) -> None: ...
    def on_robot_connected(self) -> None: ...
    def on_robot_disconnected(self) -> None: ...


class RobotLink(Protocol):
    listener: Optional[RobotListener]

    @property
    def connected(self) -> bool: ...

    def send(self, msg: dict) -> bool:
        """Queue one frame for the robot. False when it cannot be sent (offline). Must not block."""


@dataclass(frozen=True)
class Step:
    after_s: float  # seconds after the mission started
    state: S
    message: str = ""  # may contain {target}
    reason: Optional[str] = None


Script = Union[Sequence[Step], str]  # a string = reject every request with that reason

SCRIPTS = {
    "happy": [
        Step(1, S.SEARCHING, "Searching for {target}"),
        Step(3, S.TARGET_DETECTED, "Candidate {target} in view"),
        Step(4, S.APPROACHING, "Approaching {target}"),
        Step(6, S.COMPLETED, "Arrived near {target}"),
    ],
    "not_found": [
        Step(1, S.SEARCHING, "Searching for {target}"),
        Step(5, S.FAILED, "Searched the area; {target} not seen", "NOT_FOUND"),
    ],
    "stall": [Step(1, S.SEARCHING, "Searching for {target}")],
    "reject": "BUSY",
}


@dataclass
class _Active:
    mission_id: str
    target: str
    expires_at: float
    started_at: float
    pending: List[Step]
    state: S = S.ACCEPTED


class FakeRobotLink:
    """A scripted in-process robot. Call `poll()` regularly (or `start()` a thread that does).

    `received` records every frame the service sent; `drop()` / `connect()` simulate the link.
    `silent=True` accepts frames but never answers (tests the ACK timeout).
    """

    def __init__(self, script: Script = "happy", clock: Callable[[], float] = time.time, *,
                 online: bool = True, cancel_delay_s: float = 0.5, max_skew_s: float = 30.0,
                 silent: bool = False, seen_capacity: int = 1000):
        self.listener: Optional[RobotListener] = None
        self.received: List[dict] = []
        self.silent = silent
        self._script = SCRIPTS[script] if isinstance(script, str) and script in SCRIPTS else script
        self._clock = clock  # wall clock: missions carry absolute UTC timestamps
        self._cancel_delay_s, self._max_skew_s = cancel_delay_s, max_skew_s
        self._seen: "OrderedDict[str, dict]" = OrderedDict()
        self._seen_capacity = seen_capacity
        self._lock = threading.RLock()
        self._inbox: Deque[dict] = deque()
        self._outbox: Deque[dict] = deque()
        self._active: Optional[_Active] = None
        self._last_status: Optional[dict] = None
        self._connected = online
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # --- RobotLink --------------------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def active_mission_id(self) -> Optional[str]:
        with self._lock:
            return self._active.mission_id if self._active else None

    def send(self, msg: dict) -> bool:
        with self._lock:
            if not self._connected:
                return False
            self.received.append(dict(msg))
            if not self.silent:
                self._inbox.append(dict(msg))
            return True

    def start(self, period_s: float = 0.1) -> None:
        if self._thread is not None:
            return
        self._stop.clear()

        def loop():
            while not self._stop.wait(period_s):
                try:
                    self.poll()
                except Exception:
                    log.exception("fake robot poll failed")

        self._thread = threading.Thread(target=loop, name="fake-robot", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    # --- simulated link ---------------------------------------------------------------------------
    def drop(self) -> None:
        """The link goes down: frames in flight are lost and the robot stops by itself (its local
        safety policy); it reports that stop once the link is back."""
        with self._lock:
            if not self._connected:
                return
            self._connected = False
            self._inbox.clear()
            self._outbox.clear()
            if self._active is not None:
                log.warning("fake robot: link lost during %s, stopping", self._active.mission_id)
                self._report(S.CANCELLED, "Stopped: lost contact with the server", "LINK_LOST", queue=False)
        if self.listener:
            self.listener.on_robot_disconnected()

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._connected = True
            if self._last_status:
                self._outbox.append(self._last_status)  # tell the service the real current state
        if self.listener:
            self.listener.on_robot_connected()
        self._flush()

    # --- the robot --------------------------------------------------------------------------------
    def poll(self) -> None:
        """Handle received frames, play due script steps, deliver the answers."""
        with self._lock:
            while self._inbox:
                self._handle(self._inbox.popleft())
            m = self._active
            if m is not None:
                now = self._clock()
                if now >= m.expires_at:
                    self._report(S.FAILED, "Mission expired", "EXPIRED")
                else:
                    elapsed = now - m.started_at
                    due = [s for s in m.pending if s.after_s <= elapsed]
                    m.pending = [s for s in m.pending if s.after_s > elapsed]
                    for s in due:
                        self._report(s.state, s.message.format(target=m.target), s.reason)
                        if self._active is None:
                            break
        self._flush()

    def _handle(self, msg: dict) -> None:
        kind = msg.get("type")
        mid = msg.get("mission_id", "")
        if kind == "MISSION_CANCEL":
            m = self._active
            if m is None or m.mission_id != mid:
                return
            at = self._clock() - m.started_at + self._cancel_delay_s
            m.pending = [Step(at, S.CANCELLED, "Stopped on request")]
            return
        if kind != "MISSION_REQUEST":
            self._outbox.append({"type": "ERROR", "reason": "UNEXPECTED_TYPE", "message": str(kind)[:50]})
            return
        if mid in self._seen:  # replay: answer the same way, never start twice
            self._outbox.append(self._seen[mid])
            return
        try:
            p.validate(msg)
            issued = p.parse_ts(msg["issued_at"]).timestamp()
            expires = p.parse_ts(msg["expires_at"]).timestamp()
        except p.ProtocolError as e:
            self._outbox.append({"type": "ERROR", "reason": "INVALID_FRAME", "message": str(e)[:p.MAX_MESSAGE_LEN]})
            return
        now = self._clock()
        reason = None
        if expires <= issued:
            reason = "INVALID_EXPIRY"
        elif issued - now > self._max_skew_s:
            reason = "CLOCK_SKEW"
        elif now >= expires:
            reason = "EXPIRED"
        elif self._active is not None:
            reason = "BUSY"
        elif isinstance(self._script, str):
            reason = self._script
        ack = {"type": "MISSION_ACK", "mission_id": mid, "status": "REJECTED" if reason else "ACCEPTED"}
        if reason:
            ack["reason"] = reason
        self._seen[mid] = ack
        while len(self._seen) > self._seen_capacity:
            self._seen.popitem(last=False)
        self._outbox.append(ack)
        if not reason:
            self._active = _Active(mid, msg["target_description"], expires, now,
                                   sorted(self._script, key=lambda s: s.after_s))

    def _report(self, state: S, message: str, reason: Optional[str], queue: bool = True) -> None:
        m = self._active
        if m is None or (state != m.state and not p.can_transition(m.state, state)):
            return
        m.state = state
        frame = {"type": "MISSION_STATUS", "mission_id": m.mission_id, "status": state.value}
        if message:
            frame["message"] = message[:p.MAX_MESSAGE_LEN]
        if reason:
            frame["reason"] = reason
        if state in p.TERMINAL:
            self._active = None
        self._last_status = frame
        if queue:
            self._outbox.append(frame)

    def _flush(self) -> None:
        while True:
            with self._lock:
                if not self._connected or not self._outbox:
                    return
                frame = self._outbox.popleft()
            if self.listener:  # outside our lock: the listener takes the store lock
                self.listener.on_robot_frame(json.dumps(frame, separators=(",", ":")))
