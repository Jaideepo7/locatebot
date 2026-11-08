"""The hand-off point between mission dispatch and the onboard robot stack.

`MissionSink` is what the perception/mission lanes implement later: `start` passes the target
description to the LocateAnything pipeline and enables the planner, which still goes through the
safety supervisor. The sink reports what is *actually* happening through `report`; the agent never
invents progress.

`ScriptedSink` is a stand-in robot for tests and desk demos: it plays a fixed list of statuses on a
timer. Call `poll()` regularly (the WebSocket client does it on every loop tick).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Sequence

from .protocol import MissionState as S

# report(state, message="", reason=None)
Reporter = Callable[..., None]


class MissionRejected(Exception):
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class MissionSink(Protocol):
    def start(self, mission_id: str, target_description: str, report: Reporter) -> None:
        """Begin searching. Return quickly; report progress later. Raise MissionRejected to refuse."""

    def cancel(self, mission_id: str) -> None:
        """Stop the mission. Confirm with report(MissionState.CANCELLED) once actually stopped."""

    def on_link_lost(self, mission_id: str) -> None:
        """The server connection dropped mid-mission. Apply the local safety policy (default: stop)."""


@dataclass(frozen=True)
class Step:
    after_s: float  # seconds after start
    state: S
    message: str = ""  # may contain {target}
    reason: Optional[str] = None


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


class ScriptedSink:
    def __init__(self, script: Sequence[Step] | str, clock: Callable[[], float] = time.monotonic,
                 cancel_delay_s: float = 0.5):
        self._script = script
        self._clock = clock
        self._cancel_delay_s = cancel_delay_s
        self._lock = threading.Lock()
        self._mission: Optional[str] = None
        self._target = ""
        self._report: Optional[Reporter] = None
        self._pending: list[Step] = []
        self._t0 = 0.0

    def start(self, mission_id: str, target_description: str, report: Reporter) -> None:
        if isinstance(self._script, str):
            raise MissionRejected(self._script)
        with self._lock:
            self._mission, self._target, self._report = mission_id, target_description, report
            self._t0 = self._clock()
            self._pending = sorted(self._script, key=lambda s: s.after_s)

    def cancel(self, mission_id: str) -> None:
        with self._lock:
            if mission_id != self._mission:
                return
            at = self._clock() - self._t0 + self._cancel_delay_s
            self._pending = [Step(at, S.CANCELLED, "Stopped on request")]

    def on_link_lost(self, mission_id: str) -> None:
        with self._lock:
            if mission_id != self._mission:
                return
            report, self._pending, self._mission = self._report, [], None
        report(S.CANCELLED, "Stopped: lost contact with the server", "LINK_LOST")

    def poll(self) -> None:
        with self._lock:
            if self._mission is None:
                return
            elapsed = self._clock() - self._t0
            due = [s for s in self._pending if s.after_s <= elapsed]
            self._pending = [s for s in self._pending if s.after_s > elapsed]
            report = self._report
            if any(s.state in (S.COMPLETED, S.FAILED, S.CANCELLED) for s in due):
                self._mission, self._pending = None, []
        for s in due:
            report(s.state, s.message.format(target=self._target), s.reason)
