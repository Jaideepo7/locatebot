"""Jetson-side mission agent: a transport-free state machine.

The WebSocket client feeds it frames (`on_frame`), connection events (`on_connected`,
`on_disconnected`) and a periodic `tick()`. It sends frames through the `send` callable it was
given. It owns no motion: it validates requests, rejects expired, future-dated, duplicate and
concurrent missions, hands the target text to the `MissionSink`, and relays the sink's reports.

Locking: the agent never calls the sink while holding its own lock, so a sink may call `report`
from any thread at any time.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from functools import partial
from typing import Callable, Optional

from . import protocol as p
from .protocol import MissionState as S
from .sink import MissionRejected, MissionSink

log = logging.getLogger(__name__)


@dataclass
class _Mission:
    mission_id: str
    target: str
    expires_at: float
    state: S = S.ACCEPTED
    starting: bool = True
    buffered: list = field(default_factory=list)
    link_lost_notified: bool = False


class JetsonAgent:
    def __init__(self, sink: MissionSink, send: Callable[[str], None], clock: Callable[[], float] = time.time,
                 max_skew_s: float = 30.0, seen_capacity: int = 1000):
        self._sink = sink
        self._send_fn = send
        self._clock = clock  # wall clock (epoch seconds): missions carry absolute UTC timestamps
        self._max_skew_s = max_skew_s
        self._seen_capacity = seen_capacity
        self._seen: "OrderedDict[str, dict]" = OrderedDict()  # mission_id -> the ACK we sent
        self._lock = threading.RLock()
        self._active: Optional[_Mission] = None
        self._last_status: Optional[dict] = None
        self._connected = False

    @property
    def active_mission_id(self) -> Optional[str]:
        with self._lock:
            return self._active.mission_id if self._active else None

    # --- connection events --------------------------------------------------------------------
    def on_connected(self) -> None:
        with self._lock:
            self._connected = True
            if self._active:
                self._active.link_lost_notified = False
            if self._last_status:
                self._send(self._last_status)  # tell the backend the real current state

    def on_disconnected(self) -> None:
        with self._lock:
            self._connected = False
            m = self._active
            if m is None or m.link_lost_notified:
                return
            m.link_lost_notified = True
        log.warning("server link lost during mission %s; applying local safety policy", m.mission_id)
        self._call_sink("on_link_lost", m.mission_id)

    # --- inbound frames -----------------------------------------------------------------------
    def on_frame(self, frame) -> None:
        try:
            msg = p.decode(frame)
        except p.ProtocolError as e:
            log.warning("rejected frame: %s", e)
            with self._lock:
                self._send({"type": "ERROR", "reason": "INVALID_FRAME", "message": str(e)[:p.MAX_MESSAGE_LEN]})
            return
        if msg["type"] == "MISSION_REQUEST":
            self._on_request(msg)
        elif msg["type"] == "MISSION_CANCEL":
            self._on_cancel(msg["mission_id"])
        else:
            with self._lock:
                self._send({"type": "ERROR", "reason": "UNEXPECTED_TYPE", "message": msg["type"]})

    def _on_request(self, msg: dict) -> None:
        mid = msg["mission_id"]
        now = self._clock()
        with self._lock:
            if mid in self._seen:  # replay or resend: answer the same way, never start twice
                self._send(self._seen[mid])
                return
            issued = p.parse_ts(msg["issued_at"]).timestamp()
            expires = p.parse_ts(msg["expires_at"]).timestamp()
            reason = None
            if expires <= issued:
                reason = "INVALID_EXPIRY"
            elif issued - now > self._max_skew_s:
                reason = "CLOCK_SKEW"
            elif now >= expires:
                reason = "EXPIRED"
            elif self._active is not None:
                reason = "BUSY"
            if reason:
                self._ack(mid, reason)
                return
            m = self._active = _Mission(mid, msg["target_description"], expires)
        try:
            self._sink.start(mid, m.target, partial(self.report, mid))
            reason = None
        except MissionRejected as e:
            reason = e.reason if isinstance(e.reason, str) and p._REASON_RE.match(e.reason) else "REJECTED_BY_ROBOT"
        except Exception:
            log.exception("sink.start failed")
            reason = "ROBOT_ERROR"
        with self._lock:
            if reason:
                self._active = None
                self._ack(mid, reason)
                return
            m.starting = False
            self._ack(mid, None)
            buffered, m.buffered = m.buffered, []
        for args in buffered:  # reports the sink made from inside start(), after the ACK
            self.report(mid, *args)

    def _ack(self, mid: str, reason: Optional[str]) -> None:
        ack = {"type": "MISSION_ACK", "mission_id": mid, "status": "REJECTED" if reason else "ACCEPTED"}
        if reason:
            ack["reason"] = reason
        self._seen[mid] = ack
        while len(self._seen) > self._seen_capacity:
            self._seen.popitem(last=False)
        self._send(ack)

    def _on_cancel(self, mid: str) -> None:
        with self._lock:
            if self._active is None or self._active.mission_id != mid:
                log.info("cancel for inactive mission %s ignored", mid)
                return
        self._call_sink("cancel", mid)  # CANCELLED is sent when the sink confirms it stopped

    # --- sink reports -------------------------------------------------------------------------
    def report(self, mission_id: str, state, message: str = "", reason: Optional[str] = None) -> None:
        state = S(state)  # ValueError for anything that is not one of the eight states
        with self._lock:
            m = self._active
            if m is None or m.mission_id != mission_id:
                log.info("dropping %s for inactive mission %s", state.value, mission_id)
                return
            if m.starting:
                m.buffered.append((state, message, reason))
                return
            if state != m.state and not p.can_transition(m.state, state):
                log.warning("dropping illegal transition %s -> %s", m.state.value, state.value)
                return
            m.state = state
            frame = {"type": "MISSION_STATUS", "mission_id": mission_id, "status": state.value}
            if message:
                frame["message"] = str(message)[:p.MAX_MESSAGE_LEN]
            if reason and isinstance(reason, str) and p._REASON_RE.match(reason):
                frame["reason"] = reason
            if state in p.TERMINAL:
                self._active = None
            self._last_status = frame
            self._send(frame)

    def tick(self) -> None:
        with self._lock:
            m = self._active
            if m is None or m.starting or self._clock() < m.expires_at:
                return
            self._active = None
            frame = {"type": "MISSION_STATUS", "mission_id": m.mission_id, "status": "FAILED",
                     "message": "Mission expired", "reason": "EXPIRED"}
            self._last_status = frame
            self._send(frame)
        self._call_sink("cancel", m.mission_id)

    # --- helpers ------------------------------------------------------------------------------
    def _send(self, frame: dict) -> None:
        if not self._connected:
            return
        try:
            self._send_fn(json.dumps(frame, separators=(",", ":")))
        except Exception as e:  # a dead socket must never break mission bookkeeping
            log.warning("send failed: %s", e)

    def _call_sink(self, method: str, mission_id: str) -> None:
        try:
            getattr(self._sink, method)(mission_id)
        except Exception:
            log.exception("sink.%s failed", method)
