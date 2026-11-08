"""Mission management: turns customer commands into missions, robot reports into state changes,
and both into queued customer replies.

Rules it enforces:
* one active mission per robot; customers see and cancel only their own missions;
* the mission id, issue time and expiry are generated here, never taken from the customer;
* state only changes along protocol.TRANSITIONS, and only because the robot reported it, or because
  a Pi-side timeout expired (FAILED with reason EXPIRED / NO_ACK / LINK_LOST / ROBOT_OFFLINE);
* cancellation is a request: the mission is CANCELLED only when the robot confirms it.
Nothing here can move the robot; the only frames it sends are MISSION_REQUEST and MISSION_CANCEL.
"""
from __future__ import annotations

import logging
import time
import uuid
from datetime import datetime, timezone
from typing import Callable, List, Optional, Tuple

from . import protocol as p
from .protocol import MissionState as S
from . import notifier as text
from .commands import Kind, parse_command
from .messaging import Conversation, InboundMessage
from .store import Mission, Store

log = logging.getLogger(__name__)


class RobotUnavailable(Exception):
    pass


class RobotBusy(Exception):
    def __init__(self, mission: Mission):
        super().__init__(mission.mission_id)
        self.mission = mission


class MissionNotFound(Exception):
    pass


class AlreadyFinished(Exception):
    pass


def _iso(epoch: float) -> str:
    return p.format_ts(datetime.fromtimestamp(epoch, timezone.utc))


class MissionService:
    def __init__(self, store: Store, link, robot_id: str, clock: Callable[[], float] = time.time,
                 ttl_s: float = 600.0, ack_timeout_s: float = 10.0, link_grace_s: float = 15.0,
                 rate_limit: int = 5, rate_window_s: float = 60.0):
        self._store, self._link, self._robot_id, self._clock = store, link, robot_id, clock
        self._ttl_s, self._ack_timeout_s, self._grace_s = ttl_s, ack_timeout_s, link_grace_s
        self._rate_limit, self._rate_window_s = rate_limit, rate_window_s
        self._recover()

    # --- queries ------------------------------------------------------------------------------
    def get(self, mission_id: str) -> Optional[Mission]:
        return self._store.get_mission(mission_id)

    # --- customer messages --------------------------------------------------------------------
    def handle_inbound(self, msg: InboundMessage) -> None:
        """Process one verified inbound message exactly once (keyed by msg.event_id).
        Raises StorageUnavailable if it could not be recorded, so the provider retries."""
        cmd = parse_command(msg.text)
        now = self._clock()
        after: List[Callable[[], None]] = []
        with self._store.lock:
            with self._store.transaction():
                if not self._store.insert_event(msg.event_id, now):
                    log.info("duplicate delivery %s ignored", msg.event_id)
                    return
                for i, reply in enumerate(self._command(cmd, msg, now, after)):
                    self._store.enqueue(f"{msg.event_id}:{i}", msg.conversation, reply, now)
            for action in after:  # only after the commit: never tell the robot about an unsaved mission
                action()

    def _command(self, cmd, msg: InboundMessage, now: float, after: list) -> List[str]:
        conv = msg.conversation
        if cmd.kind == Kind.START:
            return [text.START_TEXT]
        if cmd.kind == Kind.HELP:
            return [text.HELP_TEXT]
        if cmd.kind == Kind.UNKNOWN:
            return [text.UNKNOWN]
        if cmd.kind == Kind.NOT_COMMAND:
            return [text.NOT_COMMAND]
        if cmd.kind == Kind.NOT_TEXT:
            return [text.NOT_TEXT]
        if cmd.kind == Kind.STATUS:
            m = self._store.latest_for(conv.provider, conv.conversation_id, msg.sender_id)
            return [text.describe(m.state, m.target) if m else text.NO_MISSIONS]
        if cmd.kind == Kind.CANCEL:
            m = self._store.latest_for(conv.provider, conv.conversation_id, msg.sender_id)
            if m is None or m.terminal:
                return [text.NO_ACTIVE]
            self._store.update_mission(m.mission_id, cancel_requested=1, updated_at=now)
            if not self._link.connected:
                return [text.CANCEL_UNREACHABLE]
            after.append(lambda: self._link.send({"type": "MISSION_CANCEL", "mission_id": m.mission_id}))
            return [text.CANCELLING]
        # FIND
        if cmd.error:
            return [cmd.error]
        if self._store.count_created_since(conv.provider, conv.conversation_id,
                                           now - self._rate_window_s) >= self._rate_limit:
            return [text.RATE_LIMITED]
        active = self._store.active_mission(self._robot_id)
        if active is not None:
            mine = (active.provider, active.conversation_id, active.sender_id) == \
                   (conv.provider, conv.conversation_id, msg.sender_id)
            if not mine:
                return [text.OTHER_CUSTOMER_BUSY]
            return [text.already_searching(active.target, active.target.casefold() == cmd.description.casefold())]
        if not self._link.connected:
            return [text.UNAVAILABLE]
        m = self._new_mission(cmd.description, now, conv, msg.sender_id)
        after.append(lambda: self._send_request(m))
        return [text.RECEIVED]

    # --- operator API -------------------------------------------------------------------------
    def create_mission(self, target: str, idempotency_key: Optional[str] = None) -> Tuple[Mission, bool]:
        """Raises InvalidDescription, RobotBusy or RobotUnavailable. Returns (mission, created)."""
        target = p.normalize_description(target)
        now = self._clock()
        with self._store.lock:
            with self._store.transaction():
                if idempotency_key:
                    existing = self._store.find_by_idempotency_key(idempotency_key)
                    if existing:
                        return existing, False
                active = self._store.active_mission(self._robot_id)
                if active is not None:
                    raise RobotBusy(active)
                if not self._link.connected:
                    raise RobotUnavailable()
                m = self._new_mission(target, now, None, None, idempotency_key)
            self._send_request(m)
            return self._store.get_mission(m.mission_id), True

    def cancel(self, mission_id: str) -> Mission:
        """Operator cancel. Raises MissionNotFound / AlreadyFinished. Confirmation comes from the robot."""
        with self._store.lock:
            m = self._store.get_mission(mission_id)
            if m is None or m.robot_id != self._robot_id:
                raise MissionNotFound(mission_id)
            if m.terminal:
                raise AlreadyFinished(mission_id)
            self._store.update_mission(mission_id, cancel_requested=1, updated_at=self._clock())
            self._link.send({"type": "MISSION_CANCEL", "mission_id": mission_id})
            return self._store.get_mission(mission_id)

    # --- robot link events --------------------------------------------------------------------
    def on_robot_frame(self, frame) -> None:
        try:
            msg = p.decode(frame)
        except p.ProtocolError as e:
            log.warning("bad frame from robot: %s", e)
            return
        if msg["type"] == "ERROR":
            log.warning("robot reported %s: %s", msg["reason"], msg.get("message", ""))
            return
        if msg["type"] not in ("MISSION_ACK", "MISSION_STATUS"):
            log.warning("robot sent unexpected %s", msg["type"])
            return
        now = self._clock()
        with self._store.lock, self._store.transaction():
            m = self._store.get_mission(msg["mission_id"])
            if m is None or m.robot_id != self._robot_id:
                log.warning("robot reported unknown mission %s", msg["mission_id"])
                return
            if msg["type"] == "MISSION_ACK":
                if m.state != S.PENDING:
                    return  # repeated ACK
                if msg["status"] == "ACCEPTED":
                    self._store.update_mission(m.mission_id, state=S.ACCEPTED, updated_at=now)
                else:
                    self._fail(m, msg.get("reason", "REJECTED"), now, text.rejected_text(msg.get("reason")))
                return
            new, current = S(msg["status"]), m.state
            if current == S.PENDING and new != S.PENDING and (new == S.ACCEPTED or p.can_transition(S.ACCEPTED, new)):
                current = S.ACCEPTED  # the ACK was lost (e.g. reconnect); a later status implies acceptance
            if new == current:
                self._store.update_mission(m.mission_id, state=new, updated_at=now,
                                           last_message=msg.get("message", m.last_message))
                return
            if not p.can_transition(current, new):
                log.warning("ignoring illegal robot transition %s -> %s for %s", current.value, new.value, m.mission_id)
                return
            self._transition(m, current, new, msg.get("reason"), msg.get("message"), now)

    def on_robot_connected(self) -> None:
        now = self._clock()
        with self._store.lock, self._store.transaction():
            for m in self._store.active_missions(self._robot_id):
                if m.link_lost_at is not None:
                    self._store.update_mission(m.mission_id, link_lost_at=None, updated_at=now)
                    self._reply(m, f"{m.mission_id}:reconnected@{now:.3f}", text.RECONNECTED, now)

    def on_robot_disconnected(self) -> None:
        now = self._clock()
        with self._store.lock, self._store.transaction():
            for m in self._store.active_missions(self._robot_id):
                if m.link_lost_at is None:
                    self._store.update_mission(m.mission_id, link_lost_at=now, updated_at=now)
                    self._reply(m, f"{m.mission_id}:linklost@{now:.3f}", text.LINK_LOST, now)

    def tick(self) -> None:
        """Apply Pi-side timeouts. Call about once a second."""
        now = self._clock()
        cancels = []
        with self._store.lock:
            with self._store.transaction():
                for m in self._store.active_missions(self._robot_id):
                    if now >= m.expires_at:
                        self._fail(m, "EXPIRED", now)
                        cancels.append({"type": "MISSION_CANCEL", "mission_id": m.mission_id, "reason": "EXPIRED"})
                    elif m.link_lost_at is not None:
                        if not self._link.connected and now - m.link_lost_at >= self._grace_s:
                            self._fail(m, "LINK_LOST", now)
                    elif m.state == S.PENDING and now - m.issued_at >= self._ack_timeout_s:
                        self._fail(m, "NO_ACK", now)
                        cancels.append({"type": "MISSION_CANCEL", "mission_id": m.mission_id, "reason": "NO_ACK"})
            for frame in cancels:
                self._link.send(frame)

    # --- internals ----------------------------------------------------------------------------
    def _recover(self) -> None:
        """After a restart nothing is connected yet: start the link-loss grace for unfinished missions."""
        now = self._clock()
        with self._store.lock, self._store.transaction():
            for m in self._store.active_missions(self._robot_id):
                if m.link_lost_at is None:
                    self._store.update_mission(m.mission_id, link_lost_at=now)

    def _new_mission(self, target: str, now: float, conv: Optional[Conversation], sender_id: Optional[str],
                     idempotency_key: Optional[str] = None) -> Mission:
        m = Mission(mission_id=str(uuid.uuid4()), robot_id=self._robot_id, target=target, state=S.PENDING,
                    issued_at=now, expires_at=now + self._ttl_s, created_at=now, updated_at=now,
                    provider=conv.provider if conv else None, conversation_id=conv.conversation_id if conv else None,
                    reply_as=conv.reply_as if conv else None, sender_id=sender_id, idempotency_key=idempotency_key)
        self._store.insert_mission(m)
        return m

    def _send_request(self, m: Mission) -> None:
        frame = {"type": "MISSION_REQUEST", "mission_id": m.mission_id, "mission_type": "FIND_OBJECT",
                 "target_description": m.target, "issued_at": _iso(m.issued_at), "expires_at": _iso(m.expires_at)}
        if not self._link.send(frame):
            with self._store.transaction():
                self._fail(self._store.get_mission(m.mission_id), "ROBOT_OFFLINE", self._clock())

    def _transition(self, m: Mission, old: S, new: S, reason: Optional[str], message: Optional[str], now: float):
        changes = {"state": new, "updated_at": now}
        if message:
            changes["last_message"] = message
        if new == S.FAILED or (new == S.CANCELLED and reason):
            changes["fail_reason"] = reason or "ROBOT_REPORTED"
        self._store.update_mission(m.mission_id, **changes)
        reply = text.status_text(old, new, m.target, reason, message)
        if reply:
            self._reply(m, f"{m.mission_id}:{new.value}", reply, now)

    def _fail(self, m: Mission, reason: str, now: float, reply: Optional[str] = None) -> None:
        self._store.update_mission(m.mission_id, state=S.FAILED, fail_reason=reason, updated_at=now)
        self._reply(m, f"{m.mission_id}:FAILED", reply or text.status_text(m.state, S.FAILED, m.target, reason), now)

    def _reply(self, m: Mission, key: str, reply: Optional[str], now: float) -> None:
        conv = m.conversation
        if conv is None or not reply:
            return
        if not self._store.enqueue(key, conv, reply, now):
            # same state reached again (e.g. SEARCHING after losing the target): new, still deterministic key
            self._store.enqueue(f"{key}@{now:.3f}", conv, reply, now)
