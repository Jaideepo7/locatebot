"""`ZmqRobotLink`: the Pi's `RobotLink` to the Orin over the existing ZeroMQ link (R3).

* `send(dict)` turns `MissionService`'s `MISSION_REQUEST` / `MISSION_CANCEL` frames into
  `MissionRequest` / `MissionCancel` link messages (`locatebot/link/messages.py`; mission times
  become Unix wall-clock seconds) and queues them for the Pi's PUB socket (port 5555).
* A thread subscribes to the Orin's PUB (5556) for `mission_status` and `orin_status` (never
  newest-only) and converts statuses back into the frames `MissionService.on_robot_frame` expects:
  the first status of a mission becomes `MISSION_ACK` (`ACCEPTED`, or `REJECTED` when it is a
  `FAILED` with one of the bridge's rejection reasons `BUSY`/`EXPIRED`/`CLOCK_SKEW`/
  `INVALID_EXPIRY`); later ones are `MISSION_STATUS`.
* `connected` = an `orin_status` heard within `connected_timeout_s` (3 s); changes call
  `listener.on_robot_connected()` / `on_robot_disconnected()`.

Sockets are not thread-safe. Standalone, the link owns its PUB socket and only its thread uses
it. In the full car the Pi process already owns the 5555 PUB (T1.3): pass that `publisher=` and
call `flush_outgoing(publisher)` from the thread that owns it; the link's thread then never
touches the publisher. Nothing here can drive the car: only mission frames are carried.
"""
from __future__ import annotations

import json
import logging
import re
import threading
import time
from collections import OrderedDict, deque
from typing import Callable, Deque, Optional

from . import protocol as p

log = logging.getLogger(__name__)

REJECT_REASONS = frozenset({"BUSY", "EXPIRED", "CLOCK_SKEW", "INVALID_EXPIRY"})
_REASON_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,39}$")


def to_link_message(msg: dict):
    """MissionService frame (dict) -> MissionRequest / MissionCancel. Raises ValueError."""
    from locatebot.link.messages import MissionCancel, MissionRequest
    p.validate(msg)
    if msg["type"] == "MISSION_REQUEST":
        return MissionRequest(msg["mission_id"], msg["target_description"],
                              p.parse_ts(msg["issued_at"]).timestamp(), p.parse_ts(msg["expires_at"]).timestamp())
    if msg["type"] == "MISSION_CANCEL":
        return MissionCancel(msg["mission_id"], msg.get("reason", ""))
    raise ValueError(f"{msg['type']} is not sent to the robot")


class ZmqRobotLink:
    def __init__(self, orin_endpoint: str, *, pub_endpoint: Optional[str] = None, publisher=None,
                 clock: Callable[[], float] = time.monotonic, connected_timeout_s: float = 3.0,
                 poll_s: float = 0.05, context=None, seen_capacity: int = 1000):
        from locatebot.link.messages import PI_PUB_PORT
        from locatebot.link.transport import Publisher, bind_endpoint
        self.listener = None
        self.orin_endpoint = orin_endpoint
        self._clock, self._timeout_s, self._poll_s = clock, connected_timeout_s, poll_s
        self._context = context
        self._external_pub = publisher is not None
        # Standalone: created here, then used only by the link thread (a thread start is a full
        # memory barrier, which is what ZeroMQ needs to move a socket between threads).
        self._pub = publisher if publisher is not None else \
            Publisher(pub_endpoint or bind_endpoint(PI_PUB_PORT), context=context)
        self.pub_endpoint = getattr(self._pub, "endpoint", None)
        self._lock = threading.Lock()
        self._out: Deque = deque(maxlen=1000)
        self._seen: "OrderedDict[str, bool]" = OrderedDict()  # mission ids with any status
        self._seen_capacity = seen_capacity
        self._last_heard: Optional[float] = None
        self._connected = False
        self._boot_id: Optional[str] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.stats = {"sent": 0, "send_errors": 0, "statuses": 0, "orin_status": 0, "bad_status": 0,
                      "orin_restarts": 0}

    # --- RobotLink ------------------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._connected

    def send(self, msg: dict) -> bool:
        """Queue a mission frame for the Orin. False when the Orin is not heard or the frame is bad."""
        if not self._connected:
            return False
        try:
            link_msg = to_link_message(msg)
            from locatebot.link.messages import encode
            encode(link_msg)  # shape check now, so a bad frame fails here and not in the thread
        except (ValueError, TypeError) as e:
            log.warning("cannot send %s to the Orin: %s", msg.get("type"), e)
            self.stats["send_errors"] += 1
            return False
        with self._lock:
            self._out.append(link_msg)
        return True

    def flush_outgoing(self, publisher=None) -> int:
        """Send queued mission messages on ``publisher`` (default: our own). Call from the thread
        that owns that publisher."""
        pub = publisher or self._pub
        n = 0
        while True:
            with self._lock:
                if not self._out:
                    return n
                msg = self._out.popleft()
            if pub.send(msg):
                n += 1
                self.stats["sent"] += 1
            else:
                self.stats["send_errors"] += 1
                log.warning("Pi PUB dropped %s", type(msg).__name__)

    # --- thread -----------------------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="zmq-robot-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        if not self._external_pub:
            self._pub.close()

    def _run(self) -> None:
        from locatebot.link.transport import Subscriber
        try:
            sub = Subscriber(self.orin_endpoint, ["mission_status", "orin_status"], context=self._context)
        except Exception:
            log.exception("cannot subscribe to the Orin at %s", self.orin_endpoint)
            return
        with sub:
            while not self._stop.is_set():
                try:
                    if not self._external_pub:
                        self.flush_outgoing()
                    for m in sub.poll(self._poll_s):
                        self._on_link_message(m)
                    self._check_timeout()
                except Exception:
                    log.exception("robot link loop error")
                    self._stop.wait(self._poll_s)

    # --- inbound ----------------------------------------------------------------------------------
    def _on_link_message(self, m) -> None:
        from locatebot.link.messages import MissionStatus, OrinStatus
        if isinstance(m, OrinStatus):
            self.stats["orin_status"] += 1
            if self._boot_id is not None and m.boot_id != self._boot_id:
                self.stats["orin_restarts"] += 1
                log.warning("Orin restarted (boot id changed)")
            self._boot_id = m.boot_id
            self._last_heard = self._clock()
            if not self._connected:
                self._connected = True
                log.info("Orin link up")
                if self.listener:
                    self.listener.on_robot_connected()
        elif isinstance(m, MissionStatus):
            self.stats["statuses"] += 1
            frame = self._frame(m)
            if frame is None:
                self.stats["bad_status"] += 1
                return
            if self.listener:
                self.listener.on_robot_frame(json.dumps(frame, separators=(",", ":")))

    def _frame(self, m) -> Optional[dict]:
        reason = m.reason if m.reason and _REASON_RE.match(m.reason) else None
        if m.reason and reason is None:
            log.warning("dropping non-code reason %r from the Orin", m.reason[:40])
        first = m.mission_id not in self._seen
        self._seen[m.mission_id] = True
        self._seen.move_to_end(m.mission_id)
        while len(self._seen) > self._seen_capacity:
            self._seen.popitem(last=False)
        if first and m.state == "ACCEPTED":
            frame = {"type": "MISSION_ACK", "mission_id": m.mission_id, "status": "ACCEPTED"}
        elif first and m.state == "FAILED" and reason in REJECT_REASONS:
            frame = {"type": "MISSION_ACK", "mission_id": m.mission_id, "status": "REJECTED", "reason": reason}
        else:
            frame = {"type": "MISSION_STATUS", "mission_id": m.mission_id, "status": m.state}
            if m.message:
                frame["message"] = m.message[:p.MAX_MESSAGE_LEN]
            if reason:
                frame["reason"] = reason
        try:
            p.validate(frame)
        except p.ProtocolError as e:
            log.warning("dropping mission status from the Orin: %s", e)
            return None
        return frame

    def _check_timeout(self) -> None:
        if self._connected and (self._last_heard is None or self._clock() - self._last_heard > self._timeout_s):
            self._connected = False
            log.warning("Orin link down (no orin_status for %.0f s)", self._timeout_s)
            if self.listener:
                self.listener.on_robot_disconnected()
