"""ZeroMQ PUB/SUB transport for the Pi <-> Orin link, over the wire format in ``messages``.

Sockets (``docs/link/README.md``)
- Pi PUB binds ``tcp://*:5555`` (``scan``, ``frame``, ``pi_status``, ``range``,
  ``mission_request``, ``mission_cancel``); the Orin SUB connects.
- Orin PUB binds ``tcp://*:5556`` (``drive``, ``estop``, ``orin_status``, ``mission_status``);
  the Pi SUB connects. Mission topics must never be ``newest_only``.
Addresses come from the caller (config/CLI) or the ``LOCATEBOT_PI_HOST`` /
``LOCATEBOT_ORIN_HOST`` environment variables. No real address is hard-coded.

Newest-only without CONFLATE
ZeroMQ's ``ZMQ_CONFLATE`` keeps only the last *frame*, so it corrupts our three-frame
messages. ``Subscriber.poll`` instead drains every message currently queued (multipart
messages are always delivered whole) and, for topics in ``newest_only``, keeps just the last
one per topic. A small ``RCVHWM`` bounds the local queue. ``recv`` is plain FIFO.

Sending never blocks: ``send`` uses ``NOBLOCK``. A PUB socket silently drops messages for a
subscriber whose queue is full (``SNDHWM``), so delivery is best effort. Note that a full
queue drops the *newest* messages, not the oldest; newest-only works because senders publish
continuously, so the next drain sees fresh messages again. ``linger`` is short
(PUB) or zero (SUB) so ``close`` never hangs on unsent data.

Malformed input never raises out of ``recv``/``poll``: any message that fails ``decode``
(``ProtocolError``, or another ``ValueError``/``TypeError``) or whose topic is not exactly a
subscribed one is dropped and counted in ``malformed``.

Threads: ZeroMQ sockets are not thread-safe. Each ``Publisher``/``Subscriber``/``DriveReceiver``
must be created, used and closed from one thread. Contexts are thread-safe and shared.

Dead links: ZMTP heartbeats (``heartbeat_s``) drop a connection whose peer stopped answering,
which discards that connection's queued messages; ZeroMQ then reconnects and re-subscribes.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass

import zmq

from locatebot.link.messages import ORIN_PUB_PORT, PI_PUB_PORT, DriveCmd, EStopCmd, decode, encode

PI_HOST_ENV = "LOCATEBOT_PI_HOST"
ORIN_HOST_ENV = "LOCATEBOT_ORIN_HOST"


# --- endpoints ---------------------------------------------------------------

def bind_endpoint(port: int, interface: str = "*") -> str:
    """Endpoint a publisher binds, e.g. ``bind_endpoint(PI_PUB_PORT) == "tcp://*:5555"``."""
    return f"tcp://{interface}:{port}"


def _peer(env: str, port: int, host: str | None) -> str:
    host = host or os.environ.get(env)
    if not host:
        raise ValueError(f"no peer address: pass a host or set {env}")
    return f"tcp://{host}:{port}"


def pi_pub_endpoint(host: str | None = None) -> str:
    """Endpoint the Orin connects to for scans/frames/status (host or ``$LOCATEBOT_PI_HOST``)."""
    return _peer(PI_HOST_ENV, PI_PUB_PORT, host)


def orin_pub_endpoint(host: str | None = None) -> str:
    """Endpoint the Pi connects to for drive/estop/status (host or ``$LOCATEBOT_ORIN_HOST``)."""
    return _peer(ORIN_HOST_ENV, ORIN_PUB_PORT, host)


def _timeout_ms(timeout_s: float | None) -> int | None:
    if timeout_s is None:
        return None  # wait forever
    return max(0, math.ceil(timeout_s * 1000.0))


def _common_opts(sock: zmq.Socket, linger_s: float, heartbeat_s: float) -> None:
    sock.setsockopt(zmq.LINGER, max(0, int(linger_s * 1000)))
    if heartbeat_s > 0:
        ivl = int(heartbeat_s * 1000)
        sock.setsockopt(zmq.HEARTBEAT_IVL, ivl)
        sock.setsockopt(zmq.HEARTBEAT_TIMEOUT, 3 * ivl)
        sock.setsockopt(zmq.HEARTBEAT_TTL, 3 * ivl)


class _Closable:
    _sock: zmq.Socket | None

    @property
    def closed(self) -> bool:
        return self._sock is None

    def _live(self) -> zmq.Socket:
        if self._sock is None:
            raise RuntimeError(f"{type(self).__name__} is closed")
        return self._sock

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# --- publisher ---------------------------------------------------------------

class Publisher(_Closable):
    """PUB socket bound to ``endpoint``. ``send`` never blocks; full queues drop messages."""

    def __init__(self, endpoint: str, *, sndhwm: int = 100, linger_s: float = 0.2,
                 heartbeat_s: float = 1.0, context: zmq.Context | None = None) -> None:
        sock = (context or zmq.Context.instance()).socket(zmq.PUB)
        try:
            sock.setsockopt(zmq.SNDHWM, sndhwm)
            _common_opts(sock, linger_s, heartbeat_s)
            sock.bind(endpoint)
            self.endpoint: str = sock.getsockopt_string(zmq.LAST_ENDPOINT)
        except Exception:
            sock.close(linger=0)
            raise
        self._sock = sock
        self.sent = 0
        self.dropped = 0  # only EAGAIN; PUB drops at SNDHWM silently, so this is a lower bound

    def send(self, msg) -> bool:
        """Encode and publish ``msg``. Returns False if it was dropped locally.

        Encoding errors (``ProtocolError``/``TypeError``) are the caller's bug and are raised.
        """
        parts = encode(msg)
        try:
            self._live().send_multipart(parts, flags=zmq.NOBLOCK, copy=False)
        except zmq.Again:
            self.dropped += 1
            return False
        self.sent += 1
        return True


# --- subscriber --------------------------------------------------------------

class Subscriber(_Closable):
    """SUB socket connected to ``endpoint`` for ``topics`` (None = all topics).

    ``recv`` returns the next message in arrival order. ``poll`` drains the queue and applies
    newest-only to topics in ``newest_only``.
    """

    def __init__(self, endpoint: str, topics, *, newest_only=(), rcvhwm: int = 100,
                 max_drain: int = 10_000, heartbeat_s: float = 1.0,
                 context: zmq.Context | None = None) -> None:
        self.topics = None if topics is None else frozenset(t.encode() if isinstance(t, str) else t
                                                            for t in topics)
        self.newest_only = frozenset(t.encode() if isinstance(t, str) else t for t in newest_only)
        if self.topics is not None and not self.newest_only <= self.topics:
            raise ValueError("newest_only topics must be subscribed")
        self.max_drain = max_drain
        sock = (context or zmq.Context.instance()).socket(zmq.SUB)
        try:
            sock.setsockopt(zmq.RCVHWM, rcvhwm)
            _common_opts(sock, 0.0, heartbeat_s)
            sock.connect(endpoint)
            for t in (self.topics if self.topics is not None else (b"",)):
                sock.setsockopt(zmq.SUBSCRIBE, t)
        except Exception:
            sock.close(linger=0)
            raise
        self.endpoint = endpoint
        self._sock = sock
        self.received = 0
        self.malformed = 0
        self.superseded = 0  # newest-only messages replaced by a newer one in the same drain

    def _decode(self, parts):
        if self.topics is not None and (not parts or bytes(parts[0]) not in self.topics):
            self.malformed += 1  # e.g. b"drivex" prefix-matching the b"drive" subscription
            return None
        try:
            msg = decode(parts)
        except (ValueError, TypeError):  # ProtocolError is a ValueError
            self.malformed += 1
            return None
        self.received += 1
        return msg

    def _wait(self, timeout_s: float | None) -> bool:
        return bool(self._live().poll(_timeout_ms(timeout_s), zmq.POLLIN))

    def _recv_raw(self):
        try:
            return self._live().recv_multipart(zmq.NOBLOCK)
        except zmq.Again:
            return None

    def recv(self, timeout_s: float | None = 0.0):
        """Next valid message in arrival order, or None after ``timeout_s`` (None = forever)."""
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while True:
            parts = self._recv_raw()
            if parts is not None:
                msg = self._decode(parts)
                if msg is not None:
                    return msg
                continue  # malformed: try the next one
            left = None if deadline is None else deadline - time.monotonic()
            if left is not None and left <= 0 or not self._wait(left):
                return None

    def poll(self, timeout_s: float | None = 0.0) -> list:
        """Wait up to ``timeout_s`` for traffic, then drain everything queued.

        Returns valid messages in arrival order; for a ``newest_only`` topic only its last
        message of this drain is kept, at that message's arrival position.
        """
        self._wait(timeout_s)
        raw = []
        for _ in range(self.max_drain):
            parts = self._recv_raw()
            if parts is None:
                break
            raw.append(parts)
        msgs = [(bytes(p[0]) if p else b"", m) for p in raw if (m := self._decode(p)) is not None]
        if not self.newest_only:
            return [m for _, m in msgs]
        last = {t: i for i, (t, _) in enumerate(msgs) if t in self.newest_only}
        out = []
        for i, (t, m) in enumerate(msgs):
            if t in self.newest_only and last[t] != i:
                self.superseded += 1
            else:
                out.append(m)
        return out


# --- Pi-side drive receiver --------------------------------------------------

@dataclass(frozen=True)
class DriveUpdate:
    drive: DriveCmd | None             # newest new drive command of this poll, or None
    estops: tuple[EStopCmd, ...] = ()  # every e-stop message received, in arrival order


class DriveReceiver(_Closable):
    """Pi side of the Orin's ``drive``/``estop`` topics.

    Two SUB sockets on the same endpoint: ``drive`` with a small queue, and ``estop`` with its
    own large queue, so a drive backlog can never crowd an e-stop out (PUB filters per
    subscriber, so the e-stop pipe only carries e-stops).

    - Drive: drops ``seq <= last_seq`` (replay/reorder) and returns only the highest new seq
      per poll. After an Orin restart its seq starts low again and every command is dropped
      (the safety supervisor's command timeout stops the car) until ``reset()`` is called,
      which the Pi should do when ``orin_status`` shows the Orin restarted.
    - E-stop: every message is delivered, never conflated or seq-filtered. ``engage=False`` is
      passed through as data only: the Pi controller must ignore network releases and release
      only on a local action.
    - Stamp each returned command with the Pi's clock on receipt before the safety supervisor.
    """

    def __init__(self, endpoint: str, *, drive_hwm: int = 4, estop_hwm: int = 1000,
                 heartbeat_s: float = 1.0, context: zmq.Context | None = None) -> None:
        self._drive = Subscriber(endpoint, ["drive"], rcvhwm=drive_hwm, heartbeat_s=heartbeat_s,
                                 context=context)
        try:
            self._estop = Subscriber(endpoint, ["estop"], rcvhwm=estop_hwm,
                                     heartbeat_s=heartbeat_s, context=context)
        except Exception:
            self._drive.close()
            raise
        self._sock = self._drive._sock  # for _Closable.closed
        self._poller = zmq.Poller()
        self._poller.register(self._drive._sock, zmq.POLLIN)
        self._poller.register(self._estop._sock, zmq.POLLIN)
        self.last_seq: int | None = None
        self.dropped_old = 0  # seq <= last_seq
        self.superseded = 0   # newer valid drive arrived in the same poll

    @property
    def malformed(self) -> int:
        return self._drive.malformed + self._estop.malformed

    def reset(self) -> None:
        """Forget ``last_seq`` (call when the Orin restarted and its seq counter began again)."""
        self.last_seq = None

    def poll(self, timeout_s: float | None = 0.0) -> DriveUpdate:
        self._live()
        self._poller.poll(_timeout_ms(timeout_s))
        estops = tuple(m for m in self._estop.poll(0.0) if isinstance(m, EStopCmd))
        best = None
        for m in self._drive.poll(0.0):
            if not isinstance(m, DriveCmd):
                continue
            if self.last_seq is not None and m.seq <= self.last_seq:
                self.dropped_old += 1
            elif best is None or m.seq > best.seq:
                if best is not None:
                    self.superseded += 1
                best = m
            else:
                self.superseded += 1
        if best is not None:
            self.last_seq = best.seq
        return DriveUpdate(best, estops)

    def close(self) -> None:
        if self._sock is not None:
            self._estop.close()
            self._drive.close()
            self._sock = None
