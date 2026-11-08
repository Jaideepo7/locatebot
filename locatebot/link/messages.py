"""Wire format between the Raspberry Pi and the Jetson Orin.

Every message is three byte frames, ready for a ZeroMQ multipart send:

    [topic, header, payload]

- ``topic``: the message type as ASCII (``b"scan"``), so SUB sockets can filter by prefix.
- ``header``: UTF-8 JSON object with ``"v"`` (protocol version), ``"type"`` and the fields.
- ``payload``: binary data (scan arrays, JPEG bytes) or ``b""``.

Directions and clocks
- Pi -> Orin: ``Scan``, ``Frame``, ``PiStatus``, ``Range``. All their times are the **Pi's
  monotonic clock**, so scan stamps, range readings and frame capture times line up on the
  Orin (pose lookup for item localization uses them).
- Pi -> Orin: ``MissionRequest``, ``MissionCancel`` (missions from a Pi-side sender). The one
  exception to the clock rule: ``issued_s`` / ``expires_s`` are **Unix wall-clock** seconds
  (``time.time()``) because a mission is born outside the car; the Orin compares them with its
  own wall clock and a skew tolerance, so both machines need NTP/chrony.
- Orin -> Pi: ``DriveCmd``, ``EStopCmd``, ``OrinStatus``, ``MissionStatus``. ``sent_s`` is the
  Orin's clock and is for latency diagnostics only. The Pi must stamp a command with its own
  clock **on receipt** before giving it to the safety supervisor; it never compares Orin times
  with Pi times. ``OrinStatus.boot_id`` changes on every Orin process start: when it changes
  the Pi resets its drive ``seq`` filter (``DriveReceiver.reset``).

Decoding is strict: wrong frame count, version, type, missing fields, bad payload size,
out-of-range drive or range values, an unknown mission state, or a string that is too long,
empty where it must not be, or contains control characters raise ``ProtocolError`` (encode
checks the same rules); the receiver drops the message (for drive
commands the safety supervisor then times out and stops). Unknown extra header fields are
ignored so a newer sender can add fields without breaking an older receiver.

Sockets (``transport.py``): Pi PUB on tcp port 5555 (scan, frame, pi_status, range,
mission_request, mission_cancel), Orin PUB on tcp port 5556 (drive, estop, orin_status,
mission_status). Frame subscribers keep only the newest frame; mission topics must never be
newest-only.
"""
from __future__ import annotations

import json
import math
import unicodedata
from dataclasses import dataclass, fields

import numpy as np

from locatebot.lidar.types import Scan

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 4 * 1024 * 1024  # JPEGs larger than this are a bug, not a photo
MAX_ID_CHARS = 64        # mission_id, boot_id, sensor
MAX_TARGET_CHARS = 120   # MissionRequest.target
MAX_MESSAGE_CHARS = 300  # MissionStatus.message
MAX_REASON_CHARS = 64    # MissionCancel.reason, MissionStatus.reason (short codes, may be empty)
# Mission states in protocol order (same order as orin/missions.TRANSITIONS).
MISSION_STATES = ("PENDING", "ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING",
                  "COMPLETED", "FAILED", "CANCELLED")
PI_PUB_PORT = 5555
ORIN_PUB_PORT = 5556


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    capture_s: float  # Pi monotonic clock
    seq: int


@dataclass(frozen=True)
class DriveCmd:
    throttle: float  # [-1, 1]
    steering: float  # [-1, 1], positive left
    seq: int
    sent_s: float    # Orin clock, diagnostics only


@dataclass(frozen=True)
class EStopCmd:
    engage: bool
    seq: int
    sent_s: float


@dataclass(frozen=True)
class PiStatus:
    stamp_s: float
    lidar_ok: bool
    scans_per_s: float
    camera_ok: bool
    estop: bool
    safety_reason: str  # locatebot.safety Reason value of the last decision
    throttle: float     # last throttle actually sent to the motors
    steering: float
    cpu_temp_c: float | None


@dataclass(frozen=True)
class OrinStatus:
    stamp_s: float
    vision_alive: bool
    vision_latency_s: float | None
    mission_state: str
    cpu_temp_c: float | None
    gpu_temp_c: float | None
    boot_id: str        # random per Orin process start (e.g. uuid4().hex); a change = Orin restarted


@dataclass(frozen=True)
class Range:
    """One distance reading from a single-beam sensor on the Pi (Pi -> Orin, topic ``range``).

    ``range_m`` is None when the sensor got no echo (nothing in range, or an absorbing/angled
    surface): unknown, never "free". ``fov_rad`` is the full cone angle (HC-SR04 ~0.26 rad).
    """

    stamp_s: float           # Pi monotonic clock, time of the measurement
    sensor: str              # e.g. "front_ultrasonic"
    range_m: float | None    # >= 0, None = no echo
    fov_rad: float           # full cone angle, (0, pi]


@dataclass(frozen=True)
class MissionRequest:
    """A customer/operator mission for the Orin (Pi -> Orin, topic ``mission_request``).

    ``target`` is the normalised object description (``/find blue toolbox`` -> "blue toolbox").
    Times are Unix wall-clock seconds; the Orin rejects expired or far-future requests.
    """

    mission_id: str
    target: str
    issued_s: float          # Unix wall clock
    expires_s: float         # Unix wall clock


@dataclass(frozen=True)
class MissionCancel:
    """Cancel a mission (Pi -> Orin, topic ``mission_cancel``). ``reason`` may be empty."""

    mission_id: str
    reason: str


@dataclass(frozen=True)
class MissionStatus:
    """Mission progress from the Orin (Orin -> Pi, topic ``mission_status``).

    ``state`` is one of ``MISSION_STATES``; ``message`` is a short human-readable text (may be
    empty); ``reason`` is a short code such as ``NOT_FOUND`` (empty when there is none).
    """

    mission_id: str
    state: str
    message: str
    reason: str


# field kinds for JSON-only messages: "f" float, "f?" optional float, "i" int, "b" bool, "s" str,
# and checked strings: "id" (1..MAX_ID_CHARS), "target" (1..MAX_TARGET_CHARS),
# "message" (0..MAX_MESSAGE_CHARS), "reason" (0..MAX_REASON_CHARS), "state" (in MISSION_STATES).
# Checked strings must not contain control characters; non-empty ones must not be blank.
_JSON_TYPES: dict[str, tuple[type, dict[str, str]]] = {
    "drive": (DriveCmd, {"throttle": "f", "steering": "f", "seq": "i", "sent_s": "f"}),
    "estop": (EStopCmd, {"engage": "b", "seq": "i", "sent_s": "f"}),
    "pi_status": (PiStatus, {"stamp_s": "f", "lidar_ok": "b", "scans_per_s": "f", "camera_ok": "b",
                             "estop": "b", "safety_reason": "s", "throttle": "f", "steering": "f",
                             "cpu_temp_c": "f?"}),
    "orin_status": (OrinStatus, {"stamp_s": "f", "vision_alive": "b", "vision_latency_s": "f?",
                                 "mission_state": "s", "cpu_temp_c": "f?", "gpu_temp_c": "f?",
                                 "boot_id": "id"}),
    "range": (Range, {"stamp_s": "f", "sensor": "id", "range_m": "f?", "fov_rad": "f"}),
    "mission_request": (MissionRequest, {"mission_id": "id", "target": "target", "issued_s": "f",
                                         "expires_s": "f"}),
    "mission_cancel": (MissionCancel, {"mission_id": "id", "reason": "reason"}),
    "mission_status": (MissionStatus, {"mission_id": "id", "state": "state", "message": "message",
                                       "reason": "reason"}),
}
# checked string kinds: (min length, max length)
_TEXT_LIMITS = {"id": (1, MAX_ID_CHARS), "target": (1, MAX_TARGET_CHARS),
                "message": (0, MAX_MESSAGE_CHARS), "reason": (0, MAX_REASON_CHARS)}
_TYPE_OF = {cls: name for name, (cls, _) in _JSON_TYPES.items()}
_SCAN_DT = [np.dtype("<f4"), np.dtype("<f4"), np.dtype("u1")]


def _check_drive(throttle: float, steering: float) -> None:
    for name, v in (("throttle", throttle), ("steering", steering)):
        if not (isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 1.0):
            raise ProtocolError(f"drive {name} must be finite and within [-1, 1], got {v!r}")


def _has_control(text: str) -> bool:
    return any(unicodedata.category(ch) == "Cc" for ch in text)


def _check_fields(type_: str, spec: dict[str, str], values: dict) -> None:
    """Rules shared by encode and decode beyond the basic JSON types."""
    for key, kind in spec.items():
        v = values[key]
        if kind in _TEXT_LIMITS or kind == "state":
            if not isinstance(v, str):
                raise ProtocolError(f"{type_}.{key} must be a string")
            if _has_control(v):
                raise ProtocolError(f"{type_}.{key} contains control characters")
        if kind in _TEXT_LIMITS:
            lo, hi = _TEXT_LIMITS[kind]
            if len(v) > hi:
                raise ProtocolError(f"{type_}.{key} longer than {hi} characters")
            if lo and not v.strip():
                raise ProtocolError(f"{type_}.{key} must not be empty")
        elif kind == "state" and v not in MISSION_STATES:
            raise ProtocolError(f"{type_}.{key} {v[:40]!r} is not a mission state")
    if type_ == "drive":
        _check_drive(values["throttle"], values["steering"])
    elif type_ == "range":
        r, fov = values["range_m"], values["fov_rad"]
        if r is not None and r < 0.0:
            raise ProtocolError(f"range.range_m must be >= 0 or null, got {r!r}")
        if not 0.0 < fov <= math.pi:
            raise ProtocolError(f"range.fov_rad must be in (0, pi], got {fov!r}")


def _header(type_: str, **fields_) -> bytes:
    return json.dumps({"v": PROTOCOL_VERSION, "type": type_, **fields_}, allow_nan=False,
                      separators=(",", ":")).encode()


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _wire_value(v):
    return None if isinstance(v, float) and not math.isfinite(v) else v


# --- encode ------------------------------------------------------------------

def encode(msg) -> list[bytes]:
    if isinstance(msg, Scan):
        n = len(msg)
        payload = (np.asarray(msg.angles_rad, _SCAN_DT[0]).tobytes()
                   + np.asarray(msg.ranges_m, _SCAN_DT[1]).tobytes()
                   + np.asarray(msg.quality, _SCAN_DT[2]).tobytes())
        return [b"scan", _header("scan", stamp_s=msg.stamp_s, seq=msg.seq, n=n), payload]
    if isinstance(msg, Frame):
        if len(msg.jpeg) > MAX_FRAME_BYTES:
            raise ProtocolError(f"frame of {len(msg.jpeg)} bytes exceeds {MAX_FRAME_BYTES}")
        if not _finite(msg.capture_s):
            raise ProtocolError(f"frame capture_s must be finite, got {msg.capture_s!r}")
        return [b"frame", _header("frame", capture_s=msg.capture_s, seq=msg.seq), bytes(msg.jpeg)]
    type_ = _TYPE_OF.get(type(msg))
    if type_ is None:
        raise TypeError(f"cannot encode {type(msg).__name__}")
    spec = _JSON_TYPES[type_][1]
    body = {}
    for f in fields(msg):
        v = getattr(msg, f.name)
        if spec[f.name] == "f" and not _finite(v):
            raise ProtocolError(f"{type_}.{f.name} must be finite, got {v!r}")
        body[f.name] = _wire_value(v)
    _check_fields(type_, spec, body)
    return [type_.encode(), _header(type_, **body), b""]


# --- decode ------------------------------------------------------------------

def _get(hdr: dict, key: str, kind: str):
    if key not in hdr:
        raise ProtocolError(f"missing header field {key!r}")
    v = hdr[key]
    if kind == "f?" and v is None:
        return None
    if kind in ("f", "f?"):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ProtocolError(f"{key} must be a number")
        try:
            f = float(v)
        except OverflowError:
            raise ProtocolError(f"{key} is out of range") from None
        if not math.isfinite(f):
            raise ProtocolError(f"{key} must be finite")
        return f
    if kind == "i":
        if isinstance(v, bool) or not isinstance(v, int):
            raise ProtocolError(f"{key} must be an integer")
        return v
    if kind == "b":
        if not isinstance(v, bool):
            raise ProtocolError(f"{key} must be a boolean")
        return v
    if not isinstance(v, str):
        raise ProtocolError(f"{key} must be a string")
    return v


def decode(parts: list[bytes]):
    if len(parts) != 3:
        raise ProtocolError(f"expected 3 frames, got {len(parts)}")
    topic, raw, payload = (bytes(p) for p in parts)
    try:
        hdr = json.loads(raw)
    except (ValueError, UnicodeDecodeError) as e:
        raise ProtocolError(f"header is not JSON: {e}") from None
    if not isinstance(hdr, dict):
        raise ProtocolError("header must be a JSON object")
    if hdr.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(f"protocol version {hdr.get('v')!r}, expected {PROTOCOL_VERSION}")
    type_ = hdr.get("type")
    if topic != str(type_).encode():
        raise ProtocolError(f"topic {topic!r} does not match header type {type_!r}")

    if type_ == "scan":
        n = _get(hdr, "n", "i")
        if n < 0 or len(payload) != 9 * n:
            raise ProtocolError(f"scan payload is {len(payload)} bytes, expected {9 * n}")
        a = np.frombuffer(payload, _SCAN_DT[0], n, 0)
        r = np.frombuffer(payload, _SCAN_DT[1], n, 4 * n)
        q = np.frombuffer(payload, _SCAN_DT[2], n, 8 * n)
        return Scan(stamp_s=_get(hdr, "stamp_s", "f"), seq=_get(hdr, "seq", "i"),
                    angles_rad=a.astype(np.float64), ranges_m=r.astype(np.float64), quality=q)
    if type_ == "frame":
        if len(payload) > MAX_FRAME_BYTES:
            raise ProtocolError(f"frame of {len(payload)} bytes exceeds {MAX_FRAME_BYTES}")
        return Frame(payload, _get(hdr, "capture_s", "f"), _get(hdr, "seq", "i"))
    if type_ not in _JSON_TYPES:
        raise ProtocolError(f"unknown message type {type_!r}")
    if payload:
        raise ProtocolError(f"unexpected payload on {type_} message")
    cls, spec = _JSON_TYPES[type_]
    values = {k: _get(hdr, k, kind) for k, kind in spec.items()}
    _check_fields(type_, spec, values)
    return cls(**values)
