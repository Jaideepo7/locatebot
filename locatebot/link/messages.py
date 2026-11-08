"""Wire format between the Raspberry Pi and the Jetson Orin.

Every message is three byte frames, ready for a ZeroMQ multipart send:

    [topic, header, payload]

- ``topic``: the message type as ASCII (``b"scan"``), so SUB sockets can filter by prefix.
- ``header``: UTF-8 JSON object with ``"v"`` (protocol version), ``"type"`` and the fields.
- ``payload``: binary data (scan arrays, JPEG bytes) or ``b""``.

Directions and clocks
- Pi -> Orin: ``Scan``, ``Frame``, ``PiStatus``. All their times are the **Pi's monotonic
  clock**, so scan stamps and frame capture times line up on the Orin (pose lookup for
  item localization uses both).
- Orin -> Pi: ``DriveCmd``, ``EStopCmd``, ``OrinStatus``. ``sent_s`` is the Orin's clock and
  is for latency diagnostics only. The Pi must stamp a command with its own clock **on
  receipt** before giving it to the safety supervisor; it never compares Orin times with
  Pi times.

Decoding is strict: wrong frame count, version, type, missing fields, bad payload size or
out-of-range drive values raise ``ProtocolError``; the receiver drops the message (for drive
commands the safety supervisor then times out and stops). Unknown extra header fields are
ignored so a newer sender can add fields without breaking an older receiver.

Planned sockets (not implemented here): Pi PUB on tcp port 5555 (scan, frame, pi_status),
Orin PUB on tcp port 5556 (drive, estop, orin_status); frame subscribers set CONFLATE so
only the newest frame is kept.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields

import numpy as np

from locatebot.lidar.types import Scan

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 4 * 1024 * 1024  # JPEGs larger than this are a bug, not a photo
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


# field kinds for JSON-only messages: "f" float, "f?" optional float, "i" int, "b" bool, "s" str
_JSON_TYPES: dict[str, tuple[type, dict[str, str]]] = {
    "drive": (DriveCmd, {"throttle": "f", "steering": "f", "seq": "i", "sent_s": "f"}),
    "estop": (EStopCmd, {"engage": "b", "seq": "i", "sent_s": "f"}),
    "pi_status": (PiStatus, {"stamp_s": "f", "lidar_ok": "b", "scans_per_s": "f", "camera_ok": "b",
                             "estop": "b", "safety_reason": "s", "throttle": "f", "steering": "f",
                             "cpu_temp_c": "f?"}),
    "orin_status": (OrinStatus, {"stamp_s": "f", "vision_alive": "b", "vision_latency_s": "f?",
                                 "mission_state": "s", "cpu_temp_c": "f?", "gpu_temp_c": "f?"}),
}
_TYPE_OF = {cls: name for name, (cls, _) in _JSON_TYPES.items()}
_SCAN_DT = [np.dtype("<f4"), np.dtype("<f4"), np.dtype("u1")]


def _check_drive(throttle: float, steering: float) -> None:
    for name, v in (("throttle", throttle), ("steering", steering)):
        if not (isinstance(v, (int, float)) and math.isfinite(v) and abs(v) <= 1.0):
            raise ProtocolError(f"drive {name} must be finite and within [-1, 1], got {v!r}")


def _header(type_: str, **fields_) -> bytes:
    return json.dumps({"v": PROTOCOL_VERSION, "type": type_, **fields_}, allow_nan=False,
                      separators=(",", ":")).encode()


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
        return [b"frame", _header("frame", capture_s=msg.capture_s, seq=msg.seq), bytes(msg.jpeg)]
    type_ = _TYPE_OF.get(type(msg))
    if type_ is None:
        raise TypeError(f"cannot encode {type(msg).__name__}")
    if isinstance(msg, DriveCmd):
        _check_drive(msg.throttle, msg.steering)
    body = {f.name: _wire_value(getattr(msg, f.name)) for f in fields(msg)}
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
        return float(v)
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
    if cls is DriveCmd:
        _check_drive(values["throttle"], values["steering"])
    return cls(**values)
