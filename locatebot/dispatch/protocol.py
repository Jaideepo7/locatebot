"""Wire protocol between the backend and the Jetson (JSON text frames over a WebSocket).

Shared by both sides so they validate identically. Deliberately small: a customer can only ask
the robot to *find* something. There is no message type or field for motion, speed, or shell
commands, and adding one must be a reviewed change (see tests/dispatch/test_protocol.py).
"""
from __future__ import annotations

import json
import re
import unicodedata
import uuid
from datetime import datetime, timezone
from enum import Enum


class MissionState(str, Enum):
    PENDING = "PENDING"
    ACCEPTED = "ACCEPTED"
    SEARCHING = "SEARCHING"
    TARGET_DETECTED = "TARGET_DETECTED"
    APPROACHING = "APPROACHING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


S = MissionState
TERMINAL = frozenset({S.COMPLETED, S.FAILED, S.CANCELLED})
_ENDINGS = {S.FAILED, S.CANCELLED}
TRANSITIONS = {
    S.PENDING: {S.ACCEPTED} | _ENDINGS,
    S.ACCEPTED: {S.SEARCHING} | _ENDINGS,
    S.SEARCHING: {S.TARGET_DETECTED} | _ENDINGS,
    S.TARGET_DETECTED: {S.APPROACHING, S.SEARCHING, S.COMPLETED} | _ENDINGS,  # SEARCHING = target lost
    S.APPROACHING: {S.COMPLETED, S.SEARCHING} | _ENDINGS,
}


def can_transition(old: MissionState, new: MissionState) -> bool:
    return new in TRANSITIONS.get(old, ())


MISSION_TYPES = frozenset({"FIND_OBJECT"})
ACK_STATUSES = frozenset({"ACCEPTED", "REJECTED"})
# type -> (required fields besides "type", optional fields)
SCHEMA = {
    "MISSION_REQUEST": ({"mission_id", "mission_type", "target_description", "issued_at", "expires_at"}, set()),
    "MISSION_CANCEL": ({"mission_id"}, {"reason"}),
    "MISSION_ACK": ({"mission_id", "status"}, {"reason"}),
    "MISSION_STATUS": ({"mission_id", "status"}, {"message", "reason"}),
    "ERROR": ({"reason"}, {"message"}),  # Jetson -> backend: a frame it could not accept
}

MAX_FRAME_BYTES = 4096
MAX_DESCRIPTION_LEN = 120
MAX_MESSAGE_LEN = 300
_REASON_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,39}$")
_ALLOWED_PUNCT = set(" -'.,&/#()+")


class ProtocolError(ValueError):
    pass


class InvalidDescription(ValueError):
    pass


def normalize_description(raw: str) -> str:
    """Return a clean target description, or raise InvalidDescription with a customer-safe reason.

    Allowed: letters, marks, digits, spaces and  - ' . , & / # ( ) +
    Any whitespace run (including newlines) becomes one space. Everything else is rejected,
    which rules out shell metacharacters ($ ; ` | > < { }) and control characters.
    """
    if not isinstance(raw, str):
        raise InvalidDescription("Please describe the object in words.")
    text = " ".join(raw.split())
    if not text:
        raise InvalidDescription("Please tell me what object to look for, for example: /find blue toolbox")
    if len(text) > MAX_DESCRIPTION_LEN:
        raise InvalidDescription(f"That description is too long. Please keep it under {MAX_DESCRIPTION_LEN} characters.")
    for ch in text:
        if ch in _ALLOWED_PUNCT:
            continue
        if unicodedata.category(ch)[0] not in "LMN":
            raise InvalidDescription("Please use only letters, numbers and simple punctuation.")
    if sum(ch.isalnum() for ch in text) < 2:
        raise InvalidDescription("Please be more specific, for example: /find blue toolbox")
    return text


def format_ts(dt: datetime) -> str:
    dt = dt.astimezone(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S") + (f".{dt.microsecond // 1000:03d}" if dt.microsecond else "") + "Z"


_TS_RE = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{1,6})?Z$")


def parse_ts(value) -> datetime:
    if not isinstance(value, str) or not _TS_RE.match(value):
        raise ProtocolError(f"timestamp must be ISO-8601 UTC ending in Z: {value!r}")
    try:
        dt = datetime.fromisoformat(value[:-1]).replace(tzinfo=timezone.utc)
    except ValueError as e:
        raise ProtocolError(f"invalid timestamp {value!r}") from e
    if not 2000 <= dt.year < 2100:
        raise ProtocolError(f"timestamp out of range: {value!r}")
    return dt


def _require_uuid(value) -> None:
    if not isinstance(value, str):
        raise ProtocolError("mission_id must be a string")
    try:
        if str(uuid.UUID(value)) != value.lower():
            raise ValueError
    except ValueError:
        raise ProtocolError("mission_id must be a canonical UUID") from None


def _optional_text(msg: dict, key: str, limit: int) -> None:
    if key in msg and (not isinstance(msg[key], str) or len(msg[key]) > limit):
        raise ProtocolError(f"{key} must be a string of at most {limit} characters")


def validate(msg) -> dict:
    """Check a decoded message against SCHEMA and the per-field rules. Returns it unchanged."""
    if not isinstance(msg, dict):
        raise ProtocolError("frame must be a JSON object")
    kind = msg.get("type")
    if kind not in SCHEMA:
        raise ProtocolError(f"unknown message type {kind!r}")
    required, optional = SCHEMA[kind]
    keys = set(msg) - {"type"}
    if required - keys:
        raise ProtocolError(f"{kind} missing {sorted(required - keys)}")
    if keys - required - optional:
        raise ProtocolError(f"{kind} has unknown fields {sorted(keys - required - optional)}")
    if "mission_id" in required:
        _require_uuid(msg["mission_id"])
    _optional_text(msg, "message", MAX_MESSAGE_LEN)
    if "reason" in msg and (not isinstance(msg["reason"], str) or not _REASON_RE.match(msg["reason"])):
        raise ProtocolError("reason must be an UPPER_SNAKE code")
    if kind == "MISSION_REQUEST":
        if msg["mission_type"] not in MISSION_TYPES:
            raise ProtocolError(f"unsupported mission_type {msg['mission_type']!r}")
        try:
            if normalize_description(msg["target_description"]) != msg["target_description"]:
                raise ProtocolError("target_description is not normalised")
        except InvalidDescription as e:
            raise ProtocolError(f"invalid target_description: {e}") from None
        parse_ts(msg["issued_at"])
        parse_ts(msg["expires_at"])
    elif kind == "MISSION_ACK":
        if msg["status"] not in ACK_STATUSES:
            raise ProtocolError("ack status must be ACCEPTED or REJECTED")
    elif kind == "MISSION_STATUS":
        if msg["status"] not in MissionState.__members__:
            raise ProtocolError(f"unknown mission status {msg['status']!r}")
    return msg


def decode(frame) -> dict:
    if isinstance(frame, (bytes, bytearray)):
        raise ProtocolError("binary frames are not accepted")
    if not isinstance(frame, str) or len(frame.encode("utf-8")) > MAX_FRAME_BYTES:
        raise ProtocolError("frame too large or not text")
    try:
        msg = json.loads(frame)
    except ValueError:
        raise ProtocolError("frame is not valid JSON") from None
    return validate(msg)


def encode(msg: dict) -> str:
    return json.dumps(validate(msg), separators=(",", ":"), ensure_ascii=False)
