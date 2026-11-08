"""Tests for locatebot.link.messages (pure encode/decode, no sockets)."""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D, Scan
from locatebot.link.messages import (
    MAX_FRAME_BYTES,
    PROTOCOL_VERSION,
    DriveCmd,
    MAX_ID_CHARS,
    MAX_MESSAGE_CHARS,
    MAX_REASON_CHARS,
    MAX_TARGET_CHARS,
    MISSION_STATES,
    EStopCmd,
    Frame,
    MissionCancel,
    MissionRequest,
    MissionStatus,
    OrinStatus,
    PiStatus,
    ProtocolError,
    Range,
    decode,
    encode,
)


def sample_scan(dropout=0.1) -> Scan:
    return simulate_scan(Room.box(6, 4), Pose2D(2, 2, 0.3), n_beams=450, stamp_s=123.456, seq=42, dropout=dropout)


def pi_status(**kw):
    base = dict(stamp_s=10.0, lidar_ok=True, scans_per_s=9.8, camera_ok=True, estop=False,
                safety_reason="ok", throttle=0.2, steering=-0.1, cpu_temp_c=55.5)
    base.update(kw)
    return PiStatus(**base)


# --- framing -----------------------------------------------------------------

@pytest.mark.parametrize("msg,topic", [
    (Frame(b"\xff\xd8jpeg", capture_s=1.0, seq=1), b"frame"),
    (DriveCmd(0.3, -0.2, seq=7, sent_s=5.0), b"drive"),
    (EStopCmd(True, seq=1, sent_s=1.0), b"estop"),
    (pi_status(), b"pi_status"),
    (OrinStatus(stamp_s=1.0, vision_alive=True, vision_latency_s=2.5, mission_state="SEARCHING",
                cpu_temp_c=50.0, gpu_temp_c=None, boot_id="b007"), b"orin_status"),
    (Range(stamp_s=2.0, sensor="front_ultrasonic", range_m=0.6, fov_rad=0.26), b"range"),
    (MissionRequest(mission_id="m-1", target="blue toolbox", issued_s=1.7e9, expires_s=1.7e9 + 600),
     b"mission_request"),
    (MissionCancel(mission_id="m-1", reason="USER_CANCELLED"), b"mission_cancel"),
    (MissionStatus(mission_id="m-1", state="SEARCHING", message="Looking for blue toolbox", reason=""),
     b"mission_status"),
])
def test_encode_is_three_frames_topic_header_payload(msg, topic):
    parts = encode(msg)
    assert len(parts) == 3 and all(isinstance(p, bytes) for p in parts)
    assert parts[0] == topic
    hdr = json.loads(parts[1])
    assert hdr["v"] == PROTOCOL_VERSION and hdr["type"] == topic.decode()
    assert decode(parts) == msg


def test_scan_topic():
    assert encode(sample_scan())[0] == b"scan"


# --- scans ---------------------------------------------------------------------

def test_scan_roundtrip_keeps_nan_as_unknown_and_metadata():
    s = sample_scan()
    out = decode(encode(s))
    assert isinstance(out, Scan)
    assert (out.stamp_s, out.seq) == (123.456, 42)
    assert np.array_equal(np.isnan(out.ranges_m), np.isnan(s.ranges_m))
    ok = np.isfinite(s.ranges_m)
    assert np.allclose(out.ranges_m[ok], s.ranges_m[ok], atol=1e-5)  # float32 on the wire
    assert np.allclose(out.angles_rad, s.angles_rad, atol=1e-6)
    assert np.array_equal(out.quality, s.quality)


def test_scan_payload_is_compact_binary():
    s = sample_scan()
    assert len(encode(s)[2]) == len(s) * 9  # float32 angle + float32 range + uint8 quality


def test_empty_scan_roundtrip():
    out = decode(encode(Scan(1.0, 0, [], [], [])))
    assert len(out) == 0


def test_scan_with_wrong_payload_length_rejected():
    parts = encode(sample_scan())
    with pytest.raises(ProtocolError, match="payload"):
        decode([parts[0], parts[1], parts[2][:-1]])


# --- frames --------------------------------------------------------------------

def test_frame_carries_jpeg_bytes_unchanged():
    jpeg = bytes(range(256)) * 100
    out = decode(encode(Frame(jpeg, capture_s=3.25, seq=9)))
    assert out.jpeg == jpeg and out.capture_s == 3.25 and out.seq == 9


def test_oversized_frame_rejected_on_both_sides():
    big = b"\x00" * (MAX_FRAME_BYTES + 1)
    with pytest.raises(ProtocolError):
        encode(Frame(big, capture_s=1.0, seq=1))
    hdr = json.dumps({"v": PROTOCOL_VERSION, "type": "frame", "capture_s": 1.0, "seq": 1}).encode()
    with pytest.raises(ProtocolError):
        decode([b"frame", hdr, big])


# --- commands ------------------------------------------------------------------

@pytest.mark.parametrize("throttle,steering", [(math.nan, 0.0), (0.0, math.inf), (1.2, 0.0), (0.0, -1.5)])
def test_drive_with_bad_values_rejected(throttle, steering):
    with pytest.raises(ProtocolError):
        encode(DriveCmd(throttle, steering, seq=1, sent_s=0.0))
    hdr = json.dumps({"v": PROTOCOL_VERSION, "type": "drive", "throttle": 0.0, "steering": 0.0,
                      "seq": 1, "sent_s": 0.0}).encode()
    bad = json.loads(hdr)
    bad["throttle"], bad["steering"] = throttle, steering
    with pytest.raises(ProtocolError):
        decode([b"drive", json.dumps(bad).encode(), b""])


def test_estop_engage_and_release_roundtrip():
    for engage in (True, False):
        assert decode(encode(EStopCmd(engage, seq=3, sent_s=2.0))).engage is engage


# --- status --------------------------------------------------------------------

def test_status_optional_fields_may_be_none():
    s = pi_status(cpu_temp_c=None)
    assert decode(encode(s)).cpu_temp_c is None


def test_status_nan_is_sent_as_none():
    out = decode(encode(pi_status(cpu_temp_c=math.nan)))
    assert out.cpu_temp_c is None


def test_unknown_header_fields_are_ignored_for_forward_compatibility():
    parts = encode(pi_status())
    hdr = json.loads(parts[1])
    hdr["battery_v"] = 7.4
    assert decode([parts[0], json.dumps(hdr).encode(), parts[2]]) == pi_status()


# --- malformed input -------------------------------------------------------------

def test_wrong_number_of_frames_rejected():
    parts = encode(DriveCmd(0.1, 0.0, seq=1, sent_s=0.0))
    with pytest.raises(ProtocolError):
        decode(parts[:2])
    with pytest.raises(ProtocolError):
        decode(parts + [b"extra"])


def test_version_mismatch_rejected():
    parts = encode(DriveCmd(0.1, 0.0, seq=1, sent_s=0.0))
    hdr = json.loads(parts[1])
    hdr["v"] = PROTOCOL_VERSION + 1
    with pytest.raises(ProtocolError, match="version"):
        decode([parts[0], json.dumps(hdr).encode(), parts[2]])


def test_unknown_type_rejected():
    hdr = json.dumps({"v": PROTOCOL_VERSION, "type": "teleport"}).encode()
    with pytest.raises(ProtocolError, match="type"):
        decode([b"teleport", hdr, b""])


def test_topic_header_mismatch_rejected():
    parts = encode(DriveCmd(0.1, 0.0, seq=1, sent_s=0.0))
    with pytest.raises(ProtocolError):
        decode([b"estop", parts[1], parts[2]])


@pytest.mark.parametrize("hdr", [b"not json", b"[1,2]", b'{"v":1,"type":"drive"}'])
def test_bad_or_incomplete_header_rejected(hdr):
    with pytest.raises(ProtocolError):
        decode([b"drive", hdr, b""])


def test_unexpected_payload_on_json_only_message_rejected():
    parts = encode(DriveCmd(0.1, 0.0, seq=1, sent_s=0.0))
    with pytest.raises(ProtocolError, match="payload"):
        decode([parts[0], parts[1], b"junk"])


def test_encode_rejects_unknown_object():
    with pytest.raises(TypeError):
        encode("hello")


# --- review fixes: every malformed input is a ProtocolError ------------------------

def _hdr(**kw):
    return json.dumps({"v": PROTOCOL_VERSION, **kw}).encode()


@pytest.mark.parametrize("parts", [
    [b"scan", _hdr(type="scan", stamp_s=float("nan"), seq=1, n=0), b""],
    [b"frame", _hdr(type="frame", capture_s=float("inf"), seq=1), b"x"],
    [b"drive", _hdr(type="drive", throttle=0.0, steering=0.0, seq=1, sent_s=float("nan")), b""],
    [b"drive", b'{"v":1,"type":"drive","throttle":0,"steering":0,"seq":1,"sent_s":1' + b"0" * 400 + b"}", b""],
])
def test_non_finite_or_overflowing_times_are_protocol_errors(parts):
    with pytest.raises(ProtocolError):
        decode(parts)


def test_encode_rejects_non_finite_times():
    with pytest.raises(ProtocolError):
        encode(Frame(b"x", capture_s=float("nan"), seq=1))
    with pytest.raises(ProtocolError):
        encode(DriveCmd(0.1, 0.0, seq=1, sent_s=float("inf")))


# --- Step C contracts: range, missions, boot_id -------------------------------------

def orin_status(**kw):
    base = dict(stamp_s=1.0, vision_alive=False, vision_latency_s=None, mission_state="idle",
                cpu_temp_c=None, gpu_temp_c=None, boot_id="3f2a" * 8)
    base.update(kw)
    return OrinStatus(**base)


def rng(**kw):
    base = dict(stamp_s=5.0, sensor="front_ultrasonic", range_m=0.6, fov_rad=0.26)
    base.update(kw)
    return Range(**base)


def request(**kw):
    base = dict(mission_id="5b0c1f0e-1d2a-4b8e-9f00-123456789abc", target="blue toolbox",
                issued_s=1_762_600_000.0, expires_s=1_762_600_600.0)
    base.update(kw)
    return MissionRequest(**base)


def status(**kw):
    base = dict(mission_id="m-1", state="ACCEPTED", message="On it", reason="")
    base.update(kw)
    return MissionStatus(**base)


def cancel(**kw):
    base = dict(mission_id="m-1", reason="")
    base.update(kw)
    return MissionCancel(**base)


def _forged(msg, **changes):
    """Encode ``msg`` and edit its JSON header, to test what decode accepts."""
    parts = encode(msg)
    hdr = json.loads(parts[1])
    for k, v in changes.items():
        if v is _DROP:
            del hdr[k]
        else:
            hdr[k] = v
    return [parts[0], json.dumps(hdr).encode(), parts[2]]


_DROP = object()


def test_orin_status_carries_boot_id():
    s = orin_status()
    out = decode(encode(s))
    assert out.boot_id == "3f2a" * 8 and out == s
    with pytest.raises(ProtocolError, match="boot_id"):
        decode(_forged(s, boot_id=_DROP))
    for bad in ("", "  ", "x" * (MAX_ID_CHARS + 1), "a\nb"):
        with pytest.raises(ProtocolError):
            encode(orin_status(boot_id=bad))
        with pytest.raises(ProtocolError):
            decode(_forged(s, boot_id=bad))


def test_boot_id_is_required_by_the_dataclass():
    with pytest.raises(TypeError):
        OrinStatus(1.0, True, None, "idle", None, None)  # noqa - old 6-field call must not pass silently


@pytest.mark.parametrize("msg", [
    rng(), rng(range_m=None), rng(range_m=0.0), rng(fov_rad=math.pi),
    request(), request(target="x" * MAX_TARGET_CHARS),
    cancel(), cancel(reason="USER_CANCELLED"), cancel(mission_id="i" * MAX_ID_CHARS),
    status(message="", reason=""), status(message="m" * MAX_MESSAGE_CHARS, reason="r" * MAX_REASON_CHARS),
    status(message="Found the blue toolbox at (1.2, -0.4) \u2014 approaching", state="TARGET_DETECTED"),
])
def test_new_messages_round_trip(msg):
    parts = encode(msg)
    assert parts[2] == b""
    assert json.loads(parts[1])["type"] == parts[0].decode()
    assert decode(parts) == msg


@pytest.mark.parametrize("state", MISSION_STATES)
def test_every_mission_state_round_trips(state):
    assert decode(encode(status(state=state))).state == state


def test_mission_states_are_the_protocol_list():
    assert MISSION_STATES == ("PENDING", "ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING",
                              "COMPLETED", "FAILED", "CANCELLED")


@pytest.mark.parametrize("state", ["", "searching", "DONE", "SEARCHING ", 3, None])
def test_wrong_mission_state_rejected_on_encode_and_decode(state):
    with pytest.raises(ProtocolError):
        encode(status(state=state))
    with pytest.raises(ProtocolError):
        decode(_forged(status(), state=state))


def test_range_no_echo_is_null_and_nan_becomes_null():
    parts = encode(rng(range_m=None))
    assert json.loads(parts[1])["range_m"] is None
    assert decode(encode(rng(range_m=math.nan))).range_m is None


@pytest.mark.parametrize("kw", [dict(range_m=-0.1), dict(fov_rad=0.0), dict(fov_rad=-0.2), dict(fov_rad=4.0),
                                dict(fov_rad=math.nan), dict(stamp_s=math.inf), dict(sensor=""),
                                dict(sensor="s" * (MAX_ID_CHARS + 1))])
def test_bad_range_rejected_on_both_sides(kw):
    with pytest.raises(ProtocolError):
        encode(rng(**kw))
    wire = {k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in kw.items()}
    with pytest.raises(ProtocolError):
        decode(_forged(rng(), **wire))


@pytest.mark.parametrize("msg,field,limit", [
    (request(), "target", MAX_TARGET_CHARS),
    (request(), "mission_id", MAX_ID_CHARS),
    (status(), "message", MAX_MESSAGE_CHARS),
    (status(), "reason", MAX_REASON_CHARS),
    (status(), "mission_id", MAX_ID_CHARS),
    (cancel(), "reason", MAX_REASON_CHARS),
    (cancel(), "mission_id", MAX_ID_CHARS),
])
def test_string_limits(msg, field, limit):
    import dataclasses
    ok = dataclasses.replace(msg, **{field: "a" * limit})
    assert decode(encode(ok)) == ok
    long = dataclasses.replace(msg, **{field: "a" * (limit + 1)})
    with pytest.raises(ProtocolError, match="longer"):
        encode(long)
    with pytest.raises(ProtocolError, match="longer"):
        decode(_forged(msg, **{field: "a" * (limit + 1)}))


@pytest.mark.parametrize("bad", ["a\x00b", "line\nbreak", "tab\there", "del\x7f", "c1\x85"])
@pytest.mark.parametrize("msg,field", [
    (request(), "target"), (request(), "mission_id"), (status(), "message"), (status(), "reason"),
    (cancel(), "reason"), (rng(), "sensor"),
])
def test_control_characters_rejected(msg, field, bad):
    import dataclasses
    with pytest.raises(ProtocolError, match="control"):
        encode(dataclasses.replace(msg, **{field: bad}))
    with pytest.raises(ProtocolError, match="control"):
        decode(_forged(msg, **{field: bad}))


@pytest.mark.parametrize("msg,field", [
    (request(), "target"), (request(), "mission_id"), (status(), "mission_id"), (cancel(), "mission_id"),
])
def test_required_strings_must_not_be_blank(msg, field):
    import dataclasses
    for blank in ("", "   "):
        with pytest.raises(ProtocolError, match="empty"):
            encode(dataclasses.replace(msg, **{field: blank}))
        with pytest.raises(ProtocolError, match="empty"):
            decode(_forged(msg, **{field: blank}))


@pytest.mark.parametrize("msg", [rng(), request(), cancel(), status(), orin_status()])
def test_new_messages_missing_field_rejected(msg):
    import dataclasses
    for f in dataclasses.fields(msg):
        with pytest.raises(ProtocolError, match="missing"):
            decode(_forged(msg, **{f.name: _DROP}))


@pytest.mark.parametrize("msg", [rng(), request(), cancel(), status(), orin_status()])
def test_new_messages_ignore_extra_fields(msg):
    assert decode(_forged(msg, priority=5, note="from a newer sender")) == msg


@pytest.mark.parametrize("msg,field,value", [
    (request(), "target", 5), (request(), "issued_s", "soon"), (request(), "expires_s", None),
    (request(), "issued_s", True), (cancel(), "reason", None), (status(), "message", ["x"]),
    (rng(), "range_m", "far"), (rng(), "fov_rad", None), (orin_status(), "boot_id", 7),
])
def test_new_messages_wrong_types_rejected(msg, field, value):
    with pytest.raises(ProtocolError):
        decode(_forged(msg, **{field: value}))


def test_new_messages_reject_payload_and_topic_mismatch():
    parts = encode(request())
    with pytest.raises(ProtocolError, match="payload"):
        decode([parts[0], parts[1], b"junk"])
    with pytest.raises(ProtocolError):
        decode([b"mission_status", parts[1], parts[2]])


def test_mission_times_must_be_finite():
    with pytest.raises(ProtocolError):
        encode(request(expires_s=math.inf))
    with pytest.raises(ProtocolError):
        decode(_forged(request(), issued_s=None))
