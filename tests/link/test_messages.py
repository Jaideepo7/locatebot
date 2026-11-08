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
    EStopCmd,
    Frame,
    OrinStatus,
    PiStatus,
    ProtocolError,
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
                cpu_temp_c=50.0, gpu_temp_c=None), b"orin_status"),
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
