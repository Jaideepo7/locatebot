import json
from datetime import datetime, timedelta, timezone

import pytest

from locatebot.dispatch import protocol as p
from locatebot.dispatch.protocol import MissionState as S


NOW = datetime(2026, 11, 7, 12, 0, 0, tzinfo=timezone.utc)
MID = "3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70"


def request(**over):
    msg = {
        "type": "MISSION_REQUEST",
        "mission_id": MID,
        "mission_type": "FIND_OBJECT",
        "target_description": "blue toolbox",
        "issued_at": p.format_ts(NOW),
        "expires_at": p.format_ts(NOW + timedelta(minutes=10)),
    }
    msg.update(over)
    return msg


# --- description rules (P2, P9-P13, J4) ---------------------------------------------------

def test_description_collapses_whitespace_and_keeps_case():
    assert p.normalize_description("  Blue   Toolbox \t") == "Blue Toolbox"


@pytest.mark.parametrize("raw", ["", "   ", "???", "🔧", "--", "a"])
def test_description_needs_real_words(raw):
    with pytest.raises(p.InvalidDescription):
        p.normalize_description(raw)


def test_description_length_limit():
    p.normalize_description("x" * p.MAX_DESCRIPTION_LEN)
    with pytest.raises(p.InvalidDescription):
        p.normalize_description("x" * (p.MAX_DESCRIPTION_LEN + 1))


@pytest.mark.parametrize("raw", ["$(rm -rf /)", "; reboot", "red cup`id`", "cup | sh", "cup > /dev/null",
                                 "cup\x00", "cup\x1b[2J", "cup{}", "<script>"])
def test_description_rejects_shell_and_control_characters(raw):
    with pytest.raises(p.InvalidDescription):
        p.normalize_description(raw)


def test_description_newlines_become_spaces():
    assert p.normalize_description("red\ncup\r\n") == "red cup"


def test_description_allows_common_object_wording():
    for ok in ["1/2-inch bolt", "Dr. Pepper can", "kid's red ball", "M&M bag", "café mug", "box #3 (small)"]:
        assert p.normalize_description(ok) == ok


def test_description_motion_words_are_only_text():
    assert p.normalize_description("forward 5m full speed") == "forward 5m full speed"


# --- encode / decode (J1-J7) ----------------------------------------------------------------

def test_request_round_trips():
    msg = request()
    assert p.decode(p.encode(msg)) == msg


@pytest.mark.parametrize("field", ["mission_id", "mission_type", "target_description", "issued_at", "expires_at"])
def test_request_missing_field_rejected(field):
    msg = request()
    del msg[field]
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(msg))


def test_unknown_fields_and_types_rejected():
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(request(speed=3.0)))
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps({"type": "DRIVE", "speed": 1}))
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(request(mission_type="DRIVE_TO")))


@pytest.mark.parametrize("bad", ["not json", "[]", "null", '"x"', "{"])
def test_garbage_is_protocol_error(bad):
    with pytest.raises(p.ProtocolError):
        p.decode(bad)


def test_wrong_types_rejected():
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(request(target_description=5)))
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(request(mission_id="not-a-uuid")))


def test_target_rules_checked_on_decode():
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(request(target_description="; reboot")))


@pytest.mark.parametrize("ts", ["2026-11-07 12:00:00", "2026-11-07T12:00:00", "2026-11-07T12:00:00+02:00",
                                "9999-01-01T00:00:00Z", "NaN", "", "2026-13-01T00:00:00Z"])
def test_bad_timestamps_rejected(ts):
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(request(issued_at=ts)))


def test_timestamps_parse_with_z_and_fraction():
    assert p.parse_ts("2026-11-07T12:00:00Z") == NOW
    assert p.parse_ts("2026-11-07T12:00:00.250Z") == NOW + timedelta(milliseconds=250)


def test_status_values_checked():
    ok = {"type": "MISSION_STATUS", "mission_id": MID, "status": "SEARCHING", "message": "Searching"}
    assert p.decode(json.dumps(ok)) == ok
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(dict(ok, status="DANCING")))
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(dict(ok, message="x" * 1000)))


def test_ack_values_checked():
    ok = {"type": "MISSION_ACK", "mission_id": MID, "status": "ACCEPTED"}
    assert p.decode(json.dumps(ok)) == ok
    rej = {"type": "MISSION_ACK", "mission_id": MID, "status": "REJECTED", "reason": "BUSY"}
    assert p.decode(json.dumps(rej)) == rej
    with pytest.raises(p.ProtocolError):
        p.decode(json.dumps(dict(ok, status="SEARCHING")))


def test_oversized_frame_rejected_before_parsing():
    with pytest.raises(p.ProtocolError):
        p.decode("{" + " " * p.MAX_FRAME_BYTES + "}")


# --- states and transitions (M4, M5) -------------------------------------------------------

def test_eight_states_exactly():
    assert {s.value for s in S} == {"PENDING", "ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING",
                                    "COMPLETED", "FAILED", "CANCELLED"}


@pytest.mark.parametrize("a,b", [(S.PENDING, S.ACCEPTED), (S.ACCEPTED, S.SEARCHING), (S.SEARCHING, S.TARGET_DETECTED),
                                 (S.TARGET_DETECTED, S.APPROACHING), (S.APPROACHING, S.COMPLETED),
                                 (S.TARGET_DETECTED, S.SEARCHING), (S.TARGET_DETECTED, S.COMPLETED),
                                 (S.SEARCHING, S.FAILED), (S.APPROACHING, S.CANCELLED), (S.PENDING, S.FAILED)])
def test_legal_transitions(a, b):
    assert p.can_transition(a, b)


@pytest.mark.parametrize("a,b", [(S.PENDING, S.COMPLETED), (S.ACCEPTED, S.COMPLETED), (S.SEARCHING, S.COMPLETED),
                                 (S.COMPLETED, S.SEARCHING), (S.CANCELLED, S.SEARCHING), (S.FAILED, S.COMPLETED),
                                 (S.SEARCHING, S.PENDING)])
def test_illegal_transitions(a, b):
    assert not p.can_transition(a, b)


# --- S1: the wire format can never carry motion or shell commands ----------------------------

def test_message_schema_is_frozen():
    assert p.SCHEMA == {
        "MISSION_REQUEST": ({"mission_id", "mission_type", "target_description", "issued_at", "expires_at"}, set()),
        "MISSION_CANCEL": ({"mission_id"}, {"reason"}),
        "MISSION_ACK": ({"mission_id", "status"}, {"reason"}),
        "MISSION_STATUS": ({"mission_id", "status"}, {"message", "reason"}),
        "ERROR": ({"reason"}, {"message"}),
    }
    assert p.MISSION_TYPES == {"FIND_OBJECT"}
