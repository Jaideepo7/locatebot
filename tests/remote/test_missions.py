import json
import uuid

import pytest

from locatebot.remote.messaging import Conversation, InboundMessage
from locatebot.remote.missions import AlreadyFinished, MissionNotFound, MissionService, RobotBusy, RobotUnavailable
from locatebot.remote.store import Store
from locatebot.remote import protocol as p

T0 = 1_794_000_000.0
ROBOT = "jetson-01"


class Clock:
    def __init__(self):
        self.t = T0

    def __call__(self):
        return self.t


class FakeLink:
    def __init__(self, connected=True):
        self.connected = connected
        self.sent = []
        self.fail_sends = False

    def send(self, msg):
        if not self.connected or self.fail_sends:
            return False
        p.validate(msg)  # everything the backend sends must be a valid protocol frame
        self.sent.append(msg)
        return True


_n = 0


def inbound(text, conv="pho_spc_01jaaaaaaaaaaaaaaaaaaaaaaa", sender="pho_usr_01jaaaaaaaaaaaaaaaaaaaaaaa", event_id=None):
    global _n
    _n += 1
    return InboundMessage(event_id or f"pho_evt_{_n:026d}", Conversation("photon", conv, "pho_usr_bot"),
                          sender, "someone", text)


CONV_B = "pho_spc_01jbbbbbbbbbbbbbbbbbbbbbbb"
USR_B = "pho_usr_01jbbbbbbbbbbbbbbbbbbbbbbb"


@pytest.fixture
def rig(tmp_path):
    clock, link = Clock(), FakeLink()
    store = Store(str(tmp_path / "d.db"))
    svc = MissionService(store, link, robot_id=ROBOT, clock=clock, ttl_s=600, ack_timeout_s=10, link_grace_s=15)
    return svc, store, link, clock


def replies(store, conv=None):
    return [o.text for o in store.list_outbox() if conv is None or o.conversation.conversation_id == conv]


def robot(svc, mission_id, kind="MISSION_STATUS", **fields):
    svc.on_robot_frame(json.dumps({"type": kind, "mission_id": mission_id, **fields}))


def start_find(svc, link, text="/find blue toolbox", **kw):
    svc.handle_inbound(inbound(text, **kw))
    return link.sent[-1]["mission_id"]


# --- E1 happy path -------------------------------------------------------------------------

def test_happy_path_messages_in_order(rig):
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    req = link.sent[-1]
    assert req["type"] == "MISSION_REQUEST" and req["target_description"] == "blue toolbox"
    assert req["mission_type"] == "FIND_OBJECT"
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="SEARCHING", message="Searching for blue toolbox")
    robot(svc, mid, status="TARGET_DETECTED")
    robot(svc, mid, status="APPROACHING")
    robot(svc, mid, status="COMPLETED")
    assert replies(store) == [
        "Request received. Sending your target to the robot.",
        "The robot has started searching for: blue toolbox.",
        "Target detected. Preparing to approach it.",
        "The robot is approaching the target.",
        "Search mission completed.",
    ]
    m = svc.get(mid)
    assert m.state == p.MissionState.COMPLETED and m.last_message == "Searching for blue toolbox"


def test_mission_id_and_timestamps_server_generated(rig):  # M1, M2
    svc, _, link, _ = rig
    mid = start_find(svc, link)
    assert uuid.UUID(mid).version == 4
    req = link.sent[-1]
    assert p.parse_ts(req["issued_at"]).timestamp() == T0
    assert p.parse_ts(req["expires_at"]).timestamp() == T0 + 600


# --- duplicates ------------------------------------------------------------------------------

def test_duplicate_event_creates_one_mission(rig):  # D1, D2
    svc, store, link, _ = rig
    msg = inbound("/find blue toolbox")
    for _ in range(3):
        svc.handle_inbound(msg)
    assert len(link.sent) == 1
    assert replies(store) == ["Request received. Sending your target to the robot."]


def test_double_tap_same_target(rig):  # D3
    svc, store, link, _ = rig
    start_find(svc, link)
    svc.handle_inbound(inbound("/find Blue  Toolbox"))
    assert len(link.sent) == 1
    assert replies(store)[-1] == "I'm already searching for: blue toolbox."


def test_crash_mid_transaction_leaves_nothing(rig, monkeypatch):  # D4
    svc, store, link, _ = rig
    msg = inbound("/find blue toolbox")

    def boom(*a, **k):
        raise RuntimeError("crash")

    monkeypatch.setattr(store, "enqueue", boom)
    with pytest.raises(RuntimeError):
        svc.handle_inbound(msg)
    monkeypatch.undo()
    assert store.active_mission(ROBOT) is None and link.sent == []
    svc.handle_inbound(msg)  # Photon's retry now succeeds exactly once
    assert len(link.sent) == 1


def test_api_idempotency_key(rig):  # D6
    svc, _, link, _ = rig
    a, created_a = svc.create_mission("red cup", idempotency_key="op-1")
    b, created_b = svc.create_mission("red cup", idempotency_key="op-1")
    assert a.mission_id == b.mission_id and created_a and not created_b
    assert len(link.sent) == 1


# --- one mission at a time, ownership ----------------------------------------------------------

def test_busy_with_own_other_target(rig):  # M3, E9
    svc, store, link, _ = rig
    start_find(svc, link, "/find a b")
    svc.handle_inbound(inbound("/find red cup"))
    assert replies(store)[-1] == "I'm already searching for: a b. Send /cancel first if you want something else."
    assert len(link.sent) == 1


def test_busy_with_other_customer(rig):
    svc, store, link, _ = rig
    start_find(svc, link)
    svc.handle_inbound(inbound("/find red cup", conv=CONV_B, sender=USR_B))
    assert replies(store, CONV_B) == ["The robot is busy with another request. Please try again in a few minutes."]


def test_new_find_allowed_after_terminal(rig):  # M3
    svc, _, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="CANCELLED")
    start_find(svc, link, "/find red cup")
    assert link.sent[-1]["target_description"] == "red cup"


def test_other_customer_cannot_see_or_cancel(rig):  # M11
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    svc.handle_inbound(inbound("/cancel", conv=CONV_B, sender=USR_B))
    svc.handle_inbound(inbound("/status", conv=CONV_B, sender=USR_B))
    assert all(f["type"] != "MISSION_CANCEL" for f in link.sent)
    assert replies(store, CONV_B) == ["You don't have an active mission.",
                                      "You don't have any missions yet. Send /find <object> to start one."]
    assert not svc.get(mid).cancel_requested


def test_same_space_other_sender_cannot_cancel(rig):  # M11 in a group chat
    svc, _, link, _ = rig
    start_find(svc, link)
    svc.handle_inbound(inbound("/cancel", sender=USR_B))
    assert all(f["type"] != "MISSION_CANCEL" for f in link.sent)


# --- robot availability ------------------------------------------------------------------------

def test_robot_offline_no_mission(rig):  # L5, E3
    svc, store, link, _ = rig
    link.connected = False
    svc.handle_inbound(inbound("/find mug"))
    assert store.active_mission(ROBOT) is None
    assert replies(store) == ["The robot is currently unavailable. Please try again later."]
    with pytest.raises(RobotUnavailable):
        svc.create_mission("mug")


def test_send_failure_fails_mission(rig):
    svc, store, link, _ = rig
    link.fail_sends = True
    svc.handle_inbound(inbound("/find mug"))
    assert store.active_mission(ROBOT) is None
    assert replies(store)[-1] == "The robot is currently unavailable. Please try again later."


def test_api_busy(rig):
    svc, _, link, _ = rig
    start_find(svc, link)
    with pytest.raises(RobotBusy):
        svc.create_mission("red cup")


# --- robot reports -----------------------------------------------------------------------------

def test_rejected_ack_fails_mission(rig):
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="REJECTED", reason="BUSY")
    assert svc.get(mid).state == p.MissionState.FAILED and svc.get(mid).fail_reason == "BUSY"
    assert replies(store)[-1] == "The robot couldn't start this search (it is busy)."


def test_illegal_jump_ignored(rig):  # M4
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="COMPLETED")
    assert svc.get(mid).state == p.MissionState.ACCEPTED
    assert "Search mission completed." not in replies(store)


def test_status_without_ack_implies_accept(rig):
    svc, _, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    assert svc.get(mid).state == p.MissionState.SEARCHING


def test_terminal_is_final(rig):  # M5
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="CANCELLED")
    n = len(replies(store))
    robot(svc, mid, status="SEARCHING")
    assert svc.get(mid).state == p.MissionState.CANCELLED and len(replies(store)) == n


def test_repeated_status_not_repeated_to_customer(rig):  # N5
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="SEARCHING", message="a")
    robot(svc, mid, status="SEARCHING", message="b")
    assert replies(store).count("The robot has started searching for: blue toolbox.") == 1
    assert svc.get(mid).last_message == "b"


def test_target_lost_then_searching_again(rig):
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    for s in ["SEARCHING", "TARGET_DETECTED", "SEARCHING"]:
        robot(svc, mid, status=s)
    assert replies(store)[-1] == "Lost sight of the target. Searching again for: blue toolbox."


def test_not_found_never_claims_detection(rig):  # E2
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    robot(svc, mid, status="FAILED", reason="NOT_FOUND")
    assert replies(store)[-1] == "I couldn't spot blue toolbox. The search has ended."
    assert not any("detected" in r.lower() for r in replies(store))


def test_not_found_appends_the_robots_summary(rig):
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    robot(svc, mid, status="FAILED", reason="NOT_FOUND", message="not found: blue toolbox; found: nothing")
    assert replies(store)[-1] == ("I couldn't spot blue toolbox. The search has ended. "
                                  "(not found: blue toolbox; found: nothing)")


@pytest.mark.parametrize("frame", ["garbage", json.dumps({"type": "MISSION_STATUS", "mission_id": str(uuid.uuid4()),
                                                          "status": "COMPLETED"}),
                                   json.dumps({"type": "MISSION_REQUEST"}),
                                   json.dumps({"type": "ERROR", "reason": "INVALID_FRAME"})])
def test_garbage_and_unknown_missions_ignored(rig, frame):  # L9
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    svc.on_robot_frame(frame)
    assert svc.get(mid).state == p.MissionState.PENDING and len(replies(store)) == 1


# --- cancel ------------------------------------------------------------------------------------

def test_cancel_waits_for_robot_confirmation(rig):  # M9, E5
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="SEARCHING")
    svc.handle_inbound(inbound("/cancel"))
    assert link.sent[-1] == {"type": "MISSION_CANCEL", "mission_id": mid}
    m = svc.get(mid)
    assert m.state == p.MissionState.SEARCHING and m.cancel_requested
    assert replies(store)[-1] == "Cancelling... I'll confirm once the robot has stopped."
    robot(svc, mid, status="CANCELLED")
    assert replies(store)[-1] == "Mission cancelled."


def test_cancel_api_errors(rig):  # M10
    svc, _, link, _ = rig
    with pytest.raises(MissionNotFound):
        svc.cancel(str(uuid.uuid4()))
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="REJECTED", reason="BUSY")
    with pytest.raises(AlreadyFinished):
        svc.cancel(mid)


def test_cancel_while_robot_offline(rig):
    svc, store, link, _ = rig
    start_find(svc, link)
    svc.on_robot_disconnected()
    link.connected = False
    svc.handle_inbound(inbound("/cancel"))
    assert replies(store)[-1] == "I can't reach the robot right now. It stops on its own when it loses contact."


# --- timeouts ----------------------------------------------------------------------------------

def test_mission_expiry(rig):  # M6, E8
    svc, store, link, clock = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="SEARCHING")
    clock.t += 599
    svc.tick()
    assert svc.get(mid).state == p.MissionState.SEARCHING
    clock.t += 1
    svc.tick()
    m = svc.get(mid)
    assert (m.state, m.fail_reason) == (p.MissionState.FAILED, "EXPIRED")
    assert link.sent[-1] == {"type": "MISSION_CANCEL", "mission_id": mid, "reason": "EXPIRED"}
    assert replies(store)[-1] == "The search for blue toolbox timed out."


def test_ack_timeout(rig):  # M7
    svc, store, link, clock = rig
    mid = start_find(svc, link)
    clock.t += 11
    svc.tick()
    assert svc.get(mid).fail_reason == "NO_ACK"
    assert replies(store)[-1] == "The robot didn't respond. Please try again."


def test_disconnect_then_grace_expiry(rig):  # L6, E6
    svc, store, link, clock = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    link.connected = False
    svc.on_robot_disconnected()
    assert replies(store)[-1] == "Lost contact with the robot. It will stop on its own for safety."
    clock.t += 14
    svc.tick()
    assert svc.get(mid).state == p.MissionState.ACCEPTED
    clock.t += 1
    svc.tick()
    assert svc.get(mid).fail_reason == "LINK_LOST"
    assert replies(store)[-1] == "Mission ended: the robot is offline."


def test_reconnect_within_grace(rig):  # L7, E7
    svc, store, link, clock = rig
    mid = start_find(svc, link)
    robot(svc, mid, "MISSION_ACK", status="ACCEPTED")
    robot(svc, mid, status="SEARCHING")
    link.connected = False
    svc.on_robot_disconnected()
    clock.t += 5
    link.connected = True
    svc.on_robot_connected()
    robot(svc, mid, status="SEARCHING")  # the agent re-reports its state on reconnect
    clock.t += 30
    svc.tick()
    assert svc.get(mid).state == p.MissionState.SEARCHING
    assert replies(store)[-2:] == ["Lost contact with the robot. It will stop on its own for safety.",
                                   "Reconnected to the robot."]


def test_robot_stopped_itself_after_link_loss(rig):
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    svc.on_robot_disconnected()
    svc.on_robot_connected()
    robot(svc, mid, status="CANCELLED", reason="LINK_LOST")
    assert replies(store)[-1] == "The robot stopped the search after losing contact with the server."


def test_restart_recovers_missions(tmp_path):  # S6
    clock, link = Clock(), FakeLink()
    path = str(tmp_path / "d.db")
    svc = MissionService(Store(path), link, robot_id=ROBOT, clock=clock, link_grace_s=15)
    mid = start_find(svc, link)
    svc2 = MissionService(Store(path), FakeLink(connected=False), robot_id=ROBOT, clock=clock, link_grace_s=15)
    assert svc2.get(mid).state == p.MissionState.PENDING
    clock.t += 16
    svc2.tick()
    assert svc2.get(mid).state == p.MissionState.FAILED


# --- other commands ----------------------------------------------------------------------------

def test_start_help_and_status_texts(rig):  # N3
    svc, store, link, _ = rig
    svc.handle_inbound(inbound("/start"))
    svc.handle_inbound(inbound("/help"))
    r = replies(store)
    assert r[0].startswith("Hello! I'm your autonomous robot assistant. Tell me what object you want me to find.")
    for cmd in ["/start", "/find", "/status", "/cancel", "/help"]:
        assert cmd in r[1]
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    svc.handle_inbound(inbound("/status"))
    assert replies(store)[-1] == "Current mission: searching for blue toolbox."


def test_invalid_input_never_reaches_robot(rig):  # E10
    svc, store, link, _ = rig
    for t in ["/find", "/find ; reboot", None, "hello", "/dance"]:
        svc.handle_inbound(inbound(t))
    assert link.sent == []
    r = replies(store)
    assert r[0].startswith("Please tell me what object to look for")
    assert r[2] == "I can only read text messages. Send /find <object>, for example: /find blue toolbox"


def test_rate_limit(rig):  # S5
    svc, store, link, clock = rig
    for i in range(5):
        mid = start_find(svc, link, f"/find item {i}")
        robot(svc, mid, status="CANCELLED")
    svc.handle_inbound(inbound("/find item 6"))
    assert replies(store)[-1] == "You're sending requests too quickly. Please wait a minute and try again."
    clock.t += 61
    start_find(svc, link, "/find item 7")
    assert link.sent[-1]["target_description"] == "item 7"


def test_replies_go_to_the_requesting_space(rig):  # N4
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    for o in store.list_outbox():
        assert o.conversation == Conversation("photon", "pho_spc_01jaaaaaaaaaaaaaaaaaaaaaaa", "pho_usr_bot")


def test_reply_keys_are_deterministic(rig):  # D7
    svc, store, link, _ = rig
    mid = start_find(svc, link)
    robot(svc, mid, status="SEARCHING")
    keys = [o.key for o in store.list_outbox()]
    assert keys[1] == f"{mid}:SEARCHING"
    assert len(set(keys)) == len(keys)
