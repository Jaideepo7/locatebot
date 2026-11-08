"""ZmqRobotLink over 127.0.0.1 ZeroMQ with a fake Orin (publisher + subscriber)."""
import json
import time
import uuid
from datetime import datetime, timezone

import pytest

pytest.importorskip("zmq")
from locatebot.link.messages import MissionCancel, MissionRequest, MissionStatus, OrinStatus  # noqa: E402
from locatebot.link.transport import Publisher, Subscriber  # noqa: E402
from locatebot.remote import protocol as p  # noqa: E402
from locatebot.remote.robot_zmq import ZmqRobotLink, to_link_message  # noqa: E402

LOCAL = "tcp://127.0.0.1:*"


class Listener:
    def __init__(self):
        self.frames, self.events = [], []

    def on_robot_frame(self, frame):
        self.frames.append(json.loads(frame))

    def on_robot_connected(self):
        self.events.append("connected")

    def on_robot_disconnected(self):
        self.events.append("disconnected")


def wait(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def iso(t):
    return p.format_ts(datetime.fromtimestamp(t, timezone.utc))


def request(mid=None, target="blue toolbox", issued=1_794_000_000.0):
    return {"type": "MISSION_REQUEST", "mission_id": mid or str(uuid.uuid4()), "mission_type": "FIND_OBJECT",
            "target_description": target, "issued_at": iso(issued), "expires_at": iso(issued + 600)}


def status(boot="b1"):
    return OrinStatus(time.monotonic(), False, None, "idle", None, None, boot)


@pytest.fixture
def rig():
    orin_pub = Publisher(LOCAL)
    link = ZmqRobotLink(orin_pub.endpoint, pub_endpoint=LOCAL, connected_timeout_s=0.4, poll_s=0.01)
    lst = Listener()
    link.listener = lst
    orin_sub = Subscriber(link.pub_endpoint, ["mission_request", "mission_cancel"])
    link.start()
    yield link, lst, orin_pub, orin_sub
    link.stop()
    orin_sub.close()
    orin_pub.close()


def connect(link, orin_pub):
    assert wait(lambda: (orin_pub.send(status()), link.connected)[1]), "never heard orin_status"
    time.sleep(0.2)  # let the Orin's SUB subscription reach the Pi's PUB (slow joiner)


def recv(sub, timeout=3.0):
    return sub.recv(timeout)


def test_to_link_message_encoding():
    req = request(issued=1_794_000_000.5)
    m = to_link_message(req)
    assert isinstance(m, MissionRequest) and m.target == "blue toolbox"
    assert m.issued_s == pytest.approx(1_794_000_000.5) and m.expires_s == pytest.approx(1_794_000_600.5)
    c = to_link_message({"type": "MISSION_CANCEL", "mission_id": req["mission_id"], "reason": "EXPIRED"})
    assert c == MissionCancel(req["mission_id"], "EXPIRED")
    with pytest.raises(ValueError):
        to_link_message({"type": "MISSION_ACK", "mission_id": req["mission_id"], "status": "ACCEPTED"})
    with pytest.raises(ValueError):
        to_link_message({"type": "MISSION_REQUEST", "mission_id": "x"})


def test_not_connected_until_orin_status_then_request_and_cancel_reach_the_orin(rig):
    link, lst, orin_pub, orin_sub = rig
    assert not link.connected and link.send(request()) is False
    connect(link, orin_pub)
    assert lst.events == ["connected"]
    req = request()
    assert link.send(req) is True
    got = recv(orin_sub)
    assert isinstance(got, MissionRequest) and got.mission_id == req["mission_id"] and got.target == "blue toolbox"
    assert link.send({"type": "MISSION_CANCEL", "mission_id": req["mission_id"]})
    assert recv(orin_sub) == MissionCancel(req["mission_id"], "")


def test_statuses_become_service_frames(rig):
    link, lst, orin_pub, _ = rig
    connect(link, orin_pub)
    a, b = str(uuid.uuid4()), str(uuid.uuid4())
    for st in [MissionStatus(a, "ACCEPTED", "searching for x", ""),
               MissionStatus(a, "SEARCHING", "searching for x", ""),
               MissionStatus(a, "FAILED", "not found: x; found: nothing", "NOT_FOUND"),
               MissionStatus(b, "FAILED", "rejected: busy", "BUSY")]:
        orin_pub.send(st)
    assert wait(lambda: len(lst.frames) == 4)
    assert lst.frames == [
        {"type": "MISSION_ACK", "mission_id": a, "status": "ACCEPTED"},
        {"type": "MISSION_STATUS", "mission_id": a, "status": "SEARCHING", "message": "searching for x"},
        {"type": "MISSION_STATUS", "mission_id": a, "status": "FAILED", "message": "not found: x; found: nothing",
         "reason": "NOT_FOUND"},
        {"type": "MISSION_ACK", "mission_id": b, "status": "REJECTED", "reason": "BUSY"},
    ]
    for f in lst.frames:
        p.validate(f)


def test_malformed_messages_are_dropped(rig):
    link, lst, orin_pub, _ = rig
    connect(link, orin_pub)
    orin_pub._sock.send_multipart([b"mission_status", b"{not json", b""])
    orin_pub._sock.send_multipart([b"mission_status", b'{"v":1,"type":"mission_status","mission_id":"x"}', b""])
    orin_pub.send(MissionStatus("not-a-uuid", "SEARCHING", "", ""))           # Pi protocol needs a UUID
    orin_pub.send(MissionStatus(str(uuid.uuid4()), "SEARCHING", "", "lower case"))  # reason is dropped
    good = str(uuid.uuid4())
    orin_pub.send(MissionStatus(good, "ACCEPTED", "", ""))
    assert wait(lambda: any(f["mission_id"] == good for f in lst.frames))
    assert link.stats["bad_status"] == 1
    assert [f["mission_id"] for f in lst.frames][-1] == good
    assert "reason" not in lst.frames[0]
    assert link.connected


def test_disconnect_when_orin_status_stops_and_reconnect(rig):
    link, lst, orin_pub, _ = rig
    connect(link, orin_pub)
    assert wait(lambda: not link.connected, timeout=2.0)
    assert lst.events == ["connected", "disconnected"]
    assert link.send(request()) is False
    connect(link, orin_pub)
    assert lst.events == ["connected", "disconnected", "connected"]


def test_orin_restart_is_counted(rig):
    link, lst, orin_pub, _ = rig
    connect(link, orin_pub)
    assert wait(lambda: (orin_pub.send(status("b2")), link.stats["orin_restarts"] == 1)[1])


def test_injected_publisher_is_only_used_by_its_owner():
    orin_pub = Publisher(LOCAL)
    pi_pub = Publisher(LOCAL)  # the Pi process's own 5555 socket (shared)
    orin_sub = Subscriber(pi_pub.endpoint, ["mission_request"])
    link = ZmqRobotLink(orin_pub.endpoint, publisher=pi_pub, connected_timeout_s=1.0, poll_s=0.01)
    link.start()
    try:
        connect(link, orin_pub)
        req = request()
        assert link.send(req)
        time.sleep(0.1)
        assert orin_sub.recv(0.1) is None  # the link thread never sends on a shared socket
        assert link.flush_outgoing(pi_pub) == 1  # the owner thread does
        assert orin_sub.recv(2.0).mission_id == req["mission_id"]
    finally:
        link.stop()
        assert not pi_pub.closed  # not ours to close
        for s in (orin_sub, pi_pub, orin_pub):
            s.close()
