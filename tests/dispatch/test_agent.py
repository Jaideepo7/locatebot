import json
from datetime import datetime, timedelta, timezone

import pytest

from locatebot.dispatch import protocol as p
from locatebot.dispatch.agent import JetsonAgent
from locatebot.dispatch.protocol import MissionState as S
from locatebot.dispatch.sink import MissionRejected

T0 = datetime(2026, 11, 7, 12, 0, 0, tzinfo=timezone.utc)
MID = "3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70"
MID2 = "9b1d2e3f-4a5b-4c6d-8e7f-0a1b2c3d4e5f"


class Clock:
    def __init__(self):
        self.t = T0.timestamp()

    def __call__(self):
        return self.t


class FakeSink:
    def __init__(self, reject=None):
        self.started, self.cancelled, self.link_lost = [], [], []
        self.reporters = {}
        self.reject = reject

    def start(self, mission_id, target_description, report):
        if self.reject:
            raise MissionRejected(self.reject)
        self.started.append((mission_id, target_description))
        self.reporters[mission_id] = report

    def cancel(self, mission_id):
        self.cancelled.append(mission_id)

    def on_link_lost(self, mission_id):
        self.link_lost.append(mission_id)


def req(mid=MID, issued=T0, ttl=600, target="blue toolbox"):
    return json.dumps({"type": "MISSION_REQUEST", "mission_id": mid, "mission_type": "FIND_OBJECT",
                       "target_description": target, "issued_at": p.format_ts(issued),
                       "expires_at": p.format_ts(issued + timedelta(seconds=ttl))})


def cancel(mid=MID):
    return json.dumps({"type": "MISSION_CANCEL", "mission_id": mid})


@pytest.fixture
def rig():
    clock, sink, sent = Clock(), FakeSink(), []
    agent = JetsonAgent(sink, send=sent.append, clock=clock)
    agent.on_connected()
    frames = lambda: [json.loads(f) for f in sent]
    return agent, sink, frames, clock


def test_valid_request_is_acked_and_started_once(rig):  # A1
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    assert frames() == [{"type": "MISSION_ACK", "mission_id": MID, "status": "ACCEPTED"}]
    assert sink.started == [(MID, "blue toolbox")]
    assert agent.active_mission_id == MID


def test_expired_request_rejected(rig):  # A2
    agent, sink, frames, clock = rig
    clock.t += 601
    agent.on_frame(req())
    assert frames() == [{"type": "MISSION_ACK", "mission_id": MID, "status": "REJECTED", "reason": "EXPIRED"}]
    assert sink.started == []


def test_future_issued_request_rejected(rig):  # A3
    agent, sink, frames, _ = rig
    agent.on_frame(req(issued=T0 + timedelta(seconds=31)))
    assert frames()[-1]["reason"] == "CLOCK_SKEW"
    assert sink.started == []


def test_expires_before_issued_rejected(rig):
    agent, sink, frames, _ = rig
    agent.on_frame(req(ttl=-5))
    assert frames()[-1]["status"] == "REJECTED"
    assert sink.started == []


def test_duplicate_mission_id_not_started_twice(rig):  # A4
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    agent.on_frame(req())
    assert len(sink.started) == 1
    assert [f["status"] for f in frames()] == ["ACCEPTED", "ACCEPTED"]


def test_duplicate_of_rejected_is_rejected_again(rig):
    agent, sink, frames, clock = rig
    clock.t += 601
    agent.on_frame(req())
    clock.t -= 601
    agent.on_frame(req())  # same id replayed later: still rejected, never started
    assert sink.started == []
    assert [f["status"] for f in frames()] == ["REJECTED", "REJECTED"]


def test_second_mission_while_busy_rejected(rig):  # A5
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    agent.on_frame(req(mid=MID2, target="red cup"))
    assert frames()[-1] == {"type": "MISSION_ACK", "mission_id": MID2, "status": "REJECTED", "reason": "BUSY"}
    assert sink.started == [(MID, "blue toolbox")]


def test_sink_can_reject(rig):
    agent, sink, frames, _ = rig
    sink.reject = "PERCEPTION_UNAVAILABLE"
    agent.on_frame(req())
    assert frames()[-1]["reason"] == "PERCEPTION_UNAVAILABLE"
    assert agent.active_mission_id is None


@pytest.mark.parametrize("bad", ["garbage", "{}", json.dumps({"type": "DRIVE", "speed": 9}),
                                 req(target="; reboot"), "x" * 10000])
def test_malformed_frames_do_not_crash_or_start(rig, bad):  # A6, J4
    agent, sink, frames, _ = rig
    agent.on_frame(bad)
    assert sink.started == []
    assert frames() and frames()[-1]["type"] == "ERROR"
    agent.on_frame(req())  # still works afterwards
    assert sink.started == [(MID, "blue toolbox")]


def test_cancel_is_forwarded_and_confirmed_only_by_sink(rig):  # A7
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    agent.on_frame(cancel())
    assert sink.cancelled == [MID]
    assert all(f.get("status") != "CANCELLED" for f in frames())
    sink.reporters[MID](S.CANCELLED, "Stopped")
    assert frames()[-1] == {"type": "MISSION_STATUS", "mission_id": MID, "status": "CANCELLED", "message": "Stopped"}
    assert agent.active_mission_id is None


def test_cancel_for_unknown_mission_ignored(rig):  # A8
    agent, sink, frames, _ = rig
    agent.on_frame(cancel(MID2))
    assert sink.cancelled == []


def test_sink_statuses_relayed_in_order(rig):  # A9, A10
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    r = sink.reporters[MID]
    r(S.SEARCHING, "Searching for blue toolbox")
    r(S.TARGET_DETECTED, "Found candidate")
    r(S.APPROACHING)
    r(S.COMPLETED)
    assert [f["status"] for f in frames()] == ["ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING", "COMPLETED"]
    assert frames()[1]["message"] == "Searching for blue toolbox"
    assert agent.active_mission_id is None


def test_illegal_or_stale_sink_status_dropped(rig):
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    r = sink.reporters[MID]
    r(S.COMPLETED)  # ACCEPTED -> COMPLETED skips the search: dropped
    assert [f["status"] for f in frames()] == ["ACCEPTED"]
    r(S.SEARCHING)
    r(S.CANCELLED)
    r(S.SEARCHING)  # after terminal: dropped
    assert [f["status"] for f in frames()] == ["ACCEPTED", "SEARCHING", "CANCELLED"]


def test_reporter_rejects_non_state_values(rig):
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    with pytest.raises(ValueError):
        sink.reporters[MID]("DONE")


def test_link_lost_notifies_sink_once_and_does_not_resume(rig):  # A11
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    sink.reporters[MID](S.SEARCHING)
    agent.on_disconnected()
    agent.on_disconnected()
    assert sink.link_lost == [MID]
    assert sink.started == [(MID, "blue toolbox")]


def test_statuses_while_disconnected_are_reported_on_reconnect(rig):  # A11, L7
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    sink.reporters[MID](S.SEARCHING)
    agent.on_disconnected()
    n = len(frames())
    sink.reporters[MID](S.CANCELLED, "Stopped: link lost", "LINK_LOST")
    assert len(frames()) == n  # nothing sent while offline
    agent.on_connected()
    assert frames()[-1] == {"type": "MISSION_STATUS", "mission_id": MID, "status": "CANCELLED",
                            "message": "Stopped: link lost", "reason": "LINK_LOST"}


def test_reconnect_rereports_current_state(rig):
    agent, sink, frames, _ = rig
    agent.on_frame(req())
    sink.reporters[MID](S.SEARCHING, "Searching")
    agent.on_disconnected()
    agent.on_connected()
    assert frames()[-1]["status"] == "SEARCHING"
    assert len(sink.link_lost) == 1


def test_mission_expiry_on_jetson(rig):  # A12
    agent, sink, frames, clock = rig
    agent.on_frame(req(ttl=60))
    sink.reporters[MID](S.SEARCHING)
    clock.t += 59
    agent.tick()
    assert sink.cancelled == []
    clock.t += 2
    agent.tick()
    assert sink.cancelled == [MID]
    assert frames()[-1] == {"type": "MISSION_STATUS", "mission_id": MID, "status": "FAILED",
                            "message": "Mission expired", "reason": "EXPIRED"}
    assert agent.active_mission_id is None
    sink.reporters[MID](S.CANCELLED)  # late sink confirmation is dropped
    assert frames()[-1]["status"] == "FAILED"


def test_seen_cache_is_bounded(rig):  # A13
    agent, sink, frames, _ = rig
    agent = JetsonAgent(sink, send=lambda f: None, clock=rig[3], seen_capacity=3)
    agent.on_connected()
    import uuid
    for _ in range(10):
        mid = str(uuid.uuid4())
        agent.on_frame(req(mid=mid))
        sink.reporters[mid](S.CANCELLED)
    assert len(agent._seen) <= 3


def test_send_failure_does_not_break_agent():
    sink = FakeSink()

    def boom(_):
        raise ConnectionError("closed")

    agent = JetsonAgent(sink, send=boom, clock=Clock())
    agent.on_connected()
    agent.on_frame(req())
    assert sink.started == [(MID, "blue toolbox")]
