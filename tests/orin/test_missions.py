"""MissionBridge unit tests with a fake runtime: rejection rules, status mapping, cancel, link loss."""
import math
import threading
import uuid
from types import SimpleNamespace

import pytest

from locatebot.lidar.types import Pose2D
from locatebot.link.messages import MissionCancel, MissionRequest, MissionStatus, encode
from locatebot.mission.types import ItemView, SkillCall, SkillName, SkillState, SkillStatus, WorldState
from locatebot.orin import missions as mb
from locatebot.orin.missions import MissionBridge

T_WALL = 1_794_000_000.0


class FakeRuntime:
    def __init__(self, brain=None):
        self.brain = brain
        self.calls = []
        self.goal = ""
        self.goals = []
        self.mapper = SimpleNamespace(latest_scan=None)

    def set_goal(self, text, source="operator"):
        self.goal = text
        self.goals.append((text, source))
        return text

    def clear_goal(self):
        self.goal = ""
        self.goals.append(("", "clear"))

    def submit_operator(self, call):
        self.calls.append(call)


class FakeBrain:
    def __init__(self, paused=False):
        self.paused = paused
        self.resumed = 0

    def status(self):
        return {"paused": self.paused}

    def resume(self):
        self.paused = False
        self.resumed += 1


def item(name="marker", confirmed=True, x=1.0, y=2.0, age=0.0, sightings=3):
    return ItemView(name, confirmed, sightings, x, y, 0.1, math.hypot(x, y), 0.3, age)


def world(stamp, state=SkillState.IDLE, call=None, detail="", items=()):
    return WorldState(stamp_s=stamp, pose=Pose2D(0, 0, 0), sector_clearance_m=(1.0,) * 8, explored_frac=0.5,
                      items=tuple(items), skill=SkillStatus(state, call, detail))


class Rig:
    def __init__(self, brain=None):
        self.rt = FakeRuntime(brain)
        self.wall = [T_WALL]
        self.scan_s = [0.0]
        self.sent = []
        self.bridge = MissionBridge(self.rt, self.sent.append, clock=lambda: self.wall[0],
                                    last_scan_s=lambda: self.scan_s[0])
        self.t = 0.0

    def tick(self, state=SkillState.IDLE, call=None, detail="", items=(), dt=0.1):
        self.t += dt
        self.scan_s[0] = max(self.scan_s[0], self.t)  # scans keep arriving unless a test freezes them
        n = len(self.sent)
        self.bridge.tick(self.t, world(self.t, state, call, detail, items))
        return self.sent[n:]

    def request(self, target="marker", mid=None, issued=None, ttl=600.0):
        mid = mid or str(uuid.uuid4())
        issued = self.wall[0] if issued is None else issued
        self.bridge.on_request(MissionRequest(mid, target, issued, issued + ttl))
        return mid

    def states(self):
        return [s.state for s in self.sent]


def accepted(rig, target="marker"):
    mid = rig.request(target)
    out = rig.tick()
    assert [s.state for s in out] == ["ACCEPTED"]
    return mid


def test_transitions_match_the_pi_protocol():
    from locatebot.remote.protocol import TRANSITIONS
    assert {k.value: {v.value for v in vs} for k, vs in TRANSITIONS.items()} == mb.TRANSITIONS


def test_routine_happy_path_one_status_per_transition():
    r = Rig()
    mid = accepted(r)
    assert r.rt.goals[-1] == ("find marker", "mission")
    search = r.rt.calls[-1]
    assert search.name == SkillName.SEARCH and tuple(search.args["items"]) == ("marker",)
    assert r.tick(SkillState.RUNNING, search, "exploring")[0].state == "SEARCHING"
    assert r.tick(SkillState.RUNNING, search, "exploring") == []
    assert r.tick(SkillState.RUNNING, search, items=[item(confirmed=False)]) == []  # unconfirmed: not yet
    det = r.tick(SkillState.RUNNING, search, items=[item()])
    assert [s.state for s in det] == ["TARGET_DETECTED"] and "(1.0, 2.0)" in det[0].message
    r.tick(SkillState.SUCCEEDED, search, "found all: marker", items=[item()])
    go = r.rt.calls[-1]
    assert go.name == SkillName.GO_TO and dict(go.args) == {"item": "marker"}
    assert [s.state for s in r.tick(SkillState.RUNNING, go, "driving to marker", items=[item()])] == ["APPROACHING"]
    done = r.tick(SkillState.SUCCEEDED, go, "reached marker", items=[item()])
    assert [s.state for s in done] == ["COMPLETED"] and "arrived at marker" in done[0].message
    assert r.rt.goal == ""
    for _ in range(5):
        assert r.tick(SkillState.SUCCEEDED, go, "reached marker", items=[item()]) == []
    assert r.states() == ["ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING", "COMPLETED"]
    assert all(s.mission_id == mid for s in r.sent)
    for s in r.sent:
        encode(s)  # every status is a valid link message
    assert r.bridge.status()["active"] is None


@pytest.mark.parametrize("issued_off,ttl,reason", [(-700.0, 600.0, "EXPIRED"), (6.0, 600.0, "CLOCK_SKEW"),
                                                   (0.0, -1.0, "INVALID_EXPIRY")])
def test_rejections(issued_off, ttl, reason):
    r = Rig()
    r.request(issued=T_WALL + issued_off, ttl=ttl)
    out = r.tick()
    assert [(s.state, s.reason) for s in out] == [("FAILED", reason)]
    assert out[0].message.startswith("rejected:")
    assert r.rt.calls == [] and r.rt.goals == []


def test_small_clock_skew_is_accepted():
    r = Rig()
    r.request(issued=T_WALL + 4.0)
    assert r.states() == [] and [s.state for s in r.tick()] == ["ACCEPTED"]


def test_second_concurrent_mission_is_busy_and_first_continues():
    r = Rig()
    first = accepted(r)
    second = r.request("cup")
    out = r.tick()
    assert [(s.mission_id, s.state, s.reason) for s in out] == [(second, "FAILED", "BUSY")]
    assert r.bridge.status()["active"]["mission_id"] == first


def test_duplicate_request_is_answered_again_never_started_twice():
    r = Rig()
    mid = accepted(r)
    n_calls = len(r.rt.calls)
    r.bridge.on_request(MissionRequest(mid, "marker", T_WALL, T_WALL + 600))
    out = r.tick()
    assert [(s.mission_id, s.state) for s in out] == [(mid, "ACCEPTED")]
    assert len(r.rt.calls) == n_calls and r.bridge.status()["duplicates"] == 1


def test_not_found_carries_the_found_missing_text():
    r = Rig()
    accepted(r, "unicorn")
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search)
    detail = "explored 85% and checked the room; not found: unicorn; found: nothing"
    out = r.tick(SkillState.FAILED, search, detail)
    assert [(s.state, s.reason, s.message) for s in out] == [("FAILED", "NOT_FOUND", detail)]
    assert r.rt.goal == "" and r.bridge.status()["active"] is None


def _to_approaching(r):
    accepted(r)
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search, items=[item()])
    r.tick(SkillState.SUCCEEDED, search, "found all: marker", items=[item()])
    go = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, go, items=[item()])
    assert r.states()[-1] == "APPROACHING"
    return go


def test_no_path():
    r = Rig()
    go = _to_approaching(r)
    out = r.tick(SkillState.FAILED, go, "no drivable path to marker", items=[item()])
    assert [(s.state, s.reason) for s in out] == [("FAILED", "NO_PATH")]


def test_internal_error_is_robot_fault_and_stops():
    r = Rig()
    accepted(r)
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search)
    out = r.tick(SkillState.FAILED, search, "internal error: boom")
    assert [(s.state, s.reason) for s in out] == [("FAILED", "ROBOT_FAULT")]
    assert r.rt.calls[-1].name == SkillName.STOP


def test_mission_ttl_timeout_stops_the_robot():
    r = Rig()
    mid = r.request(ttl=30.0)
    r.tick()
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search)
    r.wall[0] += 31.0
    out = r.tick(SkillState.RUNNING, search)
    assert [(s.mission_id, s.state, s.reason) for s in out] == [(mid, "FAILED", "TIMEOUT")]
    assert r.rt.calls[-1].name == SkillName.STOP and r.rt.goal == ""


def test_target_lost_while_approaching_goes_back_to_searching():
    r = Rig()
    go = _to_approaching(r)
    assert r.tick(SkillState.FAILED, go, "haven't seen marker yet: search for it first") == []
    again = r.rt.calls[-1]
    assert again.name == SkillName.SEARCH
    assert [s.state for s in r.tick(SkillState.RUNNING, again, items=[item(age=5.0)])] == ["SEARCHING"]
    # the old sighting (before the loss) does not count as a new detection; a fresh one does
    assert r.tick(SkillState.RUNNING, again, items=[item(age=5.0)]) == []
    assert [s.state for s in r.tick(SkillState.RUNNING, again, items=[item(age=0.0)])] == ["TARGET_DETECTED"]


def test_cancel_reports_cancelled_only_once_stopped():
    r = Rig()
    mid = accepted(r)
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search)
    r.bridge.on_cancel(MissionCancel(mid, ""))
    assert r.tick(SkillState.RUNNING, search) == []
    stop = r.rt.calls[-1]
    assert stop.name == SkillName.STOP and r.rt.goal == ""
    assert r.tick(SkillState.RUNNING, stop) == []
    out = r.tick(SkillState.SUCCEEDED, stop, "stopped")
    assert [s.state for s in out] == ["CANCELLED"]
    assert r.bridge.status()["active"] is None
    r.bridge.on_cancel(MissionCancel(mid, ""))  # again: the final state is re-sent
    assert [s.state for s in r.tick(SkillState.SUCCEEDED, stop)] == ["CANCELLED"]


def test_cancel_for_another_mission_is_ignored():
    r = Rig()
    accepted(r)
    n = len(r.rt.calls)
    r.bridge.on_cancel(MissionCancel(str(uuid.uuid4()), ""))
    assert r.tick() == [] and len(r.rt.calls) == n


def test_link_loss_stops_keeps_state_and_rereports_on_reconnect():
    r = Rig()
    accepted(r)
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search)
    assert r.states()[-1] == "SEARCHING"
    frozen = r.scan_s[0]
    r.bridge._last_scan_s = lambda: frozen  # the Pi goes quiet
    for _ in range(25):
        assert r.tick(SkillState.RUNNING, search) == []
    stop = r.rt.calls[-1]
    assert stop.name == SkillName.STOP and "link lost" in stop.reason
    assert r.bridge.status()["active"]["link_down"] is True
    assert [s.state for s in r.tick(SkillState.SUCCEEDED, stop)] == []  # our own stop: no failure
    r.bridge._last_scan_s = lambda: r.scan_s[0]
    out = r.tick(SkillState.SUCCEEDED, stop)
    assert [s.state for s in out] == ["SEARCHING"]  # re-reported
    resumed = r.rt.calls[-1]
    assert resumed.name == SkillName.SEARCH and tuple(resumed.args["items"]) == ("marker",)
    assert r.bridge.status()["link_losses"] == 1


def test_someone_elses_stop_fails_the_mission_as_stopped():
    r = Rig()
    accepted(r)
    search = r.rt.calls[-1]
    r.tick(SkillState.RUNNING, search)
    op_stop = SkillCall(SkillName.STOP, {}, reason="operator", source="operator")
    r.tick(SkillState.RUNNING, op_stop)
    out = r.tick(SkillState.SUCCEEDED, op_stop, "stopped")
    assert [(s.state, s.reason) for s in out] == [("FAILED", "STOPPED")]


def test_previous_terminal_skill_is_ignored():
    r = Rig()
    old = SkillCall(SkillName.SEARCH, {"items": ("marker",)}, reason="old")
    r.request()
    r.bridge.tick(1.0, world(1.0, SkillState.FAILED, old, "not found: marker"))
    assert [s.state for s in r.sent] == ["ACCEPTED"]
    r.bridge.tick(1.1, world(1.1, SkillState.FAILED, old, "not found: marker"))
    assert [s.state for s in r.sent] == ["ACCEPTED"]


def test_brain_mode_sets_goal_resumes_and_maps_llm_skills():
    brain = FakeBrain(paused=True)
    r = Rig(brain)
    accepted(r)
    assert r.rt.calls == []  # the LLM acts, not a routine
    assert brain.resumed == 1 and r.rt.goal == "find marker"
    # the target was already on the map: go_to straight away fills in the true states
    go = SkillCall(SkillName.GO_TO, {"item": "Marker"}, reason="it is on the map", source="llm")
    out = r.tick(SkillState.RUNNING, go, items=[item()])
    assert [s.state for s in out] == ["SEARCHING", "TARGET_DETECTED", "APPROACHING"]
    assert [s.state for s in r.tick(SkillState.SUCCEEDED, go, "reached marker", items=[item()])] == ["COMPLETED"]


def test_brain_mode_look_around_is_searching():
    r = Rig(FakeBrain())
    accepted(r)
    look = SkillCall(SkillName.LOOK_AROUND, {}, source="llm")
    assert [s.state for s in r.tick(SkillState.RUNNING, look)] == ["SEARCHING"]


def test_inputs_from_other_threads():
    r = Rig()
    ids = []

    def send(k):
        for i in range(50):
            ids.append(r.request(f"thing {k} {i}"))

    threads = [threading.Thread(target=send, args=(k,)) for k in range(4)]
    for t in threads:
        t.start()
    for _ in range(20):
        r.tick()
    for t in threads:
        t.join()
    r.tick()
    assert len(r.sent) == 200
    assert r.states().count("ACCEPTED") == 1 and r.states().count("FAILED") == 199


def test_statuses_are_link_messages():
    st = MissionStatus("m", "FAILED", "x", "NOT_FOUND")
    assert encode(st)[0] == b"mission_status"
