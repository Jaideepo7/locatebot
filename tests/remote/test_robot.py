"""FakeRobotLink: the scripted stand-in for the Orin (adapted from the branch's ScriptedSink tests)."""
import json
import time
import uuid
from datetime import datetime, timezone

from locatebot.remote import protocol as p
from locatebot.remote.protocol import MissionState as S
from locatebot.remote.robot import SCRIPTS, FakeRobotLink, RobotLink, Step

T0 = 1_794_000_000.0


class Clock:
    def __init__(self):
        self.t = T0

    def __call__(self):
        return self.t


class Listener:
    def __init__(self):
        self.frames, self.events = [], []

    def on_robot_frame(self, frame):
        self.frames.append(json.loads(frame))

    def on_robot_connected(self):
        self.events.append("connected")

    def on_robot_disconnected(self):
        self.events.append("disconnected")

    def states(self):
        return [f.get("status") for f in self.frames]


def iso(t):
    return p.format_ts(datetime.fromtimestamp(t, timezone.utc))


def request(clock, mid=None, issued=None, ttl=600, target="blue toolbox"):
    issued = clock() if issued is None else issued
    return {"type": "MISSION_REQUEST", "mission_id": mid or str(uuid.uuid4()), "mission_type": "FIND_OBJECT",
            "target_description": target, "issued_at": iso(issued), "expires_at": iso(issued + ttl)}


def rig(script="happy", **kw):
    clock, lst = Clock(), Listener()
    link = FakeRobotLink(script, clock=clock, **kw)
    link.listener = lst
    return link, lst, clock


def run(link, clock, seconds, step=0.5):
    for _ in range(int(seconds / step)):
        clock.t += step
        link.poll()


def test_is_a_robot_link():
    link: RobotLink = FakeRobotLink()
    assert link.connected and hasattr(link, "send")


def test_happy_script_plays_full_mission():
    link, lst, clock = rig("happy")
    assert link.send(request(clock))
    link.poll()
    assert lst.frames[0]["type"] == "MISSION_ACK" and lst.frames[0]["status"] == "ACCEPTED"
    run(link, clock, 10)
    assert lst.states()[1:] == ["SEARCHING", "TARGET_DETECTED", "APPROACHING", "COMPLETED"]
    assert lst.frames[1]["message"] == "Searching for blue toolbox"
    assert link.active_mission_id is None
    for f in lst.frames:
        p.validate(f)  # everything the fake sends is a valid protocol frame


def test_steps_wait_for_their_time():
    link, lst, clock = rig("happy")
    link.send(request(clock))
    run(link, clock, 0.5)
    assert lst.states() == ["ACCEPTED"]


def test_not_found_script_fails_without_detection():
    link, lst, clock = rig("not_found")
    link.send(request(clock))
    run(link, clock, 10)
    assert lst.states()[1:] == ["SEARCHING", "FAILED"]
    assert lst.frames[-1]["reason"] == "NOT_FOUND"


def test_reject_script_refuses_start():
    link, lst, clock = rig("reject")
    link.send(request(clock))
    link.poll()
    assert lst.frames == [{"type": "MISSION_ACK", "mission_id": lst.frames[0]["mission_id"],
                           "status": "REJECTED", "reason": "BUSY"}]


def test_stall_script_stops_after_searching():
    link, lst, clock = rig("stall")
    link.send(request(clock))
    run(link, clock, 60)
    assert lst.states()[1:] == ["SEARCHING"]


def test_cancel_confirms_after_delay_and_stops_script():
    link, lst, clock = rig("stall")
    req = request(clock)
    link.send(req)
    run(link, clock, 2)
    link.send({"type": "MISSION_CANCEL", "mission_id": req["mission_id"]})
    link.poll()
    assert lst.states()[-1] == "SEARCHING"  # the robot confirms only once stopped
    run(link, clock, 1)
    assert lst.states()[-1] == "CANCELLED"
    n = len(lst.frames)
    run(link, clock, 20)
    assert len(lst.frames) == n


def test_cancel_for_other_mission_ignored():
    link, lst, clock = rig("stall")
    link.send(request(clock))
    run(link, clock, 2)
    link.send({"type": "MISSION_CANCEL", "mission_id": str(uuid.uuid4())})
    run(link, clock, 2)
    assert lst.states()[-1] == "SEARCHING"


def test_rejects_busy_expired_future_and_replays_duplicates():
    link, lst, clock = rig("stall")
    first = request(clock)
    link.send(first)
    link.poll()
    link.send(request(clock))  # concurrent
    link.send(request(clock, issued=clock() - 700))  # expired
    link.send(request(clock, issued=clock() + 120))  # future-dated beyond skew
    link.send(first)  # duplicate: same answer, never started twice
    link.poll()
    reasons = [f.get("reason") for f in lst.frames if f["type"] == "MISSION_ACK"]
    assert reasons == [None, "BUSY", "EXPIRED", "CLOCK_SKEW", None]
    assert link.active_mission_id == first["mission_id"]


def test_link_lost_stops_and_reports_on_reconnect():
    link, lst, clock = rig("happy")
    link.send(request(clock))
    run(link, clock, 1.5)
    link.drop()
    assert not link.connected and link.active_mission_id is None
    assert lst.events == ["disconnected"]
    assert not link.send({"type": "MISSION_CANCEL", "mission_id": str(uuid.uuid4())})
    run(link, clock, 20)
    assert lst.states()[-1] == "SEARCHING"  # nothing delivered while offline
    link.connect()
    assert lst.events == ["disconnected", "connected"]
    assert lst.frames[-1]["status"] == "CANCELLED" and lst.frames[-1]["reason"] == "LINK_LOST"


def test_expiry_fails_the_mission():
    link, lst, clock = rig("stall")
    link.send(request(clock, ttl=5))
    run(link, clock, 6)
    assert lst.frames[-1]["status"] == "FAILED" and lst.frames[-1]["reason"] == "EXPIRED"


def test_silent_robot_never_answers():
    link, lst, clock = rig("happy", silent=True)
    assert link.send(request(clock))
    run(link, clock, 10)
    assert lst.frames == [] and len(link.received) == 1


def test_custom_script_and_thread():
    script = [Step(0.05, S.SEARCHING, "looking"), Step(0.1, S.FAILED, "nope", "NOT_FOUND")]
    lst = Listener()
    link = FakeRobotLink(script)
    link.listener = lst
    link.start(period_s=0.01)
    try:
        link.send(request(time.time))
        deadline = time.time() + 2
        while time.time() < deadline and "FAILED" not in lst.states():
            time.sleep(0.01)
    finally:
        link.stop()
    assert lst.states() == ["ACCEPTED", "SEARCHING", "FAILED"]


def test_scripts_are_legal_transition_chains():
    for name, steps in SCRIPTS.items():
        if isinstance(steps, str):
            continue
        state = S.ACCEPTED
        for s in steps:
            assert p.can_transition(state, s.state), (name, state, s.state)
            state = s.state
