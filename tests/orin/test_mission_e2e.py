"""Mission bridge end to end in simulation: a fake Pi sends MissionRequest / MissionCancel over
ZeroMQ (127.0.0.1) -> MissionBridge on the Orin -> skills on the simulated car -> MissionStatus
back to the fake Pi.

Pi side: a real Publisher (mission_request / mission_cancel) and Subscriber (mission_status,
orin_status), nothing else. Orin side: a real Publisher and Subscriber, MissionBridge, OrinRuntime
on the simulated room/car of test_e2e_sim.py (no LLM: routine mode). Only the camera model and the
hardware are fake.
"""
import math
import time
import uuid

import numpy as np
import pytest

pytest.importorskip("zmq")
from locatebot.lidar.sim import raycast, simulate_scan  # noqa: E402
from locatebot.lidar.types import Pose2D  # noqa: E402
from locatebot.link.messages import MissionCancel, MissionRequest, MissionStatus, OrinStatus  # noqa: E402
from locatebot.link.transport import Publisher, Subscriber  # noqa: E402
from locatebot.mission.types import SkillName, SkillState  # noqa: E402
from locatebot.nav.sim import BicycleSim  # noqa: E402
from locatebot.orin.live_map import LiveMapper  # noqa: E402
from locatebot.orin.missions import MissionBridge  # noqa: E402
from locatebot.orin.runtime import OrinRuntime  # noqa: E402
from locatebot.orin.skills import ExecutorConfig, SkillExecutor  # noqa: E402
from locatebot.vision.service import VisionService  # noqa: E402

from .test_e2e_sim import DT, HFOV, VEH, FakeEngine, fake_jpeg, room_with_items  # noqa: E402

LOCAL = "tcp://127.0.0.1:*"
ITEMS = {"marker": (3.75, 2.4), "bolt": (5.3, 4.5)}
TTL_S = 600.0


class FakePi:
    """The Pi's half of the mission link: publishes requests/cancels, collects statuses."""

    def __init__(self, orin_endpoint):
        self.pub = Publisher(LOCAL)
        self.sub = Subscriber(orin_endpoint, ["mission_status", "orin_status"])
        self.statuses: list[MissionStatus] = []
        self.heard_orin = False

    def poll(self):
        for msg in self.sub.poll(0.0):
            if isinstance(msg, MissionStatus):
                self.statuses.append(msg)
            elif isinstance(msg, OrinStatus):
                self.heard_orin = True

    def request(self, target):
        mid = uuid.uuid4().hex
        now = time.time()
        self.pub.send(MissionRequest(mid, target, now, now + TTL_S))
        return mid

    def cancel(self, mid, reason="customer"):
        self.pub.send(MissionCancel(mid, reason))

    def states(self, mid):
        return [s.state for s in self.statuses if s.mission_id == mid]

    def last(self, mid):
        mine = [s for s in self.statuses if s.mission_id == mid]
        return mine[-1] if mine else None

    def close(self):
        self.sub.close()
        self.pub.close()


class Rig:
    def __init__(self, search_timeout_s=600.0):
        self.orin_pub = Publisher(LOCAL)
        self.pi = FakePi(self.orin_pub.endpoint)
        self.orin_sub = Subscriber(self.pi.pub.endpoint, ["mission_request", "mission_cancel"])
        # Orin: simulated room + car, vision on a fake engine, no LLM (routine mode)
        self.room = room_with_items(ITEMS)
        self.car = BicycleSim(VEH, Pose2D(1.0, 1.0, 0.2), dt=0.05)
        self.rng = np.random.default_rng(3)
        self.sim_t = 500.0
        self.k = 0
        self.runtime = OrinRuntime(LiveMapper(size_m=14.0),
                                   SkillExecutor(VEH, ExecutorConfig(search_timeout_s=search_timeout_s)),
                                   clock=lambda: self.sim_t)
        self.engine = FakeEngine()
        self.vision = VisionService(self.engine, on_result=self.runtime.on_vision_result, hfov_rad=HFOV)
        self.runtime.vision = self.vision
        self.statuses = []
        self.bridge = MissionBridge(self.runtime, self.statuses.append)
        self.runtime.missions = self.bridge
        self.sent_statuses = []
        self.last_heartbeat = -1.0
        self.vision.start()

    def close(self):
        self.vision.stop(timeout=2.0)
        self.pi.close()
        self.orin_sub.close()
        self.orin_pub.close()

    def step(self):
        """One 0.1 s step of the Orin: link in, sensors, control (+ bridge), statuses out, car."""
        for msg in self.orin_sub.poll(0.0):
            self.bridge.on_message(msg)
        now, p = self.sim_t, self.car.pose
        self.runtime.on_scan(simulate_scan(self.room, p, n_beams=508, noise_std_m=0.005, rng=self.rng,
                                           stamp_s=now, seq=self.k, dropout=0.1))
        if self.k % 5 == 0:
            visible = {}
            for name, (ix, iy) in ITEMS.items():
                d = math.hypot(ix - p.x, iy - p.y)
                b = math.atan2(iy - p.y, ix - p.x) - p.theta
                b = math.atan2(math.sin(b), math.cos(b))
                hit = raycast(self.room, (p.x, p.y), np.array([p.theta + b]))[0]
                if abs(b) < HFOV / 2 - 0.05 and d < 4.0 and hit > d - 0.2:
                    visible[name] = b
            calls = self.engine.calls
            self.runtime.on_frame(fake_jpeg(visible), now)
            want = len(self.vision.status().get("items", [])) or len(self.runtime.executor.search_items)
            deadline = time.monotonic() + 1.0
            while self.engine.calls < calls + want and time.monotonic() < deadline:
                time.sleep(0.001)
            time.sleep(0.002)
        out = self.runtime.step(now)
        while self.statuses:
            st = self.statuses.pop(0)
            self.sent_statuses.append(st)
            self.orin_pub.send(st)
        if time.monotonic() - self.last_heartbeat > 0.2:
            self.last_heartbeat = time.monotonic()
            self.orin_pub.send(OrinStatus(time.monotonic(), True, None, self.runtime.executor.status.state.value,
                                          None, None, "e2e-orin"))
        for _ in range(2):
            self.car.step(out.throttle, out.steering)
        self.sim_t += DT
        self.k += 1
        self.pi.poll()

    def run_until(self, cond, max_sim_s=300.0, wall_s=30.0):
        end_sim, end_wall = self.sim_t + max_sim_s, time.monotonic() + wall_s
        while self.sim_t < end_sim and time.monotonic() < end_wall:
            if cond():
                return True
            self.step()
        return cond()

    def connect(self):
        assert self.run_until(lambda: self.pi.heard_orin, wall_s=5.0), "Pi never heard the Orin"
        t = time.monotonic() + 0.3  # slow joiner: let the Orin's SUB reach the Pi's PUB
        self.run_until(lambda: time.monotonic() > t, wall_s=1.0)


@pytest.fixture
def make_rig():
    rigs = []

    def make(**kw):
        r = Rig(**kw)
        rigs.append(r)
        return r

    yield make
    for r in rigs:
        r.close()


def test_find_marker_end_to_end(make_rig):
    r = make_rig()
    t0 = time.monotonic()
    r.connect()
    mid = r.pi.request("marker")
    assert r.run_until(lambda: "COMPLETED" in r.pi.states(mid), max_sim_s=200.0, wall_s=25.0), r.pi.statuses
    print(f"\nfind marker: {r.sim_t - 500:.0f} s sim, {time.monotonic() - t0:.1f} s wall")
    for s in r.pi.statuses:
        print("  >", s.state, s.message)
    expected = ["ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING", "COMPLETED"]
    assert r.pi.states(mid) == expected
    assert [s.state for s in r.sent_statuses] == expected
    mx, my = ITEMS["marker"]
    assert math.hypot(r.car.pose.x - mx, r.car.pose.y - my) < 1.4
    assert r.runtime.goal == ""
    assert "found marker at (" in r.pi.statuses[2].message
    assert r.pi.last(mid).reason == ""


def test_find_unicorn_not_found(make_rig):
    r = make_rig(search_timeout_s=40.0)
    t0 = time.monotonic()
    r.connect()
    mid = r.pi.request("unicorn")
    assert r.run_until(lambda: "FAILED" in r.pi.states(mid), max_sim_s=120.0, wall_s=25.0), r.pi.statuses
    last = r.pi.last(mid)
    print(f"\nfind unicorn: {r.sim_t - 500:.0f} s sim, {time.monotonic() - t0:.1f} s wall; {last.message}")
    assert r.pi.states(mid) == ["ACCEPTED", "SEARCHING", "FAILED"]
    assert (last.state, last.reason) == ("FAILED", "NOT_FOUND")
    assert "not found: unicorn" in last.message and "found: " in last.message
    assert (r.sent_statuses[-1].state, r.sent_statuses[-1].reason) == ("FAILED", "NOT_FOUND")


def test_cancel_mid_search(make_rig):
    r = make_rig()
    t0 = time.monotonic()
    r.connect()
    mid = r.pi.request("unicorn")
    assert r.run_until(lambda: "SEARCHING" in r.pi.states(mid), wall_s=10.0), r.pi.statuses
    assert r.run_until(lambda: r.runtime.executor.status.state == SkillState.RUNNING, wall_s=5.0)
    r.pi.cancel(mid)
    assert r.run_until(lambda: "CANCELLED" in r.pi.states(mid), wall_s=10.0), r.pi.statuses
    print(f"\ncancel: {time.monotonic() - t0:.1f} s wall")
    assert r.pi.states(mid) == ["ACCEPTED", "SEARCHING", "CANCELLED"]
    st = r.runtime.executor.status
    assert st.call.name == SkillName.STOP and st.state == SkillState.SUCCEEDED
    assert r.runtime.executor.search_items == ()
    assert r.runtime.goal == ""
    assert r.sent_statuses[-1].state == "CANCELLED"
