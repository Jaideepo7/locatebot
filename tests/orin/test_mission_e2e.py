"""R3 end to end in simulation: Telegram /find on the Pi -> ZeroMQ (127.0.0.1) -> MissionBridge on
the Orin -> skills on the simulated car -> statuses back -> customer replies.

Pi side: the real RemoteService (MissionService, SQLite outbox, OutboxSender, background thread)
with a console-style recording provider and the real ZmqRobotLink. Orin side: a real Publisher and
Subscriber, MissionBridge, OrinRuntime on the simulated room/car of test_e2e_sim.py (no LLM:
routine mode). Only Photon/Telegram, the camera model and the hardware are fake.
"""
import math
import time
import uuid

import numpy as np
import pytest

pytest.importorskip("zmq")
from locatebot.lidar.sim import raycast, simulate_scan  # noqa: E402
from locatebot.lidar.types import Pose2D  # noqa: E402
from locatebot.link.messages import OrinStatus  # noqa: E402
from locatebot.link.transport import Publisher, Subscriber  # noqa: E402
from locatebot.mission.types import SkillName, SkillState  # noqa: E402
from locatebot.nav.sim import BicycleSim  # noqa: E402
from locatebot.orin.live_map import LiveMapper  # noqa: E402
from locatebot.orin.missions import MissionBridge  # noqa: E402
from locatebot.orin.runtime import OrinRuntime  # noqa: E402
from locatebot.orin.skills import ExecutorConfig, SkillExecutor  # noqa: E402
from locatebot.remote.config import Settings  # noqa: E402
from locatebot.remote.messaging import Conversation, ConsoleProvider, InboundMessage, SendResult  # noqa: E402
from locatebot.remote.robot_zmq import ZmqRobotLink  # noqa: E402
from locatebot.remote.service import RemoteService  # noqa: E402
from locatebot.vision.service import VisionService  # noqa: E402

from .test_e2e_sim import DT, HFOV, VEH, FakeEngine, fake_jpeg, room_with_items  # noqa: E402

LOCAL = "tcp://127.0.0.1:*"
PROJECT = "pho_prj_01jabcdefghjkmnpqrstvwxyz1"
SPACE, USER, BOT = "pho_spc_01jaaaaaaaaaaaaaaaaaaaaaaa", "pho_usr_01jaaaaaaaaaaaaaaaaaaaaaaa", "pho_usr_01jbot"
ITEMS = {"marker": (3.75, 2.4), "bolt": (5.3, 4.5)}


class Telegram(ConsoleProvider):
    """ConsoleProvider that also records what the customer would read."""

    def __init__(self):
        self.chat = []

    def send_text(self, conversation, text, idempotency_key):
        self.chat.append(text)
        return SendResult(True)


class Rig:
    def __init__(self, tmp_path, search_timeout_s=600.0):
        self.telegram = Telegram()
        self.orin_pub = Publisher(LOCAL)
        self.link = ZmqRobotLink(self.orin_pub.endpoint, pub_endpoint=LOCAL, poll_s=0.01)
        settings = Settings.from_env({"PHOTON_MODE": "console", "PHOTON_INBOUND": "webhook",
                                      "PHOTON_WEBHOOK_SECRET": "whsec_" + "a" * 43 + "=", "PHOTON_PROJECT_ID": PROJECT,
                                      "DISPATCH_DB_PATH": str(tmp_path / "pi.db")})
        self.pi = RemoteService(settings, provider=self.telegram, link=self.link, tick_s=0.05)
        self.orin_sub = Subscriber(self.link.pub_endpoint, ["mission_request", "mission_cancel"])
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
        self.pi.start()

    def close(self):
        self.vision.stop(timeout=2.0)
        self.pi.stop()
        self.orin_sub.close()
        self.orin_pub.close()

    def customer(self, text):
        msg = InboundMessage("pho_evt_" + uuid.uuid4().hex[:26], Conversation("photon", SPACE, BOT), USER, "c", text)
        self.pi.missions.handle_inbound(msg)

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

    def run_until(self, cond, max_sim_s=300.0, wall_s=30.0):
        end_sim, end_wall = self.sim_t + max_sim_s, time.monotonic() + wall_s
        while self.sim_t < end_sim and time.monotonic() < end_wall:
            if cond():
                return True
            self.step()
        return cond()

    def wait_replies(self, n, wall_s=3.0):
        """Keep the Orin stepping while the Pi delivers replies (its thread runs every 0.05 s)."""
        return self.run_until(lambda: len(self.telegram.chat) >= n, max_sim_s=wall_s * 10, wall_s=wall_s)

    def connect(self):
        assert self.run_until(lambda: self.link.connected, wall_s=5.0), "Pi never heard the Orin"
        t = time.monotonic() + 0.3  # slow joiner: let the Orin's SUB reach the Pi's PUB
        self.run_until(lambda: time.monotonic() > t, wall_s=1.0)


@pytest.fixture
def make_rig(tmp_path):
    rigs = []

    def make(**kw):
        d = tmp_path / str(len(rigs))
        d.mkdir()
        r = Rig(d, **kw)
        rigs.append(r)
        return r

    yield make
    for r in rigs:
        r.close()


def test_find_marker_end_to_end(make_rig):
    r = make_rig()
    t0 = time.monotonic()
    r.connect()
    r.customer("/find marker")
    done = "Search mission completed."
    assert r.run_until(lambda: done in r.telegram.chat, max_sim_s=200.0, wall_s=25.0), r.telegram.chat
    print(f"\n/find marker: {r.sim_t - 500:.0f} s sim, {time.monotonic() - t0:.1f} s wall")
    for t in r.telegram.chat:
        print("  >", t)
    assert r.telegram.chat == [
        "Request received. Sending your target to the robot.",
        "The robot has started searching for: marker.",
        "Target detected. Preparing to approach it.",
        "The robot is approaching the target.",
        "Search mission completed.",
    ]
    assert [s.state for s in r.sent_statuses] == ["ACCEPTED", "SEARCHING", "TARGET_DETECTED", "APPROACHING",
                                                  "COMPLETED"]
    mx, my = ITEMS["marker"]
    assert math.hypot(r.car.pose.x - mx, r.car.pose.y - my) < 1.4
    m = r.pi.store.latest_for("photon", SPACE)
    assert m.state.value == "COMPLETED"
    assert r.runtime.goal == ""
    assert "found marker at (" in r.sent_statuses[2].message


def test_find_unicorn_not_found(make_rig):
    r = make_rig(search_timeout_s=40.0)
    t0 = time.monotonic()
    r.connect()
    r.customer("/find unicorn")
    assert r.run_until(lambda: any("couldn't spot" in t for t in r.telegram.chat), max_sim_s=120.0, wall_s=25.0), \
        r.telegram.chat
    print(f"\n/find unicorn: {r.sim_t - 500:.0f} s sim, {time.monotonic() - t0:.1f} s wall; {r.telegram.chat[-1]}")
    assert r.telegram.chat[:2] == ["Request received. Sending your target to the robot.",
                                   "The robot has started searching for: unicorn."]
    last = r.telegram.chat[-1]
    assert last.startswith("I couldn't spot unicorn. The search has ended. (")
    assert "not found: unicorn" in last and "found: " in last
    assert (r.sent_statuses[-1].state, r.sent_statuses[-1].reason) == ("FAILED", "NOT_FOUND")
    assert not any("detected" in t.lower() for t in r.telegram.chat)


def test_cancel_mid_search(make_rig):
    r = make_rig()
    t0 = time.monotonic()
    r.connect()
    r.customer("/find unicorn")
    assert r.run_until(lambda: "The robot has started searching for: unicorn." in r.telegram.chat, wall_s=10.0)
    assert r.run_until(lambda: r.runtime.executor.status.state == SkillState.RUNNING, wall_s=5.0)
    r.customer("/cancel")
    assert r.run_until(lambda: "Mission cancelled." in r.telegram.chat, wall_s=10.0), r.telegram.chat
    print(f"\ncancel: {time.monotonic() - t0:.1f} s wall")
    assert r.telegram.chat[-2:] == ["Cancelling... I'll confirm once the robot has stopped.", "Mission cancelled."]
    st = r.runtime.executor.status
    assert st.call.name == SkillName.STOP and st.state == SkillState.SUCCEEDED
    assert r.runtime.executor.search_items == ()
    assert r.sent_statuses[-1].state == "CANCELLED"
    assert r.pi.store.latest_for("photon", SPACE).state.value == "CANCELLED"
