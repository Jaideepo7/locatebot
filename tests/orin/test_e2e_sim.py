"""End-to-end in simulation: LLM brain -> skills -> Nav-lite -> car, with LiDAR odometry mapping,
the real vision worker thread and LiDAR-ranged item localization.

Only the hardware and the model are fake: the room, car and LiDAR are simulated, the vision
"engine" reads which items the simulated camera can see from a comment segment in a tiny JPEG,
and the LLM is scripted. The pose the robot uses comes from scan matching, not ground truth.
"""
import json
import math
import re
import struct
import time

import numpy as np

from locatebot.agent.brain import Brain
from locatebot.agent.llm import FakeLLM, ToolChoice
from locatebot.lidar.sim import Room, raycast, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import SkillName, SkillState, VehicleConfig
from locatebot.nav.sim import BicycleSim
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import OrinRuntime
from locatebot.orin.skills import SkillExecutor
from locatebot.vision.postprocess import Detection
from locatebot.vision.service import VisionService

VEH = VehicleConfig()
HFOV = math.radians(70)
W, H = 640, 360
DT = 0.1


def fake_jpeg(visible: dict[str, float]) -> bytes:
    """Minimal JPEG: SOI, a COM segment with {item: bearing}, a SOF0 header with the size, EOI."""
    com = json.dumps(visible).encode()
    return (b"\xff\xd8" + b"\xff\xfe" + struct.pack(">H", len(com) + 2) + com
            + b"\xff\xc0" + struct.pack(">HBHHB", 8, 8, H, W, 3) + b"\xff\xd9")


class FakeEngine:
    """LocateAnything stand-in: boxes the prompted item if the frame says it is visible."""

    def __init__(self):
        self.calls = 0

    def locate(self, image: bytes, prompt: str, mode: str = "hybrid"):
        self.calls += 1
        i = image.index(b"\xff\xfe")
        (n,) = struct.unpack(">H", image[i + 2:i + 4])
        visible = json.loads(image[i + 4:i + 2 + n])
        item = re.search(r"description: (.+)\.$", prompt).group(1)
        if item not in visible:
            return []
        fx = (W / 2) / math.tan(HFOV / 2)
        cx = W / 2 - fx * math.tan(visible[item])
        return [Detection(item, (cx - 15, 150, cx + 15, 220))]


def room_with_items(items):
    room = Room.box(6.0, 5.0).with_box(2.6, 2.0, 3.4, 2.8)  # a table
    for x, y in items.values():
        room = room.with_box(x - 0.08, y - 0.08, x + 0.08, y + 0.08)
    return room


def test_llm_searches_then_drives_to_the_item_in_simulation():
    items_world = {"marker": (3.75, 2.4), "bolt": (5.3, 4.5)}
    room = room_with_items(items_world)
    start = Pose2D(1.0, 1.0, 0.2)
    car = BicycleSim(VEH, start, dt=0.05)
    rng = np.random.default_rng(3)
    sim_t = [500.0]
    clock = lambda: sim_t[0]  # noqa: E731

    def policy(system, user, tools, timeout):
        """A tiny scripted 'LLM' that reads the world-state text like a real model would."""
        skill = next(line for line in user.splitlines() if line.startswith("skill:"))
        if "running" in skill:
            return ToolChoice("continue", {}, "the current skill is still right")
        if "succeeded go_to" in skill:
            return ToolChoice("stop", {}, "done: at the marker")
        if "succeeded search" in skill:
            return ToolChoice("go_to", {"item": "marker"}, "found it; drive over")
        return ToolChoice("search", {"items": ["marker", "bolt"]}, "operator asked for these")

    llm = FakeLLM(default=policy)
    mapper = LiveMapper(size_m=14.0)
    runtime = OrinRuntime(mapper, SkillExecutor(VEH), clock=clock)
    brain = Brain(llm, mode="auto", on_skill=runtime.on_skill, clock=clock, decide_interval_s=5.0,
                  llm_timeout_s=5.0)
    engine = FakeEngine()
    vision = VisionService(engine, on_result=runtime.on_vision_result, hfov_rad=HFOV)
    runtime.brain, runtime.vision = brain, vision
    vision.start()

    min_wall = math.inf
    outcomes = []
    try:
        for k in range(int(400 / DT)):
            now = sim_t[0]
            p = car.pose
            runtime.on_scan(simulate_scan(room, p, n_beams=508, noise_std_m=0.005, rng=rng, stamp_s=now,
                                          seq=k, dropout=0.1))
            if k % 5 == 0:  # camera frame every 0.5 s; what the camera can see right now
                visible = {}
                for name, (ix, iy) in items_world.items():
                    d = math.hypot(ix - p.x, iy - p.y)
                    b = math.atan2(iy - p.y, ix - p.x) - p.theta
                    b = math.atan2(math.sin(b), math.cos(b))
                    hit = raycast(room, (p.x, p.y), np.array([p.theta + b]))[0]
                    if abs(b) < HFOV / 2 - 0.05 and d < 4.0 and hit > d - 0.2:
                        visible[name] = b
                calls = engine.calls
                runtime.on_frame(fake_jpeg(visible), now)
                want = len(vision.status().get("items", [])) or len(runtime.executor.search_items)
                deadline = time.monotonic() + 1.0
                while engine.calls < calls + want and time.monotonic() < deadline:
                    time.sleep(0.001)
                time.sleep(0.002)  # let on_vision_result finish
            out = runtime.step(now)
            brain.wait_idle(2.0)
            for _ in range(2):
                car.step(out.throttle, out.steering)
            sim_t[0] += DT
            s = room.segments
            q = np.array([car.pose.x, car.pose.y])
            a, b2 = s[:, :2], s[:, 2:]
            t = np.clip(np.einsum("ij,ij->i", q - a, b2 - a) / np.maximum(np.einsum("ij,ij->i", b2 - a, b2 - a), 1e-12), 0, 1)
            min_wall = min(min_wall, float(np.min(np.linalg.norm(a + t[:, None] * (b2 - a) - q, axis=1))))
            st = runtime.executor.status
            if st.state in (SkillState.SUCCEEDED, SkillState.FAILED) and st.call is not None:
                if not outcomes or outcomes[-1][0] != st.call.name or outcomes[-1][1] != st.state:
                    outcomes.append((st.call.name, st.state, st.detail))
            if outcomes and outcomes[-1][0] == SkillName.STOP:
                break
    finally:
        vision.stop(timeout=2.0)

    print(f"\nsim time {sim_t[0] - 500.0:.0f} s, closest wall {min_wall:.2f} m, LLM calls {len(llm.calls)}")
    for n, s_, d in outcomes:
        print(f"  {n.value:7} {s_.value:9} {d}")
    names = [(n.value, s.value) for n, s, _ in outcomes]
    assert ("search", "succeeded") in names, outcomes
    assert ("go_to", "succeeded") in names, outcomes
    assert names[-1] == ("stop", "succeeded"), outcomes
    # Item positions in the map frame (= the car's start pose) match the truth.
    to_map = start.inverse()
    for name, xy in items_world.items():
        mx, my = to_map.transform_points(np.array([xy]))[0]
        tr = runtime.tracker.best(name)
        assert tr is not None and tr.confirmed, name
        print(f"  {name}: mapped at ({tr.x:.2f}, {tr.y:.2f}), truth ({mx:.2f}, {my:.2f}), "
              f"error {math.hypot(tr.x - mx, tr.y - my):.2f} m, {tr.sightings} sightings")
        assert math.hypot(tr.x - mx, tr.y - my) < 0.35, (name, (tr.x, tr.y), (mx, my))
    # The car ended near the marker (true world frame) and never touched anything.
    mx, my = items_world["marker"]
    assert math.hypot(car.pose.x - mx, car.pose.y - my) < 1.4
    assert min_wall > VEH.half_width_m * 0.8, min_wall
    # The LLM saw a world state that mentions the found items before choosing go_to.
    prompts = [c[1] for c in llm.calls]
    assert any("marker" in u and "confirmed" in u for u in prompts[1:]), prompts[1][:400]
    st = runtime.status()
    assert st["stats"]["fixes"] >= 4 and st["brain"]["failures"] == 0 or st["brain"]["in_fallback"] is False
