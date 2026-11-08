"""R4 in simulation: the scripted LLM picks what to search for from the `goal:` line only.

Same rig as test_e2e_sim.py (simulated room, bicycle car, LiDAR odometry mapping, the real vision
worker with a fake engine). The operator sets the goal "find the bolt"; the policy derives the
item from that line, searches for it, drives to it and stops with a "goal done" reason.
"""
import math
import time

import numpy as np

from locatebot.agent.brain import Brain
from locatebot.agent.llm import FakeLLM, ToolChoice
from locatebot.lidar.sim import raycast, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import SkillName, SkillState
from locatebot.nav.sim import BicycleSim
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import OrinRuntime
from locatebot.orin.skills import SkillExecutor
from locatebot.vision.service import VisionService

from .test_e2e_sim import DT, HFOV, VEH, FakeEngine, fake_jpeg, room_with_items


def goal_policy(system, user, tools, timeout):
    """Reads the goal from the first line; uses the skill line only to know where it is."""
    lines = user.splitlines()
    goal = lines[0]
    assert goal.startswith("goal: ")
    if goal == "goal: none":
        return ToolChoice("stop", {}, "no goal")
    item = goal.split()[-1]  # "goal: find the bolt" -> "bolt" (a short common noun)
    skill = next(line for line in lines if line.startswith("skill:"))
    if "running" in skill:
        return ToolChoice("continue", {}, "working on the goal")
    if f"succeeded go_to(item={item})" in skill:
        return ToolChoice("stop", {}, f"goal done: arrived at the {item}")
    if "succeeded search" in skill:
        return ToolChoice("go_to", {"item": item}, f"found the {item}; driving over")
    if "failed search" in skill:
        return ToolChoice("stop", {}, f"goal impossible: no {item} found")
    return ToolChoice("search", {"items": [item]}, f"the goal asks for the {item}")


def test_goal_driven_search_go_to_and_stop_in_simulation():
    items_world = {"bolt": (3.75, 2.4), "marker": (5.3, 4.5)}
    room = room_with_items(items_world)
    start = Pose2D(1.0, 1.0, 0.2)
    car = BicycleSim(VEH, start, dt=0.05)
    rng = np.random.default_rng(3)
    sim_t = [500.0]
    clock = lambda: sim_t[0]  # noqa: E731

    llm = FakeLLM(default=goal_policy)
    runtime = OrinRuntime(LiveMapper(size_m=14.0), SkillExecutor(VEH), clock=clock)
    brain = Brain(llm, mode="auto", on_skill=runtime.on_skill, clock=clock, decide_interval_s=5.0,
                  llm_timeout_s=5.0)
    engine = FakeEngine()
    vision = VisionService(engine, on_result=runtime.on_vision_result, hfov_rad=HFOV)
    runtime.brain, runtime.vision = brain, vision
    runtime.set_goal("find the bolt")
    vision.start()
    outcomes = []
    try:
        for k in range(int(400 / DT)):
            now = sim_t[0]
            p = car.pose
            runtime.on_scan(simulate_scan(room, p, n_beams=508, noise_std_m=0.005, rng=rng, stamp_s=now,
                                          seq=k, dropout=0.1))
            if k % 5 == 0:
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
                time.sleep(0.002)
            out = runtime.step(now)
            brain.wait_idle(2.0)
            for _ in range(2):
                car.step(out.throttle, out.steering)
            sim_t[0] += DT
            st = runtime.executor.status
            if st.state in (SkillState.SUCCEEDED, SkillState.FAILED) and st.call is not None:
                if not outcomes or outcomes[-1][0] != st.call or outcomes[-1][1] != st.state:
                    outcomes.append((st.call, st.state, st.detail))
            if outcomes and outcomes[-1][0].name == SkillName.STOP:
                break
    finally:
        vision.stop(timeout=2.0)

    print(f"\nsim time {sim_t[0] - 500.0:.0f} s, LLM calls {len(llm.calls)}")
    for c, s_, d in outcomes:
        print(f"  {c.name.value:7} {dict(c.args)} {s_.value:9} {d} [{c.reason}]")
    names = [(c.name.value, s.value) for c, s, _ in outcomes]
    assert names == [("search", "succeeded"), ("go_to", "succeeded"), ("stop", "succeeded")], outcomes
    assert tuple(outcomes[0][0].args["items"]) == ("bolt",)  # chosen from the goal, nothing else
    assert dict(outcomes[1][0].args) == {"item": "bolt"}
    assert outcomes[2][0].reason.startswith("goal done")
    bx, by = items_world["bolt"]
    assert math.hypot(car.pose.x - bx, car.pose.y - by) < 1.4
    assert all(u[1].splitlines()[0] == "goal: find the bolt" for u in llm.calls)
    assert runtime.status()["goal"]["text"] == "find the bolt"
