"""Never drive on a pose the localizer doesn't trust (saved-map mode).

When localization is lost (or not established yet) the pose freezes at the last good value;
a running skill would keep steering as if the car were still there. The runtime must stop it,
hold zero output, tell the LLM why, and log the loss and the recovery.
"""
import math

import numpy as np

from locatebot.lidar.mapping import Mapper, demo_room, demo_trajectory
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import SkillCall, SkillName, SkillState, VehicleConfig
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import OrinRuntime
from locatebot.orin.skills import SkillExecutor


def saved_map(tmp_path):
    room = demo_room()
    m = Mapper(size_m=16.0)
    rng = np.random.default_rng(0)
    for i, p in enumerate(demo_trajectory(60)):
        m.add_scan(simulate_scan(room, p, rng=rng, stamp_s=i * 0.1, seq=i))
    paths = m.save(tmp_path / "room")
    return room, demo_trajectory(60)[0], paths["lbmap"]


def test_lost_localization_stops_the_skill_and_holds_still(tmp_path):
    room, start, path = saved_map(tmp_path)
    rt = OrinRuntime(LiveMapper.from_saved(path), SkillExecutor(VehicleConfig()))
    rng = np.random.default_rng(1)
    t = 100.0
    for i in range(5):  # localize at the start pose (the map frame is the first scan's pose)
        rt.on_scan(simulate_scan(room, start, rng=rng, stamp_s=t, seq=i))
        t += 0.1
    assert rt.mapper.stats()["initialized"] and not rt.mapper.stats()["lost"]
    rt.on_skill(SkillCall(SkillName.EXPLORE))
    rt.step(t)
    assert rt.executor.status.state == SkillState.RUNNING

    # The car is "teleported": scans now come from a different room entirely.
    other = Room.box(3.0, 9.0).with_box(1.0, 1.0, 1.4, 6.0)
    outs = []
    for i in range(40):
        rt.on_scan(simulate_scan(other, Pose2D(1.5, 4.0, 1.0), rng=rng, stamp_s=t, seq=100 + i))
        outs.append(rt.step(t))
        t += 0.1
        if rt.mapper.stats()["lost"]:
            break
    assert rt.mapper.stats()["lost"], "the localizer never declared lost"
    out = rt.step(t)
    assert (out.throttle, out.steering) == (0.0, 0.0)
    st = rt.executor.status
    assert st.state != SkillState.RUNNING and "localization" in st.detail
    ws = rt.world_state(t)
    assert any("localization lost" in n for n in ws.notes)
    assert any("localization lost" in e["text"] for e in rt.status()["events"])
    # New skills don't drive while lost.
    rt.on_skill(SkillCall(SkillName.GO_TO, {"x": 1.0, "y": 0.0}))
    for _ in range(5):
        out = rt.step(t)
        assert out.throttle == 0.0
        t += 0.1


def test_mapping_mode_is_unaffected():
    rt = OrinRuntime(LiveMapper(size_m=12.0), SkillExecutor(VehicleConfig()))
    rng = np.random.default_rng(2)
    for i in range(5):
        rt.on_scan(simulate_scan(demo_room(), Pose2D(4.0, 3.0, 0.0), rng=rng, stamp_s=1.0 + 0.1 * i, seq=i))
    rt.on_skill(SkillCall(SkillName.EXPLORE))
    rt.step(2.0)
    assert rt.executor.status.state == SkillState.RUNNING
    assert not any("localization" in n for n in rt.world_state(2.0).notes)


def test_skill_sent_before_localization_waits_then_runs(tmp_path):
    room, start, path = saved_map(tmp_path)
    rt = OrinRuntime(LiveMapper.from_saved(path), SkillExecutor(VehicleConfig()))
    rt.on_skill(SkillCall(SkillName.EXPLORE))
    out = rt.step(99.0)                                   # no scans yet: not localized
    assert (out.throttle, out.steering) == (0.0, 0.0)
    assert rt.executor.status.state == SkillState.RUNNING  # held, not cancelled
    assert any("not localized" in n for n in rt.world_state(99.0).notes)
    rng = np.random.default_rng(3)
    t = 100.0
    for i in range(5):
        rt.on_scan(simulate_scan(room, start, rng=rng, stamp_s=t, seq=i))
        rt.step(t)
        t += 0.1
    assert rt.mapper.stats()["initialized"]
    assert rt.executor.status.state == SkillState.RUNNING
    assert not any("locali" in n for n in rt.world_state(t).notes)
    assert any(e["text"] == "localization recovered" for e in rt.status()["events"])
