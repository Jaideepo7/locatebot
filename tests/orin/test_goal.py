"""Goal input (R4): OrinRuntime.set_goal/clear_goal, the operator routes and the world state."""
import json
import threading
import urllib.error
import urllib.request

import pytest

from locatebot.agent.brain import Brain
from locatebot.agent.llm import FakeLLM, ToolChoice
from locatebot.mission.types import VehicleConfig
from locatebot.orin.app import AppConfig, OrinApp, make_server
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import MAX_GOAL_CHARS, OrinRuntime, normalize_goal
from locatebot.orin.skills import SkillExecutor

from .test_app import runtime_with_map


def test_normalize_goal():
    assert normalize_goal("  find   the\nbolt ") == "find the bolt"
    assert normalize_goal("x" * MAX_GOAL_CHARS) == "x" * MAX_GOAL_CHARS
    for bad in ["", "   ", "x" * (MAX_GOAL_CHARS + 1), "find\x07bolt", "find \x00", None, 5]:
        with pytest.raises(ValueError):
            normalize_goal(bad)


def test_set_and_clear_goal_events_world_state_and_status():
    rt = OrinRuntime(LiveMapper(size_m=8.0), SkillExecutor(VehicleConfig()), clock=lambda: 50.0)
    assert rt.world_state().goal == "" and rt.status()["goal"] is None
    assert rt.set_goal(" find  the bolt ", source="telegram") == "find the bolt"
    assert rt.goal == "find the bolt"
    assert rt.world_state().goal == "find the bolt"
    assert rt.step(50.0) is not None and rt.world_state().goal == "find the bolt"
    st = rt.status()
    assert st["goal"] == {"text": "find the bolt", "source": "telegram", "set_s": 50.0}
    assert st["events"][-1]["text"] == "goal set: find the bolt (by telegram)"
    rt.clear_goal()
    assert rt.goal == "" and rt.status()["goal"] is None
    assert rt.status()["events"][-1]["text"] == "goal cleared"
    n = len(rt.status()["events"])
    rt.clear_goal()  # nothing to clear: no new event
    assert len(rt.status()["events"]) == n
    with pytest.raises(ValueError):
        rt.set_goal("")
    assert rt.goal == ""


def test_goal_is_thread_safe():
    rt = OrinRuntime(LiveMapper(size_m=8.0), SkillExecutor(VehicleConfig()))
    errors = []

    def writer(k):
        try:
            for i in range(200):
                rt.set_goal(f"find item {k}-{i}")
                rt.world_state()
                if i % 7 == 0:
                    rt.clear_goal()
        except Exception as e:  # pragma: no cover
            errors.append(e)

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors
    g = rt.goal
    assert g == "" or g.startswith("find item ")


def test_goal_reaches_the_brain_prompt():
    llm = FakeLLM(default=ToolChoice("look_around", {}))
    clock = [10.0]
    rt = OrinRuntime(LiveMapper(size_m=8.0), SkillExecutor(VehicleConfig()), clock=lambda: clock[0])
    rt.brain = Brain(llm, mode="approve", on_skill=rt.on_skill, clock=lambda: clock[0])
    rt.step(10.0)
    rt.brain.wait_idle(2.0)
    rt.set_goal("find the bolt")
    clock[0] = 10.5
    rt.step(10.5)
    rt.brain.wait_idle(2.0)
    user = llm.calls[-1][1]
    assert user.splitlines()[0] == "goal: find the bolt"
    assert "new goal: find the bolt" in user


@pytest.fixture
def server(tmp_path):
    app = OrinApp(runtime_with_map(), lambda t, s: None, config=AppConfig(maps_dir=str(tmp_path)))
    srv = make_server(app, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield app, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def post(url, body, ctype="application/json"):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": ctype})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_goal_routes(server):
    app, base = server
    for bad in [{}, {"text": ""}, {"text": "x" * 201}, {"text": "a\x01b"}, {"text": 5}]:
        assert post(base + "/goal", bad)[0] == 400, bad
    assert post(base + "/goal", {"text": "find the bolt"}, ctype="text/plain")[0] == 400
    assert post(base + "/goal", {"text": "  find the bolt "}) == (200, {"ok": True, "goal": "find the bolt"})
    st = json.loads(urllib.request.urlopen(base + "/status", timeout=5).read())
    assert st["goal"]["text"] == "find the bolt" and st["goal"]["source"] == "operator"
    assert any(e["text"].startswith("goal set: find the bolt") for e in st["events"])
    assert post(base + "/goal/clear", {}) == (200, {"ok": True})
    st = json.loads(urllib.request.urlopen(base + "/status", timeout=5).read())
    assert st["goal"] is None


def test_page_has_goal_controls_and_no_external_urls(server):
    app, base = server
    page = urllib.request.urlopen(base + "/", timeout=5).read()
    for needle in (b'id="goaltext"', b'id="setgoal"', b'id="cleargoal"', b'"/goal"', b'"/goal/clear"'):
        assert needle in page
    assert b"http://" not in page and b"https://" not in page
