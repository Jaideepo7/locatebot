import threading
import time

import pytest

from locatebot.agent.brain import DEFAULT_SYSTEM_PROMPT, Brain
from locatebot.agent.llm import FakeLLM, LLMError, ToolChoice
from locatebot.mission.types import SkillCall, SkillName, SkillState

from .helpers import STOP, item, ws

GO_MARKER = ToolChoice("go_to", {"item": "marker"}, "drive to it")


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(script=(), *, mode="auto", default=None, **kw):
    llm = FakeLLM(list(script), default=default)
    clk = Clock()
    emitted = []
    kw.setdefault("llm_timeout_s", 2.0)
    kw.setdefault("decide_interval_s", 5.0)
    kw.setdefault("max_failures", 2)
    kw.setdefault("retry_delay_s", 0.5)
    b = Brain(llm, mode=mode, on_skill=emitted.append, clock=clk, **kw)
    return b, llm, clk, emitted


def settle(b):
    assert b.wait_idle(2.0)


def test_default_prompt_mentions_key_rules():
    p = DEFAULT_SYSTEM_PROMPT.lower()
    for word in ("skill", "safety", "short", "marker", "noun"):
        assert word in p


def test_auto_emits_valid_call_and_waits_while_running():
    b, llm, clk, out = make([GO_MARKER])
    b.on_world_state(ws(stamp=1))
    settle(b)
    assert out == [SkillCall(SkillName.GO_TO, {"item": "marker"}, "drive to it", "llm")]
    system, user, tools, timeout = llm.calls[0]
    assert system == DEFAULT_SYSTEM_PROMPT and "pose:" in user and timeout == 2.0
    assert {t["name"] for t in tools} >= {"go_to", "stop"}
    running = ws(stamp=2, state=SkillState.RUNNING, call=out[0])
    for k in range(5):
        clk.t += 10
        b.on_world_state(running)
    settle(b)
    assert len(llm.calls) == 1  # no periodic decisions while a skill runs
    llm.script.append(ToolChoice("look_around", {}))
    b.on_world_state(ws(stamp=3, state=SkillState.SUCCEEDED, call=out[0], detail="arrived"))
    settle(b)
    assert len(llm.calls) == 2 and out[-1].name is SkillName.LOOK_AROUND
    # the prompt carries the decision history including the outcome of the first call
    assert "go_to(item=marker)" in llm.calls[1][1] and "succeeded" in llm.calls[1][1]
    st = b.status()
    assert st["mode"] == "auto" and st["failures"] == 0 and st["in_fallback"] is False
    assert st["last_latency_s"] is not None


def test_periodic_decision_when_idle():
    b, llm, clk, out = make([ToolChoice("stop", {}), ToolChoice("explore", {})])
    b.on_world_state(ws(stamp=1))
    settle(b)
    clk.t += 4.0
    b.on_world_state(ws(stamp=2))
    settle(b)
    assert len(llm.calls) == 1
    clk.t += 1.5
    b.on_world_state(ws(stamp=3))
    settle(b)
    assert len(llm.calls) == 2


@pytest.mark.parametrize("change", [
    dict(items=[item("marker", confirmed=True)]),
    dict(safety="obstacle"),
    dict(event="blocked"),
])
def test_event_triggers(change):
    run_call = SkillCall(SkillName.EXPLORE)
    b, llm, clk, out = make(default=ToolChoice("explore", {}))
    b.on_world_state(ws(stamp=1, state=SkillState.RUNNING, call=run_call))
    settle(b)
    n = len(llm.calls)
    b.on_world_state(ws(stamp=2, state=SkillState.RUNNING, call=run_call))
    settle(b)
    assert len(llm.calls) == n
    b.on_world_state(ws(stamp=3, state=SkillState.RUNNING, call=run_call, **change))
    settle(b)
    assert len(llm.calls) == n + 1


def test_unconfirmed_item_does_not_trigger():
    run_call = SkillCall(SkillName.EXPLORE)
    b, llm, clk, out = make(default=ToolChoice("explore", {}))
    b.on_world_state(ws(stamp=1, state=SkillState.RUNNING, call=run_call))
    settle(b)
    n = len(llm.calls)
    b.on_world_state(ws(stamp=2, state=SkillState.RUNNING, call=run_call, items=[item(confirmed=False)]))
    settle(b)
    assert len(llm.calls) == n


def test_approve_mode_flow():
    b, llm, clk, out = make([GO_MARKER, ToolChoice("explore", {})], mode="approve")
    b.on_world_state(ws(stamp=1))
    settle(b)
    assert out == []
    assert b.pending().name is SkillName.GO_TO
    assert b.status()["pending"]["name"] == "go_to"
    call = b.approve()
    assert call == out[0] and b.pending() is None
    assert b.approve() is None  # nothing pending
    b.on_world_state(ws(stamp=2, state=SkillState.FAILED, call=call, detail="no path"))
    settle(b)
    assert b.pending().name is SkillName.EXPLORE
    assert b.reject() is True and b.pending() is None and b.reject() is False
    assert len(out) == 1
    hist = b.history()
    assert [h.outcome for h in hist][-2:] == ["approved", "rejected"] or "rejected" in hist[-1].outcome


def test_newer_proposal_replaces_pending():
    b, llm, clk, out = make([GO_MARKER, ToolChoice("look_around", {})], mode="approve")
    b.on_world_state(ws(stamp=1))
    settle(b)
    b.on_world_state(ws(stamp=2, event="found bolt"))
    settle(b)
    assert b.pending().name is SkillName.LOOK_AROUND
    assert any(h.outcome == "replaced" for h in b.history())


def test_hanging_llm_never_blocks_and_one_request_at_a_time():
    b, llm, clk, out = make(default=GO_MARKER, llm_timeout_s=1000)
    llm.hang.clear()
    try:
        t0 = time.monotonic()
        for k in range(200):
            b.on_world_state(ws(stamp=k, event=f"e{k}", safety="obstacle" if k % 2 else "ok"))
            clk.t += 1.0
        assert time.monotonic() - t0 < 0.5
        assert llm.started.wait(1.0)
        assert len(llm.calls) == 1 and llm.max_active == 1
        assert b.status()["inflight"] is True
    finally:
        llm.hang.set()
        settle(b)
    assert llm.max_active == 1


def test_timeouts_enter_fallback_then_routine_runs():
    look = SkillCall(SkillName.LOOK_AROUND)
    home = SkillCall(SkillName.RETURN_HOME)
    b, llm, clk, out = make(default=GO_MARKER, fallback_routine=[look, home])
    llm.hang.clear()
    try:
        b.on_world_state(ws(stamp=1))
        assert llm.started.wait(1.0)
        clk.t += 2.5
        b.on_world_state(ws(stamp=2))  # request 1 times out, retry not yet due
        st = b.status()
        assert st["failures"] == 1 and "timeout" in st["last_error"] and not st["in_fallback"]
        clk.t += 0.6
        b.on_world_state(ws(stamp=3))  # retry -> request 2
        clk.t += 2.5
        b.on_world_state(ws(stamp=4))  # request 2 times out -> fallback, first step emitted
        assert b.status()["in_fallback"] is True
        assert out == [SkillCall(SkillName.LOOK_AROUND, {}, "fallback routine step 1/2", "fallback")]
        b.on_world_state(ws(stamp=5, state=SkillState.RUNNING, call=out[0]))
        assert len(out) == 1
        b.on_world_state(ws(stamp=6, state=SkillState.SUCCEEDED, call=out[0]))
        assert out[-1].name is SkillName.RETURN_HOME and out[-1].source == "fallback"
        b.on_world_state(ws(stamp=7, state=SkillState.FAILED, call=out[-1]))
        b.on_world_state(ws(stamp=8))
        assert len(out) == 2  # routine finished, nothing more
        assert b.status()["fallback_step"] is None
        assert all(c.source == "fallback" for c in out)
    finally:
        llm.hang.set()
        b.wait_idle(2.0)
    time.sleep(0.05)
    assert all(c.source == "fallback" for c in out)  # late results of abandoned requests ignored


def test_errors_fallback_waits_for_running_skill_then_recovers():
    b, llm, clk, out = make([LLMError("down"), ToolChoice("fly", {}), None, GO_MARKER])
    run = SkillCall(SkillName.EXPLORE, {}, "", "llm")
    b.on_world_state(ws(stamp=1, state=SkillState.RUNNING, call=run, event="go"))
    settle(b)
    assert b.status()["failures"] == 1 and "down" in b.status()["last_error"]
    clk.t += 0.6
    b.on_world_state(ws(stamp=2, state=SkillState.RUNNING, call=run))
    settle(b)
    st = b.status()
    assert st["in_fallback"] is True and "invalid" in st["last_error"]
    b.on_world_state(ws(stamp=3, state=SkillState.RUNNING, call=run))
    assert out == []  # the current skill finishes first
    b.on_world_state(ws(stamp=4, state=SkillState.SUCCEEDED, call=run))
    settle(b)
    assert out == [SkillCall(SkillName.STOP, {}, "fallback routine step 1/1", "fallback")]
    # LLM probed again after decide_interval_s while in fallback: None -> still failing
    clk.t += 5.1
    b.on_world_state(ws(stamp=5, state=SkillState.SUCCEEDED, call=out[0]))
    settle(b)
    assert b.status()["in_fallback"] is True and "no tool" in b.status()["last_error"]
    clk.t += 5.1
    b.on_world_state(ws(stamp=6, state=SkillState.SUCCEEDED, call=out[0]))
    settle(b)
    st = b.status()
    assert st["in_fallback"] is False and st["failures"] == 0
    assert out[-1] == SkillCall(SkillName.GO_TO, {"item": "marker"}, "drive to it", "llm")


def test_no_llm_means_fallback():
    out = []
    b = Brain(None, mode="auto", on_skill=out.append, clock=Clock())
    b.on_world_state(ws(stamp=1))
    assert b.status()["in_fallback"] is True and out[0].name is SkillName.STOP


def test_operator_stop_wins():
    b, llm, clk, out = make(default=GO_MARKER, mode="approve")
    b.on_world_state(ws(stamp=1))
    settle(b)
    assert b.pending() is not None
    llm.hang.clear()
    b.on_world_state(ws(stamp=2, event="x"))
    assert llm.started.wait(1.0)
    b.submit_operator(STOP)
    assert out == [SkillCall(SkillName.STOP, {}, "", "operator")]
    assert b.pending() is None and b.status()["paused"] is True
    llm.hang.set()
    settle(b)
    assert b.pending() is None  # late proposal discarded
    n = len(llm.calls)
    for k in range(3):
        clk.t += 10
        b.on_world_state(ws(stamp=3 + k, event=f"e{k}"))
    settle(b)
    assert len(llm.calls) == n
    b.resume()
    b.on_world_state(ws(stamp=10))
    settle(b)
    assert len(llm.calls) == n + 1 and b.pending() is not None


def test_operator_stop_halts_fallback_and_auto():
    b, llm, clk, out = make(fallback_routine=[SkillCall(SkillName.LOOK_AROUND), STOP], max_failures=1,
                            default=None)
    b.on_world_state(ws(stamp=1))
    settle(b)
    b.on_world_state(ws(stamp=2))
    assert out[-1].name is SkillName.LOOK_AROUND
    b.submit_operator(STOP)
    b.on_world_state(ws(stamp=3, state=SkillState.SUCCEEDED, call=out[0]))
    assert out[-1] == SkillCall(SkillName.STOP, {}, "", "operator") and len(out) == 2


def test_operator_non_stop_call_emitted_and_validated():
    b, llm, clk, out = make(mode="approve", default=GO_MARKER)
    b.on_world_state(ws(stamp=1))
    settle(b)
    b.submit_operator(SkillCall(SkillName.GO_TO, {"x": 1, "y": 2}))
    assert out[-1] == SkillCall(SkillName.GO_TO, {"x": 1.0, "y": 2.0}, "", "operator")
    assert b.pending() is None and b.status()["paused"] is False
    from locatebot.agent.tools import InvalidSkillCall
    with pytest.raises(InvalidSkillCall):
        b.submit_operator(SkillCall(SkillName.FOLLOW, {}))


def test_history_is_bounded():
    b, llm, clk, out = make(default=ToolChoice("stop", {}), history_len=3)
    for k in range(6):
        b.on_world_state(ws(stamp=k, event=f"e{k}"))
        settle(b)
    assert len(b.history()) == 3


def test_on_skill_exception_does_not_break_brain():
    calls = []

    def bad(c):
        calls.append(c)
        raise RuntimeError("executor down")

    b = Brain(FakeLLM([], default=GO_MARKER), mode="auto", on_skill=bad, clock=Clock())
    b.on_world_state(ws(stamp=1))
    assert b.wait_idle(2.0)
    assert calls and "executor down" in b.status()["last_error"]


def test_bad_config():
    with pytest.raises(ValueError):
        Brain(None, mode="yolo")
    with pytest.raises(ValueError):
        Brain(None, max_failures=0)
    with pytest.raises(ValueError):
        Brain(None, fallback_routine=[])


# ---------------------------------------------------------------- "continue": keep the running skill
SEARCH = SkillCall(SkillName.SEARCH, {"items": ["marker", "bolt"]}, "", "llm")


def test_continue_while_running_is_a_success_and_emits_nothing():
    b, llm, clk, out = make([ToolChoice("continue", {}, "keep searching")], default=ToolChoice("continue", {}))
    running = ws(stamp=1, state=SkillState.RUNNING, call=SEARCH)
    b.on_world_state(running)
    b.on_world_state(ws(stamp=2, state=SkillState.RUNNING, call=SEARCH, items=(item("bolt", confirmed=True),)))
    settle(b)
    assert out == []
    st = b.status()
    assert st["failures"] == 0 and st["in_fallback"] is False
    assert any("continue" in h for h in (d if isinstance(d, str) else str(d) for d in st["history"]))


def test_continue_with_nothing_running_is_a_failure():
    b, llm, clk, out = make([ToolChoice("continue", {})])
    b.on_world_state(ws(stamp=1))
    settle(b)
    assert out == [] and b.status()["failures"] == 1
    assert "no skill is running" in b.status()["last_error"]


def test_recalling_the_running_skill_does_not_restart_it():
    b, llm, clk, out = make([ToolChoice("search", {"items": ["marker", "bolt"]})])
    b.on_world_state(ws(stamp=1, state=SkillState.RUNNING, call=SEARCH))
    settle(b)
    assert out == [] and b.status()["failures"] == 0


def test_continue_tool_is_offered_and_prompt_explains_it():
    b, llm, clk, out = make([GO_MARKER])
    b.on_world_state(ws(stamp=1))
    settle(b)
    assert "continue" in {t["name"] for t in llm.calls[0][2]}
    assert "`continue`" in DEFAULT_SYSTEM_PROMPT
