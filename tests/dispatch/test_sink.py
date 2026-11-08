import pytest

from locatebot.dispatch.protocol import MissionState as S
from locatebot.dispatch.sink import SCRIPTS, MissionRejected, ScriptedSink

MID = "3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70"


class Clock:
    t = 1000.0

    def __call__(self):
        return self.t


def run(script, seconds, step=0.5, **kw):
    clock, got = Clock(), []
    sink = ScriptedSink(SCRIPTS[script], clock=clock, **kw)
    sink.start(MID, "blue toolbox", lambda s, m="", r=None: got.append((s, m, r)))
    for _ in range(int(seconds / step)):
        clock.t += step
        sink.poll()
    return sink, got, clock


def test_happy_script_plays_full_mission():
    _, got, _ = run("happy", 10)
    assert [g[0] for g in got] == [S.SEARCHING, S.TARGET_DETECTED, S.APPROACHING, S.COMPLETED]
    assert got[0][1] == "Searching for blue toolbox"


def test_steps_wait_for_their_time():
    _, got, _ = run("happy", 0.5)
    assert got == []


def test_not_found_script_fails_without_detection():
    _, got, _ = run("not_found", 10)
    assert [g[0] for g in got] == [S.SEARCHING, S.FAILED]
    assert got[-1][2] == "NOT_FOUND"


def test_reject_script_refuses_start():
    with pytest.raises(MissionRejected) as e:
        ScriptedSink(SCRIPTS["reject"], clock=Clock()).start(MID, "x y", lambda *a: None)
    assert e.value.reason == "BUSY"


def test_stall_script_stops_after_searching():
    _, got, _ = run("stall", 60)
    assert [g[0] for g in got] == [S.SEARCHING]


def test_cancel_confirms_and_stops_script():
    sink, got, clock = run("stall", 2)
    sink.cancel(MID)
    clock.t += 1
    sink.poll()
    assert got[-1][0] == S.CANCELLED
    n = len(got)
    for _ in range(20):
        clock.t += 1
        sink.poll()
    assert len(got) == n


def test_link_lost_stops_safely_and_reports_reason():
    sink, got, clock = run("happy", 1.5)
    sink.on_link_lost(MID)
    assert got[-1] == (S.CANCELLED, "Stopped: lost contact with the server", "LINK_LOST")
    for _ in range(20):
        clock.t += 1
        sink.poll()
    assert got[-1][0] == S.CANCELLED


def test_cancel_for_other_mission_ignored():
    sink, got, _ = run("stall", 2)
    sink.cancel("other")
    sink.poll()
    assert got[-1][0] == S.SEARCHING
