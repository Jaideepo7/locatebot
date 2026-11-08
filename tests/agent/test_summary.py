import math

from locatebot.agent.summary import MAX_ITEMS_LISTED, format_bearing, render_world_state
from locatebot.mission.types import SkillCall, SkillName, SkillState

from .helpers import item, ws


def test_header_line():
    text = render_world_state(ws())
    first = text.splitlines()[1]  # line 0 is the goal
    assert first.startswith("pose: (2.1, 0.8) m, heading 40°")
    assert "explored: 72 %" in first
    assert "mission: searching" in first
    assert "safety: ok" in first


def test_explored_unknown_before_map():
    assert "explored: unknown" in render_world_state(ws(explored=None))


def test_eight_named_sectors_and_nan_unknown():
    line = [l for l in render_world_state(ws()).splitlines() if l.startswith("around me:")][0]
    assert line == (
        "around me: front 1.4 m | front-left 0.5 m | left 2.2 m | back-left unknown | "
        "back 3.0 m | back-right 2.5 m | right 1.0 m | front-right 0.8 m"
    )


def test_inf_is_unknown_too():
    line = render_world_state(ws(sectors=(math.inf, 1, 1, 1, 1, 1, 1, 1))).splitlines()[2]
    assert line.startswith("around me: front unknown |")


def test_other_sector_counts_use_degrees():
    line = render_world_state(ws(sectors=(1.0, 2.0, float("nan"), 3.0))).splitlines()[2]
    assert line == "around me: front 1.0 m | 90° left 2.0 m | back unknown | 90° right 3.0 m"
    line6 = render_world_state(ws(sectors=(1,) * 6)).splitlines()[2]
    assert "60° left" in line6 and "120° right" in line6 and "back" in line6


def test_no_sectors():
    assert "around me: no lidar data" in render_world_state(ws(sectors=()))


def test_bearings():
    assert format_bearing(math.radians(30)) == "30° left"
    assert format_bearing(math.radians(-45)) == "45° right"
    assert format_bearing(math.radians(2)) == "ahead"
    assert format_bearing(math.radians(-170)) == "behind (170° right)"
    assert format_bearing(math.radians(160)) == "behind (160° left)"
    assert format_bearing(math.radians(350)) == "10° right"  # wraps to -10


def test_items_lines_and_order():
    text = render_world_state(ws(items=[
        item("bolt", confirmed=False, sightings=1, range_m=3.2, bearing_deg=-170, age_s=40),
        item("marker", confirmed=True, sightings=5, range_m=1.8, bearing_deg=30, age_s=4),
    ]))
    lines = text.splitlines()
    i = next(k for k, l in enumerate(lines) if l.startswith("items:"))
    assert lines[i] == "items: marker   confirmed (5 sightings), 1.8 m, 30° left, seen 4 s ago"
    assert lines[i + 1] == "       bolt     unconfirmed (1 sighting), 3.2 m, behind (170° right), seen 40 s ago"


def test_items_capped_and_minutes():
    many = [item(f"thing{k}", confirmed=False, sightings=1, age_s=10 + k) for k in range(MAX_ITEMS_LISTED + 3)]
    many.append(item("old", age_s=300))
    text = render_world_state(ws(items=many))
    assert "(+4 more items)" in text
    assert sum(1 for l in text.splitlines() if "sighting" in l) == MAX_ITEMS_LISTED
    assert "seen 5 min ago" in text  # confirmed 'old' ranks first


def test_no_items_and_search_list():
    text = render_world_state(ws(search_items=("marker", "bolt")))
    assert "items: none seen" in text
    assert "looking for: marker, bolt" in text


def test_skill_and_event_lines():
    call = SkillCall(SkillName.GO_TO, {"item": "marker"})
    text = render_world_state(ws(state=SkillState.RUNNING, call=call, detail="path 2.1 m",
                                 event="blocked while driving to bolt"))
    assert "skill: running go_to(item=marker) — path 2.1 m" in text
    assert text.splitlines()[-1] == "last event: blocked while driving to bolt"
    xy = SkillCall(SkillName.GO_TO, {"x": 1.0, "y": -2.25})
    assert "go_to(x=1.0, y=-2.2)" in render_world_state(ws(state=SkillState.RUNNING, call=xy)) or \
        "go_to(x=1.0, y=-2.3)" in render_world_state(ws(state=SkillState.RUNNING, call=xy))
    s = SkillCall(SkillName.SEARCH, {"items": ["marker", "bolt"]})
    assert "search(items=[marker, bolt])" in render_world_state(ws(state=SkillState.FAILED, call=s))
    assert "skill: idle" in render_world_state(ws())
    assert "last event: none" in render_world_state(ws())


def test_deterministic_bounded_and_single_line_fields():
    w = ws(items=[item(f"n{k}" * 30) for k in range(50)], event="a\nb" * 500, detail="x" * 1000,
           state=SkillState.RUNNING, call=SkillCall(SkillName.EXPLORE))
    a, b = render_world_state(w), render_world_state(w)
    assert a == b
    assert len(a) < 2500
    assert all(len(l) < 300 for l in a.splitlines())
    assert "last event: a b" in a


def test_goal_is_the_first_line():
    from dataclasses import replace
    assert render_world_state(ws()).splitlines()[0] == "goal: none"
    text = render_world_state(replace(ws(), goal="find the bolt"))
    assert text.splitlines()[0] == "goal: find the bolt"


def test_goal_is_one_line_and_truncated():
    from dataclasses import replace
    from locatebot.agent.summary import MAX_GOAL_CHARS
    first = render_world_state(replace(ws(), goal="find\nthe   bolt " + "x" * 500)).splitlines()[0]
    assert first.startswith("goal: find the bolt x")
    assert len(first) == len("goal: ") + MAX_GOAL_CHARS and first.endswith("…")
