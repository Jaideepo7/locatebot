import json
import math

import pytest

from locatebot.agent.tools import (
    MAX_ITEM_CHARS,
    MAX_ITEMS,
    InvalidSkillCall,
    parse_tool_call,
    tool_definitions,
    validate_skill_call,
)
from locatebot.mission.types import SKILL_ARGS, SkillCall, SkillName


def test_one_tool_per_skill_with_json_schema():
    tools = tool_definitions()
    assert [t["name"] for t in tools] == [s.value for s in SkillName] + ["continue"]
    json.dumps(tools)  # serialisable
    for t in tools:
        assert set(t) == {"name", "description", "input_schema"}
        assert len(t["description"]) > 30
        schema = t["input_schema"]
        assert schema["type"] == "object"
        assert schema["additionalProperties"] is False
        assert "reason" in schema["properties"]
        if t["name"] == "continue":
            continue  # not a skill: keeps the running skill going
        skill = SkillName(t["name"])
        for arg in SKILL_ARGS[skill]:
            assert arg in schema["properties"]
        assert set(schema.get("required", [])) == {a for a, (_, req) in SKILL_ARGS[skill].items() if req}
        assert "reason" in schema["properties"]
    by = {t["name"]: t for t in tools}
    items = by["search"]["input_schema"]["properties"]["items"]
    assert items["type"] == "array" and items["minItems"] == 1 and items["maxItems"] == MAX_ITEMS
    assert items["items"]["type"] == "string"
    assert by["go_to"]["input_schema"]["properties"]["x"]["type"] == "number"
    assert "short" in by["search"]["description"].lower()


def test_definitions_are_fresh_copies():
    a = tool_definitions()
    a[0]["name"] = "hacked"
    assert tool_definitions()[0]["name"] == "explore"


def test_parse_simple_and_reason():
    c = parse_tool_call("stop", {})
    assert c == SkillCall(SkillName.STOP, {}, "", "llm")
    c = parse_tool_call("explore", {"reason": "map  is\nsmall"}, source="fallback")
    assert c.reason == "map is small" and c.source == "fallback"
    assert parse_tool_call("look_around", None).name is SkillName.LOOK_AROUND


@pytest.mark.parametrize("name", ["", "drive", "STOP", None, 3])
def test_unknown_skill(name):
    with pytest.raises(InvalidSkillCall):
        parse_tool_call(name, {})


def test_args_must_be_object():
    with pytest.raises(InvalidSkillCall):
        parse_tool_call("stop", ["x"])
    with pytest.raises(InvalidSkillCall):
        parse_tool_call("stop", {1: "x"})


def test_unknown_and_missing_args():
    with pytest.raises(InvalidSkillCall, match="unknown"):
        parse_tool_call("stop", {"speed": 1})
    with pytest.raises(InvalidSkillCall, match="missing"):
        parse_tool_call("follow", {})
    with pytest.raises(InvalidSkillCall, match="missing"):
        parse_tool_call("search", {})


def test_go_to_forms():
    assert parse_tool_call("go_to", {"item": " marker "}).args == {"item": "marker"}
    c = parse_tool_call("go_to", {"x": 1, "y": -2.5})
    assert c.args == {"x": 1.0, "y": -2.5} and isinstance(c.args["x"], float)
    for bad in ({}, {"x": 1.0}, {"y": 1.0}, {"item": "a", "x": 1.0, "y": 2.0}, {"item": "a", "x": 1.0}):
        with pytest.raises(InvalidSkillCall):
            parse_tool_call("go_to", bad)


@pytest.mark.parametrize("v", [True, "1.0", None, math.nan, math.inf, -math.inf, 1e9, [1]])
def test_bad_numbers(v):
    with pytest.raises(InvalidSkillCall):
        parse_tool_call("go_to", {"x": v, "y": 0.0})


@pytest.mark.parametrize("v", ["", "   ", 5, None, "x" * (MAX_ITEM_CHARS + 1), "a\x00b", ["marker"]])
def test_bad_item_strings(v):
    with pytest.raises(InvalidSkillCall):
        parse_tool_call("follow", {"item": v})


def test_search_items():
    c = parse_tool_call("search", {"items": ["marker", " bolt", "marker"]})
    assert c.args == {"items": ["marker", "bolt"]}
    for bad in ([], "marker", ["ok", ""], ["ok", 3], [f"i{k}" for k in range(MAX_ITEMS + 1)], None):
        with pytest.raises(InvalidSkillCall):
            parse_tool_call("search", {"items": bad})


def test_reason_must_be_string_and_is_capped():
    with pytest.raises(InvalidSkillCall):
        parse_tool_call("stop", {"reason": 5})
    assert len(parse_tool_call("stop", {"reason": "y" * 999}).reason) <= 200


def test_validate_skill_call_round_trip():
    c = validate_skill_call(SkillCall(SkillName.GO_TO, {"x": 2, "y": 3}, "op", "operator"))
    assert c == SkillCall(SkillName.GO_TO, {"x": 2.0, "y": 3.0}, "op", "operator")
    with pytest.raises(InvalidSkillCall):
        validate_skill_call(SkillCall(SkillName.FOLLOW, {}))
