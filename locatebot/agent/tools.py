"""Skills as LLM tools: generated JSON-Schema definitions and strict validation of calls.

``tool_definitions()`` returns provider-neutral dicts ``{"name", "description", "input_schema"}``
(the Anthropic Messages API shape; ``llm.OpenAICompatClient`` wraps them as functions).
``parse_tool_call`` is the only way an LLM tool call becomes a ``SkillCall``.
"""
from __future__ import annotations

import copy
import math
from typing import Any, Mapping

from locatebot.mission.types import SKILL_ARGS, SkillCall, SkillName

MAX_ITEMS = 8            # items per search call
MAX_ITEM_CHARS = 40      # characters per item name
MAX_REASON_CHARS = 200
MAX_COORD_M = 100.0      # |x|, |y| bound for go_to targets (indoor map frame)


class InvalidSkillCall(ValueError):
    """A tool call that does not name a skill or does not match its argument schema."""


_DESCRIPTIONS: dict[SkillName, str] = {
    SkillName.EXPLORE: (
        "Drive to unexplored frontiers to grow the map until it stops growing. Use when the map "
        "is small or the items you need may be in parts of the room not seen yet."
    ),
    SkillName.SEARCH: (
        "Patrol the known map while the camera looks for the named items. The vision model only "
        "finds what you name: give each item as a short common noun (\"marker\", \"bolt\", "
        "\"remote\"), not a long description."
    ),
    SkillName.GO_TO: (
        "Drive to a tracked item (give `item`, the name as listed under items) or to a map "
        "position in metres (give both `x` and `y`, map frame). Give either item or x+y, never both."
    ),
    SkillName.FOLLOW: (
        "Keep a set distance behind a moving tracked item (`item`, the name as listed under "
        "items). Ends if the item is lost."
    ),
    SkillName.LOOK_AROUND: (
        "Turn slowly in place so the camera sweeps the surroundings. Use to re-find an item or "
        "check the area before moving on."
    ),
    SkillName.STOP: "Stop and hold position. Use when unsure, when blocked repeatedly, or when the task is done.",
    SkillName.RETURN_HOME: "Drive back to the pose where the session started.",
}

_ITEM_SCHEMA = {
    "type": "string",
    "minLength": 1,
    "maxLength": MAX_ITEM_CHARS,
    "description": "Short common noun naming the item, e.g. \"marker\".",
}
_REASON_SCHEMA = {
    "type": "string",
    "maxLength": MAX_REASON_CHARS,
    "description": "One short sentence: why this skill now (shown to the operator).",
}


def _arg_schema(skill: SkillName, arg: str, typ: type) -> dict:
    if typ is str:
        return dict(_ITEM_SCHEMA)
    if typ is float:
        return {"type": "number", "description": f"Map-frame {arg} in metres."}
    if typ is list:
        return {
            "type": "array",
            "items": dict(_ITEM_SCHEMA),
            "minItems": 1,
            "maxItems": MAX_ITEMS,
            "description": "Items to look for, each a short common noun.",
        }
    raise TypeError(f"unsupported arg type {typ!r} for {skill.value}.{arg}")


def _build_definitions() -> list[dict]:
    tools = []
    for skill in SkillName:
        spec = SKILL_ARGS[skill]
        props = {arg: _arg_schema(skill, arg, typ) for arg, (typ, _) in spec.items()}
        props["reason"] = dict(_REASON_SCHEMA)
        schema: dict[str, Any] = {"type": "object", "properties": props, "additionalProperties": False}
        required = [arg for arg, (_, req) in spec.items() if req]
        if required:
            schema["required"] = required
        tools.append({"name": skill.value, "description": _DESCRIPTIONS[skill], "input_schema": schema})
    return tools


_TOOLS = _build_definitions()


def tool_definitions() -> list[dict]:
    """Fresh copy of the tool list, one tool per ``SkillName`` in enum order."""
    return copy.deepcopy(_TOOLS)


def _clean_text(v: Any, what: str, limit: int) -> str:
    if not isinstance(v, str):
        raise InvalidSkillCall(f"{what} must be a string, got {type(v).__name__}")
    if any(ord(ch) < 32 and ch not in "\t\n\r" for ch in v):
        raise InvalidSkillCall(f"{what} contains control characters")
    s = " ".join(v.split())
    if not s:
        raise InvalidSkillCall(f"{what} is empty")
    if len(s) > limit:
        raise InvalidSkillCall(f"{what} longer than {limit} characters")
    return s


def _number(v: Any, what: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise InvalidSkillCall(f"{what} must be a number, got {type(v).__name__}")
    f = float(v)
    if not math.isfinite(f):
        raise InvalidSkillCall(f"{what} must be finite")
    if abs(f) > MAX_COORD_M:
        raise InvalidSkillCall(f"{what} outside ±{MAX_COORD_M:g} m")
    return f


def _items(v: Any) -> list[str]:
    if not isinstance(v, (list, tuple)):
        raise InvalidSkillCall(f"items must be a list of strings, got {type(v).__name__}")
    if not v:
        raise InvalidSkillCall("items is empty")
    if len(v) > MAX_ITEMS:
        raise InvalidSkillCall(f"more than {MAX_ITEMS} items")
    out: list[str] = []
    for k, x in enumerate(v):
        s = _clean_text(x, f"items[{k}]", MAX_ITEM_CHARS)
        if s not in out:
            out.append(s)
    return out


def parse_tool_call(name: Any, args: Any, source: str = "llm", reason: str = "") -> SkillCall:
    """Validate a tool call strictly and turn it into a ``SkillCall``.

    Raises ``InvalidSkillCall`` for an unknown skill, a non-object ``args``, unknown or missing
    arguments, wrong types (ints are accepted for floats, bools are not), non-finite or
    out-of-range numbers, empty/over-long/control-character strings, and a ``go_to`` that does
    not give exactly ``item`` or both ``x`` and ``y``. The optional ``reason`` argument becomes
    ``SkillCall.reason`` (it overrides the ``reason`` parameter).
    """
    if not isinstance(name, str):
        raise InvalidSkillCall(f"tool name must be a string, got {type(name).__name__}")
    try:
        skill = SkillName(name)
    except ValueError:
        raise InvalidSkillCall(f"unknown skill {name[:40]!r}") from None
    if args is None:
        args = {}
    if not isinstance(args, Mapping):
        raise InvalidSkillCall(f"arguments must be an object, got {type(args).__name__}")
    if any(not isinstance(k, str) for k in args):
        raise InvalidSkillCall("argument names must be strings")
    spec = SKILL_ARGS[skill]
    unknown = sorted(set(args) - set(spec) - {"reason"})
    if unknown:
        raise InvalidSkillCall(f"{skill.value}: unknown argument(s) {', '.join(u[:20] for u in unknown)}")
    missing = [a for a, (_, req) in spec.items() if req and a not in args]
    if missing:
        raise InvalidSkillCall(f"{skill.value}: missing argument(s) {', '.join(missing)}")

    out: dict[str, Any] = {}
    for arg, (typ, _) in spec.items():
        if arg not in args:
            continue
        v = args[arg]
        if typ is str:
            out[arg] = _clean_text(v, arg, MAX_ITEM_CHARS)
        elif typ is float:
            out[arg] = _number(v, arg)
        elif typ is list:
            out[arg] = _items(v)
        else:  # pragma: no cover - guarded by _build_definitions at import
            raise InvalidSkillCall(f"unsupported argument type for {arg}")

    if skill is SkillName.GO_TO:
        has_item = "item" in out
        has_x, has_y = "x" in out, "y" in out
        if has_item == (has_x or has_y) or has_x != has_y:
            raise InvalidSkillCall("go_to needs exactly `item` or both `x` and `y`")

    if "reason" in args:
        r = args["reason"]
        if not isinstance(r, str):
            raise InvalidSkillCall("reason must be a string")
        reason = r
    reason = " ".join(str(reason).split())[:MAX_REASON_CHARS]
    return SkillCall(name=skill, args=out, reason=reason, source=source)


def validate_skill_call(call: SkillCall) -> SkillCall:
    """Re-validate a ``SkillCall`` built in code (operator, fallback); returns a normalised copy."""
    if not isinstance(call, SkillCall):
        raise InvalidSkillCall(f"expected SkillCall, got {type(call).__name__}")
    name = getattr(call.name, "value", call.name)
    return parse_tool_call(name, dict(call.args or {}), source=call.source, reason=call.reason)
