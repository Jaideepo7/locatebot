"""WorldState -> compact, deterministic text for the LLM.

Output is bounded: at most ``MAX_ITEMS_LISTED`` item lines, free-text fields are flattened to
one line and truncated. Directions are in the robot frame (bearing CCW positive = left).
"""
from __future__ import annotations

import math
from typing import Any, Mapping

from locatebot.mission.types import ChangeView, ItemView, PersonView, SkillCall, SkillState, WorldState

MAX_ITEMS_LISTED = 6
MAX_CHANGES_LISTED = 3
MAX_PEOPLE_LISTED = 3
MAX_NOTES_LISTED = 4
MAX_TEXT_CHARS = 120
MAX_NAME_CHARS = 24
AHEAD_DEG = 5.0      # |bearing| at or below this reads "ahead"
BEHIND_DEG = 135.0   # |bearing| at or above this reads "behind (...)"

# detect.sector_clearance layout for 8 sectors: sector 0 centred on forward, then CCW.
SECTOR_NAMES_8 = ("front", "front-left", "left", "back-left", "back", "back-right", "right", "front-right")


def one_line(text: Any, limit: int = MAX_TEXT_CHARS) -> str:
    """Collapse whitespace/newlines/control chars and truncate to ``limit`` characters."""
    s = "".join(ch if ch.isprintable() else " " for ch in str(text))
    s = " ".join(s.split())
    if len(s) > limit:
        s = s[: max(0, limit - 1)] + "…"
    return s


def _signed_deg(rad: float) -> float:
    d = math.degrees(rad) % 360.0
    return d - 360.0 if d > 180.0 else d  # (-180, 180]


def format_bearing(bearing_rad: float) -> str:
    """'ahead', '30° left', '45° right' or 'behind (170° right)'."""
    if not math.isfinite(bearing_rad):
        return "unknown direction"
    d = _signed_deg(bearing_rad)
    a = abs(d)
    if a <= AHEAD_DEG:
        return "ahead"
    side = "left" if d > 0 else "right"
    if a >= 179.5:
        return "behind"
    if a >= BEHIND_DEG:
        return f"behind ({a:.0f}° {side})"
    return f"{a:.0f}° {side}"


def sector_labels(n: int) -> list[str]:
    if n == 8:
        return list(SECTOR_NAMES_8)
    labels = []
    for k in range(n):
        d = _signed_deg(2.0 * math.pi * k / n)
        a = abs(d)
        if a < 0.5:
            labels.append("front")
        elif a > 179.5:
            labels.append("back")
        else:
            labels.append(f"{a:.0f}° {'left' if d > 0 else 'right'}")
    return labels


def _fmt_range(r: float) -> str:
    try:
        r = float(r)
    except (TypeError, ValueError):
        return "unknown"
    return f"{r:.1f} m" if math.isfinite(r) else "unknown"  # NaN/inf = unknown, never free


def _fmt_age(age_s: float) -> str:
    if not math.isfinite(age_s) or age_s < 0:
        return "seen ? ago"
    if age_s < 120:
        return f"seen {age_s:.0f} s ago"
    if age_s < 7200:
        return f"seen {age_s / 60:.0f} min ago"
    return f"seen {age_s / 3600:.0f} h ago"


def _fmt_num(v: Any) -> str:
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, (int, float)):
        return f"{float(v):.1f}"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(one_line(x, MAX_NAME_CHARS) for x in v[:10]) + "]"
    return one_line(v, MAX_NAME_CHARS * 2)


def format_call(call: SkillCall | None) -> str:
    """'go_to(item=marker)', 'go_to(x=1.0, y=2.0)', 'search(items=[a, b])', 'explore()'."""
    if call is None:
        return "none"
    name = getattr(call.name, "value", str(call.name))
    args: Mapping[str, Any] = call.args or {}
    inner = ", ".join(f"{k}={_fmt_num(args[k])}" for k in sorted(args, key=_arg_order))
    return f"{name}({inner})"


def _arg_order(k: str) -> tuple[int, str]:
    order = {"item": 0, "items": 0, "x": 1, "y": 2}
    return (order.get(k, 9), k)


def _item_sort_key(it: ItemView) -> tuple:
    age = it.last_seen_age_s if math.isfinite(it.last_seen_age_s) else math.inf
    return (not it.confirmed, age, it.name)


def _items_lines(items: tuple[ItemView, ...]) -> list[str]:
    if not items:
        return ["items: none seen"]
    shown = sorted(items, key=_item_sort_key)[:MAX_ITEMS_LISTED]
    names = [one_line(it.name, MAX_NAME_CHARS) for it in shown]
    width = max(len(n) for n in names)
    lines = []
    for k, (it, name) in enumerate(zip(shown, names)):
        status = "confirmed" if it.confirmed else "unconfirmed"
        s = "sighting" if it.sightings == 1 else "sightings"
        body = (f"{name.ljust(width)}   {status} ({it.sightings} {s}), {_fmt_range(it.range_m)}, "
                f"{format_bearing(it.bearing_rad)}, {_fmt_age(it.last_seen_age_s)}")
        lines.append(("items: " if k == 0 else "       ") + body)
    extra = len(items) - len(shown)
    if extra > 0:
        lines.append(f"       (+{extra} more items)")
    return lines


def _changes_line(changes: tuple[ChangeView, ...]) -> list[str]:
    """'changes: 2 new things: 0.4 m wide at 1.2 m ahead (moving), ...'; nothing when empty."""
    if not changes:
        return []
    shown = sorted(changes, key=lambda c: c.range_m if math.isfinite(c.range_m) else math.inf)
    parts = [f"{c.width_m:.1f} m wide at {_fmt_range(c.range_m)} {format_bearing(c.bearing_rad)}"
             + (" (moving)" if c.moving else "") for c in shown[:MAX_CHANGES_LISTED]]
    more = len(changes) - len(parts)
    n = len(changes)
    return [f"changes: {n} new thing{'s' if n != 1 else ''}: " + ", ".join(parts)
            + (f" (+{more} more)" if more > 0 else "")]


def _person_text(p: PersonView) -> str:
    where = f"{_fmt_range(p.range_m)} {format_bearing(p.bearing_rad)}"
    tags = []
    if p.moving:
        tags.append(f"{p.speed_mps:.1f} m/s")
    tags.append("camera-confirmed" if p.confirmed_by_camera else "lidar only")
    return f"{where} {'walking' if p.moving else 'standing'} ({', '.join(tags)})"


def _people_line(people: tuple[PersonView, ...]) -> list[str]:
    """'people: 1 person 1.2 m ahead walking (0.6 m/s, camera-confirmed)'; 'people: none'."""
    if not people:
        return ["people: none"]
    shown = sorted(people, key=lambda p: p.range_m if math.isfinite(p.range_m) else math.inf)
    n = len(people)
    head = "1 person" if n == 1 else f"{n} people:"
    parts = [_person_text(p) for p in shown[:MAX_PEOPLE_LISTED]]
    more = n - len(parts)
    return [f"people: {head} " + ", ".join(parts) + (f" (+{more} more)" if more > 0 else "")]


def _notes_lines(notes: tuple[str, ...]) -> list[str]:
    return [one_line(n) for n in notes[:MAX_NOTES_LISTED] if one_line(n)]


def render_world_state(ws: WorldState) -> str:
    p = ws.pose
    heading = _signed_deg(p.theta)
    heading = 0.0 if abs(heading) < 0.5 else heading
    if ws.explored_frac is None or not math.isfinite(ws.explored_frac):
        explored = "unknown"
    else:
        explored = f"{100.0 * min(max(ws.explored_frac, 0.0), 1.0):.0f} %"
    lines = [
        f"pose: ({p.x:.1f}, {p.y:.1f}) m, heading {heading:.0f}°   explored: {explored}   "
        f"mission: {one_line(ws.mission_state, 40)}   safety: {one_line(ws.safety_reason, 40)}"
    ]
    sectors = tuple(ws.sector_clearance_m)
    if sectors:
        parts = [f"{lab} {_fmt_range(r)}" for lab, r in zip(sector_labels(len(sectors)), sectors)]
        lines.append("around me: " + " | ".join(parts))
    else:
        lines.append("around me: no lidar data")
    if ws.search_items:
        shown = [one_line(s, MAX_NAME_CHARS) for s in ws.search_items[:MAX_ITEMS_LISTED]]
        more = len(ws.search_items) - len(shown)
        lines.append("looking for: " + ", ".join(shown) + (f" (+{more} more)" if more > 0 else ""))
    lines.extend(_items_lines(tuple(ws.items)))
    lines.extend(_people_line(tuple(getattr(ws, "people", ()))))
    lines.extend(_changes_line(tuple(getattr(ws, "changes", ()))))
    lines.extend(_notes_lines(tuple(getattr(ws, "notes", ()))))
    sk = ws.skill
    state = getattr(sk.state, "value", str(sk.state))
    if sk.call is None and sk.state == SkillState.IDLE:
        skill = "skill: idle"
    else:
        skill = f"skill: {state} {format_call(sk.call)}"
        if sk.call is not None and sk.call.source != "llm":
            skill += f" [by {one_line(sk.call.source, 12)}]"
    if sk.detail:
        skill += f" — {one_line(sk.detail)}"
    lines.append(skill)
    lines.append(f"last event: {one_line(ws.last_event) or 'none'}")
    return "\n".join(lines)
