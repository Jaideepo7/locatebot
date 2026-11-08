"""Builders for agent tests."""
from __future__ import annotations

import math

from locatebot.lidar.types import Pose2D
from locatebot.mission.types import ItemView, SkillCall, SkillName, SkillState, SkillStatus, WorldState


def item(name="marker", *, confirmed=True, sightings=5, range_m=1.8, bearing_deg=30.0, age_s=4.0, x=1.0, y=2.0):
    return ItemView(
        name=name, confirmed=confirmed, sightings=sightings, x=x, y=y, sigma_m=0.1,
        range_m=range_m, bearing_rad=math.radians(bearing_deg), last_seen_age_s=age_s,
    )


def ws(*, stamp=0.0, state=SkillState.IDLE, call=None, detail="", items=(), sectors=None,
       safety="ok", event="", mission="searching", explored=0.72, search_items=(), goal=""):
    if sectors is None:
        sectors = (1.4, 0.5, 2.2, float("nan"), 3.0, 2.5, 1.0, 0.8)
    return WorldState(
        stamp_s=stamp,
        pose=Pose2D(2.1, 0.8, math.radians(40)),
        sector_clearance_m=tuple(sectors),
        explored_frac=explored,
        items=tuple(items),
        skill=SkillStatus(state=state, call=call, detail=detail),
        search_items=tuple(search_items),
        safety_reason=safety,
        mission_state=mission,
        last_event=event,
        goal=goal,
    )


STOP = SkillCall(SkillName.STOP)
