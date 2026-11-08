"""Shared mission contracts: WorldState extensions for goal, people, changes and notes."""
import dataclasses
import math

import pytest

from locatebot.lidar.types import Pose2D
from locatebot.mission.types import ChangeView, PersonView, SkillStatus, WorldState

OLD_FIELDS = ["stamp_s", "pose", "sector_clearance_m", "explored_frac", "items", "skill", "search_items",
              "safety_reason", "mission_state", "last_event"]


def person(**kw):
    base = dict(x=1.0, y=0.5, range_m=1.2, bearing_rad=0.3, speed_mps=0.6, moving=True,
                confirmed_by_camera=False, last_seen_age_s=0.2)
    base.update(kw)
    return PersonView(**base)


def change(**kw):
    base = dict(x=2.0, y=-0.4, width_m=0.4, range_m=1.2, bearing_rad=-0.1, moving=False, age_s=3.0)
    base.update(kw)
    return ChangeView(**base)


def test_new_fields_are_appended_with_defaults():
    names = [f.name for f in dataclasses.fields(WorldState)]
    assert names[:len(OLD_FIELDS)] == OLD_FIELDS
    assert names[len(OLD_FIELDS):] == ["goal", "people", "changes", "notes"]
    ws = WorldState(0.0, Pose2D(0, 0, 0), (1.0,) * 8, None, (), SkillStatus())  # old positional call
    assert ws.goal == "" and ws.people == () and ws.changes == () and ws.notes == ()


def test_world_state_carries_people_changes_notes_and_goal():
    ws = WorldState(1.0, Pose2D(0, 0, 0), (math.nan,) * 8, 0.5, (), SkillStatus(),
                    goal="find the blue toolbox", people=(person(),), changes=(change(),),
                    notes=("glass or low obstacle 0.6 m ahead (ultrasonic only)", "wifi weak here"))
    assert ws.people[0].moving and not ws.people[0].confirmed_by_camera
    assert ws.changes[0].width_m == 0.4
    assert ws.notes[1] == "wifi weak here" and ws.goal.startswith("find")


def test_views_have_the_contract_fields_and_are_frozen_and_hashable():
    assert [f.name for f in dataclasses.fields(PersonView)] == [
        "x", "y", "range_m", "bearing_rad", "speed_mps", "moving", "confirmed_by_camera", "last_seen_age_s"]
    assert [f.name for f in dataclasses.fields(ChangeView)] == [
        "x", "y", "width_m", "range_m", "bearing_rad", "moving", "age_s"]
    for v in (person(), change()):
        with pytest.raises(dataclasses.FrozenInstanceError):
            v.x = 9.0
        assert hash(v) == hash(dataclasses.replace(v))
    assert PersonView.__doc__ and ChangeView.__doc__
