"""Build the ``WorldState`` snapshot the LLM agent sees.

Combines the robot pose, the newest LiDAR scan (as per-sector clearances), the item tracks
from vision + LiDAR localization, the current skill and, in saved-map mode, the change blobs
(new/moving things vs the room map, M2). Pure function: no locks, no I/O.
"""
from __future__ import annotations

import math
from typing import Sequence

from locatebot.lidar.detect import sector_clearance
from locatebot.lidar.layers import ChangeBlob
from locatebot.lidar.people import PersonTrack
from locatebot.lidar.types import Pose2D, Scan, wrap_angle
from locatebot.localize.items import ItemTracker
from locatebot.mission.types import ChangeView, ItemView, PersonView, SkillStatus, WorldState

MAX_NOTE_CHARS = 100


def _item_views(tracker: ItemTracker | None, pose: Pose2D, now: float, max_items: int) -> tuple[ItemView, ...]:
    if tracker is None:
        return ()
    views = []
    for name in tracker.items():
        for t in tracker.tracks(name):
            dx, dy = t.x - pose.x, t.y - pose.y
            views.append(ItemView(
                name=name, confirmed=t.confirmed, sightings=t.sightings, x=t.x, y=t.y,
                sigma_m=t.sigma_m, range_m=math.hypot(dx, dy),
                bearing_rad=wrap_angle(math.atan2(dy, dx) - pose.theta),
                last_seen_age_s=max(0.0, now - t.last_seen_s)))
    # Confirmed first, then most sightings, then nearest: the LLM reads the top of the list.
    views.sort(key=lambda v: (not v.confirmed, -v.sightings, v.range_m))
    return tuple(views[:max_items])


def _change_views(changes: Sequence[ChangeBlob] | None, pose: Pose2D, now: float,
                  max_changes: int) -> tuple[ChangeView, ...]:
    views = []
    for b in changes or ():
        if getattr(b, "promoted", False):
            continue  # becoming part of the room
        x, y = b.centroid_xy
        dx, dy = x - pose.x, y - pose.y
        views.append(ChangeView(x=float(x), y=float(y), width_m=float(b.width_m), range_m=math.hypot(dx, dy),
                                bearing_rad=wrap_angle(math.atan2(dy, dx) - pose.theta), moving=bool(b.moving),
                                age_s=max(0.0, now - b.first_seen_s)))
    views.sort(key=lambda v: v.range_m)  # nearest first
    return tuple(views[:max_changes])


def _person_views(people: Sequence[PersonTrack] | None, pose: Pose2D, now: float,
                  max_people: int) -> tuple[PersonView, ...]:
    views = []
    for t in people or ():
        if not t.is_person:
            continue
        dx, dy = t.x - pose.x, t.y - pose.y
        views.append(PersonView(x=float(t.x), y=float(t.y), range_m=math.hypot(dx, dy),
                                bearing_rad=wrap_angle(math.atan2(dy, dx) - pose.theta),
                                speed_mps=max(0.0, float(t.speed_mps)), moving=bool(t.moving),
                                confirmed_by_camera=bool(t.confirmed_by_camera),
                                last_seen_age_s=max(0.0, now - t.last_seen_s)))
    views.sort(key=lambda v: v.range_m)  # nearest first
    return tuple(views[:max_people])


def build_world_state(*, now: float, pose: Pose2D, scan: Scan | None, tracker: ItemTracker | None,
                      skill: SkillStatus, search_items: tuple[str, ...] = (),
                      explored_frac: float | None = None, safety_reason: str = "ok",
                      mission_state: str = "idle", last_event: str = "", n_sectors: int = 8,
                      max_items: int = 12, changes: Sequence[ChangeBlob] | None = None,
                      max_changes: int = 5, people: Sequence[PersonTrack] | None = None,
                      max_people: int = 5, notes: Sequence[str] = (), max_notes: int = 4, goal: str = "") -> WorldState:
    if scan is not None:
        sectors = tuple(float(v) for v in sector_clearance(scan, n_sectors))
    else:
        sectors = tuple(math.nan for _ in range(n_sectors))  # no scan = unknown, never free
    return WorldState(
        stamp_s=now, pose=pose, sector_clearance_m=sectors, explored_frac=explored_frac,
        items=_item_views(tracker, pose, now, max_items), skill=skill,
        search_items=tuple(search_items), safety_reason=safety_reason,
        mission_state=mission_state, last_event=last_event, goal=goal,
        changes=_change_views(changes, pose, now, max_changes),
        people=_person_views(people, pose, now, max_people),
        notes=tuple(" ".join(str(n).split())[:MAX_NOTE_CHARS] for n in list(notes)[:max_notes] if str(n).strip()))
