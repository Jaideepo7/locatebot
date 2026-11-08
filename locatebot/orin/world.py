"""Build the ``WorldState`` snapshot the LLM agent sees.

Combines the robot pose, the newest LiDAR scan (as per-sector clearances), the item tracks
from vision + LiDAR localization, and the current skill. Pure function: no locks, no I/O.
"""
from __future__ import annotations

import math

from locatebot.lidar.detect import sector_clearance
from locatebot.lidar.types import Pose2D, Scan, wrap_angle
from locatebot.localize.items import ItemTracker
from locatebot.mission.types import ItemView, SkillStatus, WorldState


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


def build_world_state(*, now: float, pose: Pose2D, scan: Scan | None, tracker: ItemTracker | None,
                      skill: SkillStatus, search_items: tuple[str, ...] = (),
                      explored_frac: float | None = None, safety_reason: str = "ok",
                      mission_state: str = "idle", last_event: str = "", n_sectors: int = 8,
                      max_items: int = 12, goal: str = "") -> WorldState:
    if scan is not None:
        sectors = tuple(float(v) for v in sector_clearance(scan, n_sectors))
    else:
        sectors = tuple(math.nan for _ in range(n_sectors))  # no scan = unknown, never free
    return WorldState(
        stamp_s=now, pose=pose, sector_clearance_m=sectors, explored_frac=explored_frac,
        items=_item_views(tracker, pose, now, max_items), skill=skill,
        search_items=tuple(search_items), safety_reason=safety_reason,
        mission_state=mission_state, last_event=last_event, goal=goal)
