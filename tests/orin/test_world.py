"""build_world_state: what the LLM agent sees."""
import math

import numpy as np

from locatebot.lidar.types import Pose2D, Scan
from locatebot.localize.items import ItemFix, ItemTracker
from locatebot.mission.types import SkillCall, SkillName, SkillState, SkillStatus
from locatebot.orin.world import build_world_state


def _ring_scan(r=2.0, n=360, nan_back=False):
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    ranges = np.full(n, r)
    if nan_back:
        ranges[np.abs(a) > math.radians(150)] = np.nan  # whole back sector (±22.5° around 180°)
    return Scan(stamp_s=1.0, seq=0, angles_rad=a, ranges_m=ranges, quality=np.full(n, 47, np.uint8))


def _fix(x, y, t):
    return ItemFix(x=x, y=y, range_m=1.0, sigma_m=0.1, stamp_s=t)


def test_items_relative_to_pose_and_sorted():
    tr = ItemTracker(min_sightings=2)
    tr.update("marker", _fix(1.0, 1.0, 5.0))
    tr.update("marker", _fix(1.0, 1.0, 6.0))
    tr.update("bolt", _fix(-2.0, 0.0, 2.0))
    ws = build_world_state(now=10.0, pose=Pose2D(0.0, 0.0, 0.0), scan=_ring_scan(), tracker=tr,
                           skill=SkillStatus(), search_items=("marker", "bolt"), explored_frac=0.5)
    names = [i.name for i in ws.items]
    assert names[0] == "marker"  # confirmed first
    m = ws.items[0]
    assert m.confirmed and m.sightings == 2
    assert math.isclose(m.range_m, math.sqrt(2), rel_tol=1e-6)
    assert math.isclose(m.bearing_rad, math.pi / 4, rel_tol=1e-6)  # left
    assert math.isclose(m.last_seen_age_s, 4.0)
    b = ws.items[1]
    assert abs(abs(b.bearing_rad) - math.pi) < 1e-6  # behind
    assert ws.search_items == ("marker", "bolt") and ws.explored_frac == 0.5


def test_bearing_uses_heading():
    tr = ItemTracker()
    tr.update("cup", _fix(0.0, 1.0, 1.0))
    ws = build_world_state(now=1.0, pose=Pose2D(0.0, 0.0, math.pi / 2), scan=None, tracker=tr,
                           skill=SkillStatus())
    assert abs(ws.items[0].bearing_rad) < 1e-9  # straight ahead after turning left 90°


def test_sectors_from_scan_keep_unknown_as_nan():
    ws = build_world_state(now=1.0, pose=Pose2D(0, 0, 0), scan=_ring_scan(nan_back=True), tracker=None,
                           skill=SkillStatus())
    assert len(ws.sector_clearance_m) == 8
    assert math.isclose(ws.sector_clearance_m[0], 2.0)
    assert math.isnan(ws.sector_clearance_m[4])  # back sector has no returns


def test_no_scan_means_all_unknown_and_items_capped():
    tr = ItemTracker()
    for k in range(30):
        tr.update(f"item{k}", _fix(k, 0.0, 1.0))
    call = SkillCall(SkillName.EXPLORE)
    ws = build_world_state(now=2.0, pose=Pose2D(0, 0, 0), scan=None, tracker=tr,
                           skill=SkillStatus(SkillState.RUNNING, call), max_items=12,
                           safety_reason="obstacle", last_event="blocked", mission_state="exploring")
    assert all(math.isnan(v) for v in ws.sector_clearance_m)
    assert len(ws.items) == 12
    assert ws.skill.call == call and ws.safety_reason == "obstacle" and ws.last_event == "blocked"


def test_changes_become_change_views_nearest_first_without_promoted():
    from locatebot.lidar.layers import ChangeBlob
    blobs = [ChangeBlob(1, (3.0, 0.0), 0.4, 8, 2.0, 9.0, 20, moving=False),
             ChangeBlob(2, (0.0, 1.0), 0.3, 5, 8.0, 9.5, 10, moving=True),
             ChangeBlob(3, (1.0, 1.0), 0.5, 9, 0.0, 9.5, 90, promoted=True)]
    ws = build_world_state(now=10.0, pose=Pose2D(0.0, 0.0, 0.0), scan=None, tracker=None,
                           skill=SkillStatus(), changes=blobs, max_changes=5)
    assert [round(c.range_m, 3) for c in ws.changes] == [1.0, 3.0]
    near = ws.changes[0]
    assert near.moving and math.isclose(near.bearing_rad, math.pi / 2) and math.isclose(near.age_s, 2.0)
    assert build_world_state(now=1.0, pose=Pose2D(0, 0, 0), scan=None, tracker=None,
                             skill=SkillStatus()).changes == ()


def test_people_views_only_people_nearest_first_capped_and_notes_bounded():
    from locatebot.lidar.people import PersonTrack

    def tr(i, x, y, likely=True, cam=False, confirmed=True):
        return PersonTrack(i, x, y, 0.5, 0.0, 0.5, True, "legs", 5, confirmed, likely, cam, 0.0, 9.0, None)

    people = [tr(1, 3.0, 0.0), tr(2, 0.0, -1.0, cam=True), tr(3, 1.0, 1.0, likely=False),   # 3: an object
              *[tr(10 + k, 5.0 + k, 0.0) for k in range(6)]]
    ws = build_world_state(now=10.0, pose=Pose2D(0.0, 0.0, 0.0), scan=None, tracker=None, skill=SkillStatus(),
                           people=people, notes=["a  b\nc", "x" * 300, "", "n4", "n5", "n6"])
    assert len(ws.people) == 5 and [round(p.range_m, 1) for p in ws.people[:2]] == [1.0, 3.0]
    assert ws.people[0].confirmed_by_camera and math.isclose(ws.people[0].bearing_rad, -math.pi / 2)
    assert math.isclose(ws.people[0].last_seen_age_s, 1.0)
    assert ws.notes[0] == "a b c" and len(ws.notes[1]) == 100 and len(ws.notes) == 3
