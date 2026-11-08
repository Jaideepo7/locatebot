"""follow(person) with the LiDAR people tracker (M3), closed loop in simulation.

The "person" is two leg boxes with a gait; the car is the bicycle model; vision sees the person
once at the start only. Synthetic: proves the skill logic, not real-car behaviour.
"""
import copy
import math

import numpy as np

from locatebot.lidar.layers import LayeredMap
from locatebot.lidar.people import PeopleTracker
from locatebot.lidar.sim import Room
from locatebot.lidar.types import Pose2D
from locatebot.localize.items import ItemFix
from locatebot.mission.types import SkillCall, SkillName, SkillState
from locatebot.orin.skills import ExecutorConfig, SkillExecutor, TickInput

from .test_skills import DT, VEH, World, no_collision

POLE = (4.6, 2.62)   # a thin pole beside the person's path: part of the room


def legs(room, c, t, half_gap=0.12, step=0.15, leg=0.12):
    s = step * math.sin(2 * math.pi * 1.0 * t)
    for side, sg in ((half_gap, 1.0), (-half_gap, -1.0)):
        cx, cy = c[0] + s * sg, c[1] + side
        room = room.with_box(cx - leg / 2, cy - leg / 2, cx + leg / 2, cy + leg / 2)
    return room


class PeopleWorld(World):
    def __init__(self, *a, use_people=True, **kw):
        super().__init__(*a, **kw)
        self.use_people = use_people
        self.layers = None
        self.people = PeopleTracker()

    def start_layers(self):
        self.layers = LayeredMap(copy.deepcopy(self.grid))   # the saved room (without the person)

    def _sense(self):
        super()._sense()
        if getattr(self, "layers", None) is not None:
            self.layers.update(self.scan, self.car.pose, self.now)
            self.people.update(self.now, self.layers.last_clusters)

    def inp(self, home=Pose2D(0.0, 0.0, 0.0)) -> TickInput:
        ppl = tuple(self.people.people()) if self.use_people else ()
        return TickInput(now=self.now, pose=self.car.pose, grid=self.grid, scan=self.scan,
                         tracker=self.tracker, home=home, people=ppl)


def run_follow(use_people: bool):
    base = Room.box(8.0, 6.0).with_box(POLE[0] - 0.04, POLE[1] - 0.04, POLE[0] + 0.04, POLE[1] + 0.04)
    w = PeopleWorld(base, Pose2D(1.0, 2.5, 0.0), vision_every_s=1e9, use_people=use_people)
    w.premap(((1.0, 1.5), (4.0, 1.5), (7.0, 1.5), (1.0, 4.5), (4.0, 4.5), (7.0, 4.5), (4.0, 3.0)))
    w.start_layers()
    person = {"x": 2.6, "t0": None}

    def move(w):
        if person["t0"] is None:
            person["t0"] = w.now
        t = w.now - person["t0"]
        person["x"] = min(2.6 + 0.3 * t, 6.8)          # walks right at 0.3 m/s, stops at 6.8
        w.room = legs(base, (person["x"], 2.5), t)

    move(w)
    w.tracker.update("person", ItemFix(2.6, 2.5, 1.6, 0.1, w.now))  # one vision fix, then none
    ex = SkillExecutor(VEH, ExecutorConfig(follow_distance_m=1.0, follow_lost_s=6.0))
    dists = []

    def tick(w):
        move(w)
        dists.append(math.hypot(w.car.pose.x - person["x"], w.car.pose.y - 2.5))

    st = w.run(ex, SkillCall(SkillName.FOLLOW, {"item": "person"}), max_s=18, on_tick=tick)
    return w, ex, st, dists


def test_follow_person_uses_people_tracks_and_keeps_distance():
    w, ex, st, dists = run_follow(use_people=True)
    assert st == SkillState.RUNNING, ex.status.detail
    late = dists[int(10 / DT):]
    assert max(late) < 2.2, f"fell behind: {max(late):.2f} m"
    assert min(dists) > 0.4, "drove into the person"
    tgt = ex._target
    assert tgt is not None and math.hypot(tgt[0] - POLE[0], tgt[1] - POLE[1]) > 0.5
    no_collision(w)
    assert len(w.people.people()) == 1


def test_person_track_beats_a_nearer_raw_cluster():
    from locatebot.lidar.people import PersonTrack
    from locatebot.lidar.sim import simulate_scan
    room = Room.box(8.0, 6.0).with_box(3.0, 2.95, 3.1, 3.05)        # a pole right at the last target
    ex = SkillExecutor(VEH)
    ex.start(SkillCall(SkillName.FOLLOW, {"item": "person"}), 0.0)
    ex._target, ex._target_seen_s = (3.0, 3.0), 0.0
    pose = Pose2D(1.0, 3.0, 0.0)
    track = PersonTrack(7, 3.6, 3.2, 0.5, 0.0, 0.5, True, "legs", 10, True, True, False, 0.0, 0.9, None)
    scan = simulate_scan(room, pose, n_beams=508, rng=np.random.default_rng(1), stamp_s=1.0)
    inp = TickInput(now=1.0, pose=pose, grid=None, scan=scan, tracker=None, people=(track,))
    ex._update_target(inp, "person")
    assert ex._target == (3.6, 3.2) and ex._target_seen_s == 0.9
    ex._target = (3.0, 3.0)
    ex._update_target(TickInput(now=1.0, pose=pose, grid=None, scan=scan, tracker=None), "person")
    assert math.hypot(ex._target[0] - 3.0, ex._target[1] - 3.0) < 0.2   # without tracks: the pole
