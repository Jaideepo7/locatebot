"""SkillExecutor closed-loop tests: bicycle model + simulated LiDAR + live grid (synthetic only).

These prove the skill logic and its use of Nav-lite in simulation, not real-car behaviour.
"""
import math

import numpy as np
import pytest

from locatebot.lidar.grid import OccupancyGrid
from locatebot.lidar.sim import Room, raycast, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.localize.items import ItemFix, ItemTracker
from locatebot.mission.types import SkillCall, SkillName, SkillState, VehicleConfig
from locatebot.nav.sim import BicycleSim
from locatebot.orin.skills import ExecutorConfig, SkillExecutor, TickInput

VEH = VehicleConfig()
DT = 0.1  # executor tick


def seg_dist(room: Room, x: float, y: float) -> float:
    s = room.segments
    p = np.array([x, y])
    a, b = s[:, :2], s[:, 2:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0, 1)
    return float(np.min(np.linalg.norm(a + t[:, None] * ab - p, axis=1)))


def box_room() -> Room:
    """6 x 5 m room with a table-like box."""
    return Room.box(6.0, 5.0).with_box(2.6, 2.0, 3.4, 2.8)


class World:
    """Simulated car + LiDAR + live occupancy grid + optional fake vision."""

    def __init__(self, room: Room, pose: Pose2D, items: dict[str, tuple[float, float]] | None = None,
                 vision_every_s: float = 2.0, hfov_deg: float = 70.0, seed: int = 0):
        self.room = room
        self.car = BicycleSim(VEH, pose, dt=0.05)
        self.grid = OccupancyGrid.around((3.0, 2.5), 10.0, 0.05)
        self.tracker = ItemTracker(min_sightings=2)
        self.items = dict(items or {})
        self.vision_every_s = vision_every_s
        self.hfov = math.radians(hfov_deg)
        self.rng = np.random.default_rng(seed)
        self.now = 100.0
        self.next_vision = self.now
        self.min_wall_dist = math.inf
        self.scan = None
        self._sense()

    def _sense(self):
        self.scan = simulate_scan(self.room, self.car.pose, n_beams=360, noise_std_m=0.01, rng=self.rng,
                                  stamp_s=self.now)
        self.grid.integrate(self.scan, self.car.pose)

    def _vision(self, searching: tuple[str, ...]):
        if self.now < self.next_vision:
            return
        self.next_vision = self.now + self.vision_every_s
        p = self.car.pose
        for name in searching:
            if name not in self.items:
                continue
            ix, iy = self.items[name]
            d = math.hypot(ix - p.x, iy - p.y)
            bearing = math.atan2(iy - p.y, ix - p.x) - p.theta
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(bearing) > self.hfov / 2 or d > 4.0:
                continue
            wall = raycast(self.room, (p.x, p.y), np.array([p.theta + bearing]))[0]
            if wall < d - 0.15:
                continue  # hidden behind something
            self.tracker.update(name, ItemFix(ix, iy, d, 0.1, self.now))

    def premap(self, poses=((1.0, 1.0), (5.0, 1.0), (5.0, 4.0), (1.0, 4.0), (3.0, 4.4), (3.0, 0.6))):
        """Integrate scans from a few poses, as if the room had been explored already."""
        for x, y in poses:
            for th in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                scan = simulate_scan(self.room, Pose2D(x, y, th), n_beams=360, rng=self.rng, stamp_s=self.now)
                self.grid.integrate(scan, Pose2D(x, y, th))
        return self

    def inp(self, home=Pose2D(0.0, 0.0, 0.0)) -> TickInput:
        return TickInput(now=self.now, pose=self.car.pose, grid=self.grid, scan=self.scan,
                         tracker=self.tracker, home=home)

    def run(self, ex: SkillExecutor, call: SkillCall, max_s: float, home=Pose2D(0.0, 0.0, 0.0),
            on_tick=None) -> SkillState:
        ex.start(call, self.now)
        for _ in range(int(max_s / DT)):
            if on_tick:
                on_tick(self)
            out = ex.tick(self.inp(home))
            assert -1.0 <= out.steering <= 1.0 and 0.0 <= out.throttle <= VEH.max_throttle + 1e-9
            for _ in range(2):
                self.car.step(out.throttle, out.steering)
            self.now += DT
            self.min_wall_dist = min(self.min_wall_dist, seg_dist(self.room, self.car.pose.x, self.car.pose.y))
            self._sense()
            self._vision(ex.search_items)
            if ex.status.state not in (SkillState.RUNNING,):
                return ex.status.state
        return ex.status.state


def no_collision(w: World):
    assert w.min_wall_dist > VEH.half_width_m * 0.8, f"came within {w.min_wall_dist:.2f} m of a wall"


def test_stop_is_immediate_and_outputs_zero():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH)
    ex.start(SkillCall(SkillName.STOP), w.now)
    out = ex.tick(w.inp())
    assert (out.throttle, out.steering) == (0.0, 0.0)
    assert ex.status.state == SkillState.SUCCEEDED


def test_go_to_point_reaches_it():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    w.premap()
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.GO_TO, {"x": 4.8, "y": 3.8}), max_s=60)
    assert st == SkillState.SUCCEEDED, ex.status.detail
    assert math.hypot(w.car.pose.x - 4.8, w.car.pose.y - 3.8) < 0.4
    no_collision(w)


def test_go_to_item_stops_at_standoff():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    w.premap()
    w.tracker.update("marker", ItemFix(5.9, 4.0, 3.0, 0.1, w.now))  # against the right wall
    ex = SkillExecutor(VEH, ExecutorConfig(standoff_m=0.7))
    st = w.run(ex, SkillCall(SkillName.GO_TO, {"item": "marker"}), max_s=60)
    assert st == SkillState.SUCCEEDED, ex.status.detail
    d = math.hypot(w.car.pose.x - 5.9, w.car.pose.y - 4.0)
    assert d < 0.7 + 0.4
    no_collision(w)


def test_go_to_unseen_item_fails_at_once():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH)
    ex.start(SkillCall(SkillName.GO_TO, {"item": "bolt"}), w.now)
    out = ex.tick(w.inp())
    assert out.throttle == 0.0
    assert ex.status.state == SkillState.FAILED and "bolt" in ex.status.detail


def test_go_to_unreachable_point_fails():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0)).premap()
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.GO_TO, {"x": 3.0, "y": 2.4}), max_s=10)  # inside the box
    assert st == SkillState.FAILED


def test_return_home():
    w = World(box_room(), Pose2D(4.8, 3.8, math.pi))
    w.premap()
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.RETURN_HOME), max_s=60, home=Pose2D(1.0, 1.0, 0.0))
    assert st == SkillState.SUCCEEDED, ex.status.detail
    assert math.hypot(w.car.pose.x - 1.0, w.car.pose.y - 1.0) < 0.5
    no_collision(w)


def test_explore_maps_the_room_and_finishes():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.3))
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.EXPLORE), max_s=240)
    assert st == SkillState.SUCCEEDED, ex.status.detail
    assert ex.explored_frac is not None and ex.explored_frac > 0.8
    no_collision(w)


def test_search_finds_items_hidden_behind_the_box():
    room = box_room()
    w = World(room, Pose2D(1.0, 1.0, 0.0), items={"marker": (3.7, 2.4), "bolt": (5.5, 4.6)})
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.SEARCH, {"items": ["marker", "bolt"]}), max_s=240)
    assert ex.search_items == () or st != SkillState.RUNNING
    assert st == SkillState.SUCCEEDED, ex.status.detail
    assert w.tracker.best("marker").confirmed and w.tracker.best("bolt").confirmed
    no_collision(w)


def test_search_reports_missing_items():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0), items={"marker": (3.7, 2.4)})
    ex = SkillExecutor(VEH, ExecutorConfig(max_patrol_points=4))
    st = w.run(ex, SkillCall(SkillName.SEARCH, {"items": ["marker", "unicorn"]}), max_s=300)
    assert st == SkillState.FAILED
    assert "unicorn" in ex.status.detail and "marker" in ex.status.detail


def test_search_items_exposed_for_vision_while_running():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH)
    ex.start(SkillCall(SkillName.SEARCH, {"items": ["marker", "bolt"]}), w.now)
    assert ex.search_items == ("marker", "bolt")
    ex.start(SkillCall(SkillName.STOP), w.now)
    assert ex.search_items == ()


def test_look_around_turns_a_full_circle_in_open_space():
    w = World(Room.box(8.0, 8.0), Pose2D(4.0, 3.0, 0.0))
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.LOOK_AROUND), max_s=40)
    assert st == SkillState.SUCCEEDED, ex.status.detail
    no_collision(w)


def test_look_around_refuses_without_room():
    w = World(Room.box(1.2, 6.0), Pose2D(0.6, 1.0, math.pi / 2))  # narrow corridor
    ex = SkillExecutor(VEH)
    st = w.run(ex, SkillCall(SkillName.LOOK_AROUND), max_s=5)
    assert st == SkillState.FAILED and "room" in ex.status.detail


def test_follow_keeps_distance_to_a_moving_item_then_reports_lost():
    base = Room.box(8.0, 6.0)
    target = {"x": 3.0, "y": 3.0, "t0": None}

    def move(w: World):
        if target["t0"] is None:
            target["t0"] = w.now
        t = w.now - target["t0"]
        if t < 25.0:
            target["x"] = 3.0 + 0.12 * t  # walks slowly to the right
            x = target["x"]
            w.room = base.with_box(x - 0.1, 2.9, x + 0.1, 3.1)
            w.items = {"person": (x, 3.0)}
        else:
            w.items = {}  # gone
            w.room = base

    w = World(base, Pose2D(1.0, 3.0, 0.0), items={"person": (3.0, 3.0)}, vision_every_s=2.0)
    w.tracker.update("person", ItemFix(3.0, 3.0, 2.0, 0.1, w.now))
    ex = SkillExecutor(VEH, ExecutorConfig(follow_distance_m=1.0, follow_lost_s=6.0))
    dists = []

    def tick(w):
        move(w)
        dists.append(math.hypot(w.car.pose.x - target["x"], w.car.pose.y - 3.0))

    st = w.run(ex, SkillCall(SkillName.FOLLOW, {"item": "person"}), max_s=50, on_tick=tick)
    assert st == SkillState.FAILED and "lost" in ex.status.detail
    late = dists[int(15 / DT):int(24 / DT)]
    assert max(late) < 2.2, f"fell behind: {max(late):.2f} m"
    assert min(dists) > 0.4, "drove into the target"
    no_collision(w)


def test_new_call_replaces_running_one():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH)
    ex.start(SkillCall(SkillName.GO_TO, {"x": 4.8, "y": 3.8}), w.now)
    ex.tick(w.inp())
    ex.start(SkillCall(SkillName.STOP), w.now)
    out = ex.tick(w.inp())
    assert out.throttle == 0.0 and ex.status.call.name == SkillName.STOP
    assert ex.status.state == SkillState.SUCCEEDED


def test_no_map_waits_then_fails():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH, ExecutorConfig(no_map_timeout_s=2.0))
    ex.start(SkillCall(SkillName.GO_TO, {"x": 4.0, "y": 3.0}), w.now)
    for k in range(30):
        inp = TickInput(now=w.now + k * DT, pose=w.car.pose, grid=None, scan=None, tracker=w.tracker)
        out = ex.tick(inp)
        assert out.throttle == 0.0
    assert ex.status.state == SkillState.FAILED and "map" in ex.status.detail


def test_tick_never_raises_and_stops_on_internal_error(monkeypatch):
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH)
    ex.start(SkillCall(SkillName.GO_TO, {"x": 4.0, "y": 3.0}), w.now)
    import locatebot.orin.skills as skills_mod

    def boom(*a, **k):
        raise RuntimeError("planner exploded")

    monkeypatch.setattr(skills_mod, "build_costmap", boom)
    out = ex.tick(w.inp())
    assert (out.throttle, out.steering) == (0.0, 0.0)
    assert ex.status.state == SkillState.FAILED and "planner exploded" in ex.status.detail


@pytest.mark.parametrize("call", [
    SkillCall(SkillName.GO_TO, {}),
    SkillCall(SkillName.SEARCH, {"items": []}),
])
def test_invalid_calls_fail_cleanly(call):
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH)
    ex.start(call, w.now)
    out = ex.tick(w.inp())
    assert out.throttle == 0.0 and ex.status.state == SkillState.FAILED


def test_go_to_unmapped_point_says_explore_first():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0))
    ex = SkillExecutor(VEH, ExecutorConfig(max_replans=2))
    st = w.run(ex, SkillCall(SkillName.GO_TO, {"x": 8.5, "y": 8.5}), max_s=10)  # outside, never seen
    assert st == SkillState.FAILED and "unmapped" in ex.status.detail


def test_search_times_out_with_a_report():
    w = World(box_room(), Pose2D(1.0, 1.0, 0.0), items={"marker": (3.7, 2.4)})
    ex = SkillExecutor(VEH, ExecutorConfig(search_timeout_s=20.0))
    st = w.run(ex, SkillCall(SkillName.SEARCH, {"items": ["unicorn"]}), max_s=40)
    assert st == SkillState.FAILED and "timed out" in ex.status.detail and "unicorn" in ex.status.detail
