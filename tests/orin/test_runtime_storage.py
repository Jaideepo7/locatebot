"""Orin runtime/app <-> storage wiring with a fake recorder (no database needed)."""
import json
import struct
import threading
import urllib.error
import urllib.request

import numpy as np

from locatebot.lidar.mapping import demo_room
from locatebot.lidar.sim import simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import VehicleConfig
from locatebot.orin.app import AppConfig, OrinApp, make_server
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import OrinRuntime
from locatebot.orin.skills import SkillExecutor
from locatebot.vision.service import Found, VisionResult


def jpeg(tag: bytes) -> bytes:
    com = b"t:" + tag
    return (b"\xff\xd8\xff\xfe" + struct.pack(">H", len(com) + 2) + com
            + b"\xff\xc0" + struct.pack(">HBHHB", 8, 8, 360, 640, 3) + b"\xff\xd9")


class FakeRecorder:
    def __init__(self, fail=False):
        self.calls, self.fail, self.session_uid = [], fail, "sess-1"

    def __getattr__(self, name):
        def record(*a, **k):
            if self.fail:
                raise RuntimeError("database exploded")
            self.calls.append((name, a, k))
        return record


def runtime(recorder):
    rt = OrinRuntime(LiveMapper(size_m=12.0), SkillExecutor(VehicleConfig()), recorder=recorder)
    rng = np.random.default_rng(0)
    for i in range(5):
        rt.on_scan(simulate_scan(demo_room(), Pose2D(4.0, 3.0, 0.0), rng=rng, stamp_s=1.0 + 0.1 * i, seq=i))
    return rt


def sight(rt, t, tag):
    rt.on_frame(jpeg(tag), t)
    rt.on_vision_result(VisionResult("marker", t, 640, 360, [Found("marker", (300, 100, 340, 200), 0.0)]))


def test_sightings_carry_the_exact_frame_and_items_are_recorded_once_confirmed():
    rec = FakeRecorder()
    rt = runtime(rec)
    sight(rt, 1.2, b"first")
    sightings = [c for c in rec.calls if c[0] == "sighting"]
    assert len(sightings) == 1 and sightings[0][1][0] == "marker"
    assert sightings[0][2]["image"] == jpeg(b"first") and sightings[0][2]["box"] == (300, 100, 340, 200)
    assert not [c for c in rec.calls if c[0] == "item"]          # one sighting: not confirmed yet
    sight(rt, 1.4, b"second")
    items = [c for c in rec.calls if c[0] == "item"]
    assert items and items[-1][1][0] == "marker" and items[-1][1][5] is True
    assert any(c[0] == "event" and "found marker" in c[1][1] for c in rec.calls)


def test_no_frame_stored_without_storage():
    rt = runtime(None)
    rt.on_frame(jpeg(b"x"), 1.2)
    assert len(rt._frames) == 0


def test_storage_failures_never_reach_the_robot():
    rt = runtime(FakeRecorder(fail=True))
    sight(rt, 1.2, b"a")
    sight(rt, 1.4, b"b")
    assert rt.tracker.best("marker").confirmed
    rt.step(2.0)


class FakeReader:
    def __init__(self):
        self.pic = jpeg(b"served")

    def items(self, session):
        assert session == "sess-1"
        return [{"name": "marker", "x": 1.234, "y": 2.0, "sightings": 3, "confirmed": True, "photo_id": 7},
                {"name": "bolt", "x": 0.0, "y": 0.0, "sightings": 1, "confirmed": False, "photo_id": None}]

    def get_image(self, image_id):
        return ("image/jpeg", self.pic) if image_id == 7 else None


def test_app_serves_items_photos_and_stores_saved_maps(tmp_path):
    rec = FakeRecorder()
    rt = runtime(rec)
    app = OrinApp(rt, lambda t, s: None, config=AppConfig(maps_dir=str(tmp_path)), photo_db=FakeReader())
    srv = make_server(app, "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        items = json.loads(urllib.request.urlopen(base + "/api/items", timeout=5).read())
        assert items["storage"] and items["items"][0]["photo"] == "/photos/7" and items["items"][1]["photo"] is None
        r = urllib.request.urlopen(base + "/photos/7", timeout=5)
        assert r.headers["Content-Type"] == "image/jpeg" and r.read() == jpeg(b"served")
        for bad in ("/photos/8", "/photos/abc", "/photos/../etc/passwd"):
            try:
                urllib.request.urlopen(base + bad, timeout=5)
                raise AssertionError(bad)
            except urllib.error.HTTPError as e:
                assert e.code == 404
        page = urllib.request.urlopen(base + "/", timeout=5).read()
        assert b"Found item photos" in page and b"http://" not in page and b"https://" not in page
    finally:
        srv.shutdown()
        srv.server_close()
    app.save_map("room1")
    maps = [c for c in rec.calls if c[0] == "map_snapshot"]
    assert maps and maps[0][1][0] == "room1" and maps[0][1][1][:6] == b"LBMAP1" and maps[0][1][2][:4] == b"\x89PNG"


def test_app_without_storage_says_so():
    app = OrinApp(runtime(None), lambda t, s: None)
    assert app.found_items() == {"storage": False, "items": []}
