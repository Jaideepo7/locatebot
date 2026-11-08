"""Runtime + real recorder + real PostgreSQL + the operator page serving the stored photo."""
import json
import threading
import time
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
from locatebot.storage.reader import DBReader
from locatebot.storage.recorder import StorageRecorder
from locatebot.vision.service import Found, VisionResult

from .helpers import jpeg


def test_found_item_photo_reaches_the_operator_page(pg_dsn, tmp_path):
    rec = StorageRecorder(pg_dsn, tmp_path / "spool")
    rec.start_session(host="orin-test", map_name="demo")
    rec.start()
    reader = DBReader(pg_dsn)
    rt = OrinRuntime(LiveMapper(size_m=12.0), SkillExecutor(VehicleConfig()), recorder=rec)
    rng = np.random.default_rng(0)
    for i in range(5):
        rt.on_scan(simulate_scan(demo_room(), Pose2D(4.0, 3.0, 0.0), rng=rng, stamp_s=1.0 + 0.1 * i, seq=i))
    for t, tag in ((1.2, b"one"), (1.4, b"two")):
        rt.on_frame(jpeg(tag), t)
        rt.on_vision_result(VisionResult("marker", t, 640, 360, [Found("marker", (300, 100, 340, 200), 0.0)]))
    app = OrinApp(rt, lambda t, s: None, config=AppConfig(maps_dir=str(tmp_path)), photo_db=reader)
    srv = make_server(app, "127.0.0.1", 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        end, items = time.monotonic() + 10, []
        while time.monotonic() < end:
            items = json.loads(urllib.request.urlopen(base + "/api/items", timeout=5).read())["items"]
            if items and items[0]["photo"]:
                break
            time.sleep(0.05)
        assert items and items[0]["name"] == "marker" and items[0]["confirmed"]
        photo = urllib.request.urlopen(base + items[0]["photo"], timeout=5).read()
        assert photo == jpeg(b"two")                       # the newest sighting's frame
    finally:
        srv.shutdown()
        srv.server_close()
        rec.stop()
        reader.close()
    st = rec.status()
    assert st["errors"] == 0 and st["spool_files"] == 0 and st["written"] == 5  # session, 2 sightings, item, event
