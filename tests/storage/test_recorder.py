"""StorageRecorder logic with a fake database (always runs), plus one run against real Postgres."""
import time

import pytest

from locatebot.storage.db import StorageError
from locatebot.storage.recorder import StorageRecorder

from .helpers import jpeg


class FakeDB:
    """Records applied ops; ``down`` makes connect/apply fail like a lost server."""

    def __init__(self, world):
        self.world = world
        self.closed = False

    def migrate(self):
        return 1

    def apply(self, op, image=None):
        if self.world["down"]:
            raise ConnectionError("server gone")
        if op.get("text") == "bad":
            raise StorageError("bad data")
        self.world["applied"].append((op["op"], op.get("text") or op.get("item") or op.get("name"), image))

    def prune(self, max_bytes):
        return 0

    def close(self):
        self.closed = True


def make(tmp_path, **kw):
    world = {"down": False, "applied": [], "connects": 0}

    def connect():
        world["connects"] += 1
        if world["down"]:
            raise ConnectionError("refused")
        return FakeDB(world)

    rec = StorageRecorder(connect, tmp_path / "spool", backoff_max_s=0.05, **kw)
    return rec, world


def wait(pred, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_needs_a_session_first(tmp_path):
    rec, _ = make(tmp_path)
    with pytest.raises(StorageError):
        rec.event(1.0, "too early")


def test_writes_in_order(tmp_path):
    rec, world = make(tmp_path)
    rec.start_session(host="orin")
    rec.start()
    try:
        rec.event(1.0, "a")
        rec.sighting("marker", 1, 2, 0.1, 1.5, 0.2, 10.0, box=(1, 2, 3, 4), image=jpeg())
        rec.event(2.0, "b")
        assert wait(lambda: len(world["applied"]) == 4)
    finally:
        rec.stop()
    assert [a[:2] for a in world["applied"]] == [("session", None), ("event", "a"), ("sighting", "marker"), ("event", "b")]
    assert world["applied"][2][2] == jpeg()


def test_database_down_spools_then_replays_in_order(tmp_path):
    rec, world = make(tmp_path)
    world["down"] = True
    rec.start_session()
    rec.start()
    try:
        for k in range(5):
            rec.event(float(k), f"e{k}", image=jpeg(b"%d" % k) if k == 2 else None)
        assert wait(lambda: rec.status()["spool_files"] == 6)     # session + 5 events on disk
        assert world["applied"] == []
        world["down"] = False
        rec.event(9.0, "after")
        assert wait(lambda: len(world["applied"]) == 7)
    finally:
        rec.stop()
    names = [a[1] for a in world["applied"]]
    assert names == [None, "e0", "e1", "e2", "e3", "e4", "after"]      # spool first, in order
    assert world["applied"][3][2] == jpeg(b"2")                       # image survived the spool
    # "after" may also go through the spool if it arrived during the reconnect backoff: still in order
    assert rec.status()["spool_files"] == 0 and rec.status()["replayed"] in (6, 7)


def test_spool_survives_a_restart(tmp_path):
    rec, world = make(tmp_path)
    world["down"] = True
    rec.start_session()
    rec.start()
    rec.event(1.0, "while down")
    assert wait(lambda: rec.status()["spool_files"] == 2)
    rec.stop()
    rec2, world2 = make(tmp_path)        # new process, same spool dir, database back
    rec2.start_session()
    rec2.start()
    try:
        assert wait(lambda: any(a[1] == "while down" for a in world2["applied"]))
    finally:
        rec2.stop()


def test_bad_data_is_dropped_not_retried(tmp_path):
    rec, world = make(tmp_path)
    rec.start_session()
    rec.start()
    try:
        rec.event(1.0, "bad")
        rec.event(2.0, "good")
        assert wait(lambda: any(a[1] == "good" for a in world["applied"]))
    finally:
        rec.stop()
    assert rec.status()["dropped_ops"] == 1 and rec.status()["spool_files"] == 0


def test_producers_never_block_and_caps_hold(tmp_path):
    rec, world = make(tmp_path, max_queue=3, max_queue_image_bytes=len(jpeg()) + 10)
    rec.start_session()                         # 1 queued (not started: nothing drains)
    t0 = time.monotonic()
    rec.event(1.0, "a", image=jpeg(b"1"))       # fits the image budget
    rec.event(2.0, "b", image=jpeg(b"2"))       # over budget: metadata kept, picture dropped
    rec.event(3.0, "c")                         # queue full: dropped
    assert time.monotonic() - t0 < 0.1
    st = rec.status()
    assert st["dropped_images"] == 1 and st["dropped_ops"] == 1 and st["queue"] == 3


def test_spool_cap_evicts_oldest_but_keeps_the_session(tmp_path):
    rec, world = make(tmp_path, max_spool_bytes=1500)   # ~145 bytes per spooled event here
    world["down"] = True
    rec.start_session()
    rec.start()
    try:
        for k in range(20):
            rec.event(float(k), f"e{k}", image=jpeg(b"%04d" % k))
        assert wait(lambda: rec.status()["spooled"] == 21)
    finally:
        rec.stop()
    st = rec.status()
    assert st["spool_evicted"] > 0
    import json
    ops = [json.loads(p.read_text())["op"] for p in sorted((tmp_path / "spool").glob("*.json"))]
    assert ops[0] == "session"


def test_with_real_postgres(pg_dsn, tmp_path):
    from locatebot.storage.db import RobotDB
    rec = StorageRecorder(pg_dsn, tmp_path / "spool")
    uid = rec.start_session(host="orin", map_name="room1")
    rec.start()
    try:
        rec.sighting("marker", 1, 2, 0.1, 1.5, 0.2, 10.0, image=jpeg(b"real"))
        rec.item("marker", 1, 2, 0.05, 2, True, 10.0)
        rec.event(10.0, "found marker")
        rec.mission("m1", "COMPLETED")
        assert wait(lambda: rec.status()["written"] == 5, timeout=10)
    finally:
        rec.stop()
    db = RobotDB.connect(pg_dsn)
    try:
        items = db.items(uid)
        assert items[0]["name"] == "marker" and db.get_image(items[0]["photo_id"])[1] == jpeg(b"real")
        assert db.events(uid)[0]["text"] == "found marker"
    finally:
        db.close()
