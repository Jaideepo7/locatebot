"""RobotDB against a real PostgreSQL (skipped when none is available, see conftest)."""
import pytest

from locatebot.storage.db import RobotDB, StorageError, image_meta

from .helpers import jpeg

S = "s1"


@pytest.fixture
def db(pg_dsn):
    d = RobotDB.connect(pg_dsn)
    assert d.migrate() == 1
    d.apply({"op": "session", "session": S, "host": "orin", "map_name": "room1", "meta": {"k": 1}})
    yield d
    d.close()


def sighting(item="marker", x=1.0, captured=10.0, **kw):
    op = {"op": "sighting", "session": S, "item": item, "x": x, "y": 2.0, "sigma_m": 0.1, "range_m": 1.5,
          "bearing_rad": 0.2, "captured_s": captured, "box": [1, 2, 3, 4]}
    op.update(kw)
    return op


def test_migrate_is_idempotent(pg_dsn):
    a, b = RobotDB.connect(pg_dsn), RobotDB.connect(pg_dsn)
    try:
        assert a.migrate() == 1 and b.migrate() == 1 and a.migrate() == 1
    finally:
        a.close()
        b.close()


def test_image_meta_validates():
    mime, w, h, digest = image_meta(jpeg())
    assert (mime, w, h) == ("image/jpeg", 640, 360) and len(digest) == 32
    with pytest.raises(StorageError):
        image_meta(b"GIF89a....")
    with pytest.raises(StorageError):
        image_meta(b"")


def test_sighting_with_photo_and_dedupe(db):
    pic = jpeg(b"frame-1")
    db.apply(sighting(captured=10.0), pic)
    db.apply(sighting(captured=10.0, x=1.1), pic)          # second box in the same frame
    with db.conn.transaction():
        n_img = db.conn.execute("SELECT count(*) FROM image").fetchone()[0]
        ids = db.conn.execute("SELECT DISTINCT image_id FROM sighting").fetchall()
    assert n_img == 1 and len(ids) == 1                    # same picture stored once
    mime, data = db.get_image(ids[0][0])
    assert mime == "image/jpeg" and data == pic


def test_item_points_at_newest_photo_and_keeps_it(db):
    db.apply(sighting(captured=10.0), jpeg(b"a"))
    db.apply(sighting(captured=12.0), jpeg(b"b"))
    db.apply(sighting(captured=13.0))                       # no photo: doesn't win
    db.apply({"op": "item", "session": S, "name": "marker", "x": 1.0, "y": 2.0, "sigma_m": 0.05,
              "sightings": 3, "confirmed": True, "last_seen_s": 13.0})
    items = db.items(S)
    assert len(items) == 1 and items[0]["confirmed"] and items[0]["sightings"] == 3
    assert db.get_image(items[0]["photo_id"])[1] == jpeg(b"b")
    with db.conn.transaction():
        keep = db.conn.execute("SELECT keep FROM image WHERE id = %s", (items[0]["photo_id"],)).fetchone()[0]
    assert keep
    # update keeps the photo when the new upsert has no newer sighting photo
    db.apply({"op": "item", "session": S, "name": "marker", "x": 1.05, "y": 2.0, "sigma_m": 0.04,
              "sightings": 4, "confirmed": True, "last_seen_s": 14.0})
    assert db.items(S)[0]["photo_id"] == items[0]["photo_id"] and db.items(S)[0]["sightings"] == 4


def test_events_missions_and_maps(db):
    db.apply({"op": "event", "session": S, "t_s": 1.0, "text": "found marker"})
    db.apply({"op": "event", "session": S, "t_s": 2.0, "text": "search succeeded"})
    assert [e["text"] for e in db.events(S)] == ["search succeeded", "found marker"]
    for state in ("ACCEPTED", "SEARCHING", "COMPLETED"):
        db.apply({"op": "mission", "session": S, "mission_id": "m1", "state": state, "message": "", "reason": ""})
    with db.conn.transaction():
        assert db.conn.execute("SELECT state FROM mission WHERE mission_id='m1'").fetchone()[0] == "COMPLETED"
        assert db.conn.execute("SELECT count(*) FROM mission_status").fetchone()[0] == 3
    png = b"\x89PNG\r\n\x1a\n" + b"\x00\x00\x00\rIHDR" + (40).to_bytes(4, "big") + (30).to_bytes(4, "big") + b"\x08\x00\x00\x00\x00"
    map_id = db.apply({"op": "map", "session": S, "name": "room1", "lbmap_hex": b"LBMAP1\0\0rest".hex()}, png)
    with db.conn.transaction():
        lbmap, image_id = db.conn.execute("SELECT lbmap, image_id FROM map_snapshot WHERE id=%s", (map_id,)).fetchone()
    assert bytes(lbmap) == b"LBMAP1\0\0rest" and db.get_image(image_id)[0] == "image/png"


def test_bad_image_rolls_back_the_whole_operation(db):
    with pytest.raises(StorageError):
        db.apply(sighting(), b"not an image")
    with db.conn.transaction():
        assert db.conn.execute("SELECT count(*) FROM sighting").fetchone()[0] == 0


def test_prune_spares_kept_and_referenced_images(db):
    for k in range(6):
        db.apply(sighting(captured=float(k)), jpeg(b"p%d" % k))
    db.apply({"op": "item", "session": S, "name": "marker", "x": 1, "y": 2, "sigma_m": 0.1, "sightings": 6,
              "confirmed": True, "last_seen_s": 5.0})
    best = db.items(S)[0]["photo_id"]
    db.apply({"op": "snapshot", "session": S, "kind": "snapshot", "captured_s": 9.0, "keep": True}, jpeg(b"keep-me"))
    deleted = db.prune(max_bytes=0)
    assert deleted == 5                                   # 6 sightings - the item's photo
    with db.conn.transaction():
        left = {r[0] for r in db.conn.execute("SELECT id FROM image").fetchall()}
        assert db.conn.execute("SELECT count(*) FROM sighting").fetchone()[0] == 6   # rows survive, photo unlinked
    assert best in left and len(left) == 2
    assert db.prune(max_bytes=10 ** 9) == 0


def test_unknown_operation_rejected(db):
    with pytest.raises(StorageError):
        db.apply({"op": "drop_tables", "session": S})
