"""``RobotDB``: synchronous PostgreSQL access for the robot's shared storage.

One ``RobotDB`` owns one connection and is used from one thread at a time (the recorder's
writer thread, or a reader guarded by a lock). Writes arrive as *operations*: plain dicts with
an ``op`` name, built by ``StorageRecorder`` so they can be queued, spooled to disk while the
database is down and replayed later. ``apply`` runs each operation in its own transaction.

Needs the ``db`` extra (``psycopg[binary]``); the module imports psycopg lazily so the rest of
the package works without it.
"""
from __future__ import annotations

import hashlib
import json
from typing import Any

from locatebot.vision.postprocess import jpeg_size

from .schema import MIGRATIONS

MAX_IMAGE_BYTES = 4 * 1024 * 1024
IMAGE_KINDS = ("sighting", "event", "snapshot", "verify", "map")
_LOCK_KEY = 0x10CA7EB0  # advisory lock id for migrations


class StorageError(Exception):
    pass


def image_meta(data: bytes) -> tuple[str, int, int, bytes]:
    """(mime, width, height, sha256) for a JPEG or PNG; raises StorageError otherwise."""
    if not isinstance(data, (bytes, bytearray)) or not data:
        raise StorageError("image must be non-empty bytes")
    if len(data) > MAX_IMAGE_BYTES:
        raise StorageError(f"image of {len(data)} bytes exceeds {MAX_IMAGE_BYTES}")
    digest = hashlib.sha256(data).digest()
    if data[:2] == b"\xff\xd8":
        try:
            w, h = jpeg_size(bytes(data))
        except ValueError as e:
            raise StorageError(f"bad JPEG: {e}") from e
        return "image/jpeg", w, h, digest
    if data[:8] == b"\x89PNG\r\n\x1a\n" and len(data) >= 24:
        w = int.from_bytes(data[16:20], "big")
        h = int.from_bytes(data[20:24], "big")
        if w and h:
            return "image/png", w, h, digest
    raise StorageError("image must be a JPEG or PNG")


class RobotDB:
    def __init__(self, conn) -> None:
        self.conn = conn

    @classmethod
    def connect(cls, dsn: str, connect_timeout_s: float = 3.0) -> "RobotDB":
        import psycopg  # lazy: optional extra

        conn = psycopg.connect(dsn, connect_timeout=max(1, int(round(connect_timeout_s))),
                               application_name="locatebot")
        return cls(conn)

    def close(self) -> None:
        try:
            self.conn.close()
        except Exception:
            pass

    @property
    def closed(self) -> bool:
        return bool(getattr(self.conn, "closed", True))

    def ping(self) -> bool:
        try:
            with self.conn.transaction():
                self.conn.execute("SELECT 1")
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------ schema
    def migrate(self) -> int:
        """Apply pending migrations under an advisory lock; returns the schema version."""
        with self.conn.transaction():
            self.conn.execute("SELECT pg_advisory_xact_lock(%s)", (_LOCK_KEY,))
            self.conn.execute("CREATE TABLE IF NOT EXISTS schema_version ("
                              "version integer PRIMARY KEY, applied_at timestamptz NOT NULL DEFAULT now())")
            row = self.conn.execute("SELECT coalesce(max(version), 0) FROM schema_version").fetchone()
            current = int(row[0])
            for version, sql in MIGRATIONS:
                if version > current:
                    self.conn.execute(sql)
                    self.conn.execute("INSERT INTO schema_version (version) VALUES (%s)", (version,))
                    current = version
            return current

    # ------------------------------------------------------------------ writes
    def apply(self, op: dict, image: bytes | None = None) -> Any:
        """Run one recorder operation in its own transaction."""
        name = op.get("op")
        handler = getattr(self, f"_op_{name}", None)
        if handler is None:
            raise StorageError(f"unknown storage operation {name!r}")
        with self.conn.transaction():
            return handler(op, image)

    def _insert_image(self, data: bytes, kind: str, session_uid: str | None, captured_s: float | None,
                      keep: bool = False) -> int:
        if kind not in IMAGE_KINDS:
            raise StorageError(f"image kind must be one of {IMAGE_KINDS}")
        mime, w, h, digest = image_meta(data)
        row = self.conn.execute(
            "INSERT INTO image (sha256, mime, width, height, nbytes, data, kind, session_uid, captured_s, keep) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) "
            "ON CONFLICT (sha256) DO UPDATE SET keep = image.keep OR EXCLUDED.keep RETURNING id",
            (digest, mime, w, h, len(data), bytes(data), kind, session_uid, captured_s, keep)).fetchone()
        return int(row[0])

    def _op_session(self, op: dict, image) -> None:
        from psycopg.types.json import Jsonb
        self.conn.execute(
            "INSERT INTO robot_session (uid, host, map_name, meta) VALUES (%s, %s, %s, %s) "
            "ON CONFLICT (uid) DO NOTHING",
            (op["session"], op.get("host", ""), op.get("map_name", ""), Jsonb(op.get("meta") or {})))

    def _op_event(self, op: dict, image) -> int:
        row = self.conn.execute("INSERT INTO event (session_uid, t_s, text) VALUES (%s, %s, %s) RETURNING id",
                                (op["session"], float(op["t_s"]), str(op["text"])[:500])).fetchone()
        if image is not None:
            self._insert_image(image, "event", op["session"], float(op["t_s"]))
        return int(row[0])

    def _op_sighting(self, op: dict, image) -> int:
        from psycopg.types.json import Jsonb
        image_id = (self._insert_image(image, "sighting", op["session"], float(op["captured_s"]))
                    if image is not None else None)
        row = self.conn.execute(
            "INSERT INTO sighting (session_uid, item, x, y, sigma_m, range_m, bearing_rad, captured_s, box, image_id) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) RETURNING id",
            (op["session"], op["item"], float(op["x"]), float(op["y"]), float(op["sigma_m"]),
             float(op["range_m"]), float(op["bearing_rad"]), float(op["captured_s"]),
             Jsonb(op["box"]) if op.get("box") is not None else None, image_id)).fetchone()
        return int(row[0])

    def _op_item(self, op: dict, image) -> None:
        # best photo = the newest sighting of this item that has one; marked keep so pruning spares it
        best = self.conn.execute(
            "SELECT image_id FROM sighting WHERE session_uid = %s AND item = %s AND image_id IS NOT NULL "
            "ORDER BY id DESC LIMIT 1", (op["session"], op["name"])).fetchone()
        best_id = int(best[0]) if best else None
        if best_id is not None:
            self.conn.execute("UPDATE image SET keep = true WHERE id = %s", (best_id,))
        self.conn.execute(
            "INSERT INTO item (session_uid, name, x, y, sigma_m, sightings, confirmed, last_seen_s, best_image_id) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s) "
            "ON CONFLICT (session_uid, name) DO UPDATE SET x = EXCLUDED.x, y = EXCLUDED.y, "
            "sigma_m = EXCLUDED.sigma_m, sightings = EXCLUDED.sightings, confirmed = EXCLUDED.confirmed, "
            "last_seen_s = EXCLUDED.last_seen_s, "
            "best_image_id = coalesce(EXCLUDED.best_image_id, item.best_image_id), updated_at = now()",
            (op["session"], op["name"], float(op["x"]), float(op["y"]), float(op["sigma_m"]),
             int(op["sightings"]), bool(op["confirmed"]), float(op["last_seen_s"]), best_id))

    def _op_mission(self, op: dict, image) -> None:
        self.conn.execute(
            "INSERT INTO mission (mission_id, session_uid, state, message, reason) VALUES (%s, %s, %s, %s, %s) "
            "ON CONFLICT (mission_id) DO UPDATE SET state = EXCLUDED.state, message = EXCLUDED.message, "
            "reason = EXCLUDED.reason, updated_at = now()",
            (op["mission_id"], op["session"], op["state"], op.get("message", ""), op.get("reason", "")))
        self.conn.execute("INSERT INTO mission_status (mission_id, state, message, reason) VALUES (%s, %s, %s, %s)",
                          (op["mission_id"], op["state"], op.get("message", ""), op.get("reason", "")))

    def _op_snapshot(self, op: dict, image) -> int:
        if image is None:
            raise StorageError("snapshot needs an image")
        return self._insert_image(image, op.get("kind", "snapshot"), op["session"], op.get("captured_s"),
                                  keep=bool(op.get("keep", False)))

    def _op_map(self, op: dict, image) -> int:
        lbmap = bytes.fromhex(op["lbmap_hex"])
        image_id = self._insert_image(image, "map", op["session"], None, keep=True) if image is not None else None
        row = self.conn.execute(
            "INSERT INTO map_snapshot (session_uid, name, lbmap, image_id) VALUES (%s, %s, %s, %s) RETURNING id",
            (op["session"], op["name"], lbmap, image_id)).fetchone()
        return int(row[0])

    # ------------------------------------------------------------------ reads
    def get_image(self, image_id: int) -> tuple[str, bytes] | None:
        with self.conn.transaction():
            row = self.conn.execute("SELECT mime, data FROM image WHERE id = %s", (int(image_id),)).fetchone()
        return (row[0], bytes(row[1])) if row else None

    def items(self, session_uid: str | None = None) -> list[dict]:
        sql = ("SELECT session_uid, name, x, y, sigma_m, sightings, confirmed, last_seen_s, best_image_id, updated_at "
               "FROM item")
        args: tuple = ()
        if session_uid:
            sql += " WHERE session_uid = %s"
            args = (session_uid,)
        sql += " ORDER BY confirmed DESC, updated_at DESC"
        with self.conn.transaction():
            rows = self.conn.execute(sql, args).fetchall()
        keys = ("session", "name", "x", "y", "sigma_m", "sightings", "confirmed", "last_seen_s", "photo_id", "updated_at")
        return [dict(zip(keys, r)) for r in rows]

    def events(self, session_uid: str, limit: int = 50) -> list[dict]:
        with self.conn.transaction():
            rows = self.conn.execute("SELECT t_s, text, created_at FROM event WHERE session_uid = %s "
                                     "ORDER BY id DESC LIMIT %s", (session_uid, int(limit))).fetchall()
        return [{"t_s": r[0], "text": r[1], "created_at": r[2]} for r in rows]

    def image_bytes_total(self) -> int:
        with self.conn.transaction():
            return int(self.conn.execute("SELECT coalesce(sum(nbytes), 0) FROM image").fetchone()[0])

    def prune(self, max_bytes: int) -> int:
        """Delete the oldest unprotected images until the table holds at most ``max_bytes``.
        Protected: ``keep`` images and any image an item or map snapshot points to.
        Returns the number of images deleted."""
        with self.conn.transaction():
            total = int(self.conn.execute("SELECT coalesce(sum(nbytes), 0) FROM image").fetchone()[0])
            if total <= max_bytes:
                return 0
            rows = self.conn.execute(
                "SELECT id, nbytes FROM image i WHERE NOT keep "
                "AND NOT EXISTS (SELECT 1 FROM item WHERE best_image_id = i.id) "
                "AND NOT EXISTS (SELECT 1 FROM map_snapshot WHERE image_id = i.id) "
                "ORDER BY created_at, id").fetchall()
            victims = []
            for image_id, nbytes in rows:
                if total <= max_bytes:
                    break
                victims.append(image_id)
                total -= int(nbytes)
            if victims:
                self.conn.execute("DELETE FROM image WHERE id = ANY(%s)", (victims,))
            return len(victims)


def op_to_json(op: dict) -> str:
    return json.dumps(op, separators=(",", ":"), allow_nan=False)
