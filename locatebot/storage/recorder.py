"""``StorageRecorder``: non-blocking writes from the robot into PostgreSQL, with a disk spool.

Producers (the Orin runtime's vision callback, events, the mission bridge, map saves) call
``event``/``sighting``/``item``/``mission``/``map_snapshot``/``snapshot``. Each call only puts an
operation on a bounded in-memory queue and returns: **the robot never waits on the database**.

A single writer thread applies operations in order. If the database is unreachable (Pi off,
cable out, Postgres restarting) operations are appended to a spool directory on disk and
replayed in order once it is back, with exponential backoff between connection attempts.
Operations reference the session by a uid generated here, so replay needs no database ids.

Limits keep the Orin safe: the queue holds at most ``max_queue`` operations and
``max_queue_image_bytes`` of images (beyond that an image is dropped but its metadata kept),
and the spool is capped at ``max_spool_bytes`` (beyond that the oldest spooled files go).
"""
from __future__ import annotations

import json
import logging
import os
import queue
import socket
import threading
import time
import uuid
from pathlib import Path
from typing import Callable

from .db import RobotDB, StorageError, op_to_json

log = logging.getLogger(__name__)


class StorageRecorder:
    def __init__(self, connect: Callable[[], RobotDB] | str, spool_dir: str | os.PathLike, *,
                 max_queue: int = 2000, max_queue_image_bytes: int = 64 * 1024 * 1024,
                 max_spool_bytes: int = 256 * 1024 * 1024, prune_max_image_bytes: int = 2 * 1024 ** 3,
                 prune_every_s: float = 300.0, backoff_max_s: float = 30.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if isinstance(connect, str):
            dsn = connect
            connect = lambda: RobotDB.connect(dsn)  # noqa: E731
        self._connect = connect
        self.spool = Path(spool_dir)
        self.spool.mkdir(parents=True, exist_ok=True)
        self.max_queue_image_bytes = max_queue_image_bytes
        self.max_spool_bytes = max_spool_bytes
        self.prune_max_image_bytes = prune_max_image_bytes
        self.prune_every_s = prune_every_s
        self.backoff_max_s = backoff_max_s
        self._clock = clock
        self._q: queue.Queue = queue.Queue(maxsize=max_queue)
        self._lock = threading.Lock()
        self._queued_image_bytes = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._db: RobotDB | None = None
        self._next_try = 0.0
        self._backoff = 1.0
        self._last_prune = clock()
        self._seq = self._max_spool_seq()
        self.session_uid: str | None = None
        self.stats = {"queued": 0, "written": 0, "spooled": 0, "replayed": 0, "dropped_ops": 0,
                      "dropped_images": 0, "spool_evicted": 0, "errors": 0, "connected": False,
                      "last_error": "", "pruned_images": 0}

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> "StorageRecorder":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="storage-writer", daemon=True)
            self._thread.start()
        return self

    def stop(self, timeout: float = 5.0) -> None:
        """Flush what can be written within ``timeout``; the rest stays in the spool."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
        self._drain_to_spool()
        if self._db is not None:
            self._db.close()
            self._db = None

    def status(self) -> dict:
        with self._lock:
            out = dict(self.stats)
        out.update({"queue": self._q.qsize(), "spool_files": len(self._spool_files()),
                    "session": self.session_uid})
        return out

    # ------------------------------------------------------------------ producers (any thread, never block)
    def start_session(self, host: str | None = None, map_name: str = "", meta: dict | None = None) -> str:
        self.session_uid = uuid.uuid4().hex
        self._put({"op": "session", "host": host or socket.gethostname(), "map_name": map_name or "",
                   "meta": meta or {}})
        return self.session_uid

    def event(self, t_s: float, text: str, image: bytes | None = None) -> None:
        self._put({"op": "event", "t_s": float(t_s), "text": str(text)}, image)

    def sighting(self, item: str, x: float, y: float, sigma_m: float, range_m: float, bearing_rad: float,
                 captured_s: float, box=None, image: bytes | None = None) -> None:
        self._put({"op": "sighting", "item": item, "x": float(x), "y": float(y), "sigma_m": float(sigma_m),
                   "range_m": float(range_m), "bearing_rad": float(bearing_rad), "captured_s": float(captured_s),
                   "box": [float(v) for v in box] if box is not None else None}, image)

    def item(self, name: str, x: float, y: float, sigma_m: float, sightings: int, confirmed: bool,
             last_seen_s: float) -> None:
        self._put({"op": "item", "name": name, "x": float(x), "y": float(y), "sigma_m": float(sigma_m),
                   "sightings": int(sightings), "confirmed": bool(confirmed), "last_seen_s": float(last_seen_s)})

    def mission(self, mission_id: str, state: str, message: str = "", reason: str = "") -> None:
        self._put({"op": "mission", "mission_id": mission_id, "state": state, "message": message,
                   "reason": reason})

    def snapshot(self, image: bytes, kind: str = "snapshot", captured_s: float | None = None,
                 keep: bool = False) -> None:
        self._put({"op": "snapshot", "kind": kind, "captured_s": captured_s, "keep": bool(keep)}, image)

    def map_snapshot(self, name: str, lbmap: bytes, png: bytes | None = None) -> None:
        self._put({"op": "map", "name": name, "lbmap_hex": bytes(lbmap).hex()}, png)

    def _put(self, op: dict, image: bytes | None = None) -> None:
        if self.session_uid is None and op["op"] != "session":
            raise StorageError("call start_session() before recording")
        op["session"] = self.session_uid
        with self._lock:
            if image is not None and self._queued_image_bytes + len(image) > self.max_queue_image_bytes:
                image = None                       # keep the metadata, drop the picture
                self.stats["dropped_images"] += 1
            nbytes = len(image) if image is not None else 0
            try:
                self._q.put_nowait((op, image))
            except queue.Full:
                self.stats["dropped_ops"] += 1
                return
            self._queued_image_bytes += nbytes
            self.stats["queued"] += 1

    # ------------------------------------------------------------------ writer thread
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                op, image = self._q.get(timeout=0.2)
            except queue.Empty:
                op = None
            if op is not None:
                with self._lock:
                    self._queued_image_bytes -= len(image) if image is not None else 0
            try:
                if self._ensure_db():
                    self._replay_spool()
                    if op is not None:
                        self._write(op, image)
                    self._maybe_prune()
                elif op is not None:
                    self._spool_op(op, image)
            except Exception as e:  # never let the writer die
                self._note_error(e)
                if op is not None:
                    self._spool_op(op, image)

    def _ensure_db(self) -> bool:
        if self._db is not None and not self._db.closed:
            return True
        now = self._clock()
        if now < self._next_try:
            return False
        try:
            db = self._connect()
            db.migrate()
        except Exception as e:
            self._note_error(e)
            self._next_try = now + self._backoff
            self._backoff = min(self.backoff_max_s, self._backoff * 2)
            with self._lock:
                self.stats["connected"] = False
            return False
        self._db, self._backoff = db, 1.0
        with self._lock:
            self.stats["connected"] = True
        log.info("storage connected")
        return True

    def _write(self, op: dict, image: bytes | None) -> None:
        try:
            self._db.apply(op, image)
        except StorageError as e:          # bad data: log and drop, retrying won't help
            self._note_error(e)
            with self._lock:
                self.stats["dropped_ops"] += 1
            return
        except Exception:
            self._drop_connection()
            raise
        with self._lock:
            self.stats["written"] += 1

    def _drop_connection(self) -> None:
        if self._db is not None:
            self._db.close()
        self._db = None
        with self._lock:
            self.stats["connected"] = False
        self._next_try = self._clock() + self._backoff

    def _maybe_prune(self) -> None:
        if self._clock() - self._last_prune < self.prune_every_s:
            return
        self._last_prune = self._clock()
        n = self._db.prune(self.prune_max_image_bytes)
        if n:
            with self._lock:
                self.stats["pruned_images"] += n

    def _note_error(self, e: Exception) -> None:
        with self._lock:
            self.stats["errors"] += 1
            self.stats["last_error"] = f"{type(e).__name__}: {e}"[:300]
        log.warning("storage: %s", e)

    # ------------------------------------------------------------------ spool
    def _spool_files(self) -> list[Path]:
        return sorted(self.spool.glob("*.json"))

    def _max_spool_seq(self) -> int:
        seqs = [int(p.stem) for p in self.spool.glob("*.json") if p.stem.isdigit()]
        return max(seqs, default=0)

    def _spool_op(self, op: dict, image: bytes | None) -> None:
        try:
            self._seq += 1
            stem = self.spool / f"{self._seq:012d}"
            if image is not None:
                tmp = stem.with_suffix(".bin.tmp")
                tmp.write_bytes(image)
                os.replace(tmp, stem.with_suffix(".bin"))
            tmp = stem.with_suffix(".json.tmp")
            tmp.write_text(op_to_json({**op, "_has_image": image is not None}))
            os.replace(tmp, stem.with_suffix(".json"))   # the .json appears last: replay never sees half an op
            with self._lock:
                self.stats["spooled"] += 1
            self._enforce_spool_cap()
        except (OSError, ValueError) as e:
            self._note_error(e)
            with self._lock:
                self.stats["dropped_ops"] += 1

    def _enforce_spool_cap(self) -> None:
        files = self._spool_files()
        total = sum(p.stat().st_size + (p.with_suffix(".bin").stat().st_size if p.with_suffix(".bin").exists() else 0)
                    for p in files)
        for p in files:  # oldest first; keep the session record so later ops still have a parent
            if total <= self.max_spool_bytes:
                break
            try:
                if json.loads(p.read_text()).get("op") == "session":
                    continue
            except (OSError, ValueError):
                pass
            b = p.with_suffix(".bin")
            total -= p.stat().st_size + (b.stat().st_size if b.exists() else 0)
            b.unlink(missing_ok=True)
            p.unlink(missing_ok=True)
            with self._lock:
                self.stats["spool_evicted"] += 1

    def _replay_spool(self) -> None:
        for p in self._spool_files():
            if self._stop.is_set():
                return
            try:
                op = json.loads(p.read_text())
            except (OSError, ValueError) as e:
                self._note_error(e)
                p.unlink(missing_ok=True)
                continue
            b = p.with_suffix(".bin")
            image = b.read_bytes() if op.pop("_has_image", False) and b.exists() else None
            self._write(op, image)   # a connection error raises: the file stays for the next try
            b.unlink(missing_ok=True)
            p.unlink(missing_ok=True)
            with self._lock:
                self.stats["replayed"] += 1

    def _drain_to_spool(self) -> None:
        while True:
            try:
                op, image = self._q.get_nowait()
            except queue.Empty:
                return
            self._spool_op(op, image)
