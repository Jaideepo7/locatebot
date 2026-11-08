"""``DBReader``: thread-safe, best-effort reads for the operator page (photos, found items).

The HTTP server answers from several threads; one lock serialises them over one connection,
which reconnects lazily after a failure (at most once per ``retry_s``). Every method returns
``None``/``[]`` instead of raising when the database is unreachable, so the page keeps working.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from .db import RobotDB

log = logging.getLogger(__name__)


class DBReader:
    def __init__(self, connect: Callable[[], RobotDB] | str, retry_s: float = 5.0,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if isinstance(connect, str):
            dsn = connect
            connect = lambda: RobotDB.connect(dsn)  # noqa: E731
        self._connect = connect
        self._retry_s = retry_s
        self._clock = clock
        self._lock = threading.Lock()
        self._db: RobotDB | None = None
        self._next_try = 0.0
        self.last_error = ""

    def _get(self) -> RobotDB | None:
        if self._db is not None and not self._db.closed:
            return self._db
        if self._clock() < self._next_try:
            return None
        try:
            self._db = self._connect()
        except Exception as e:
            self._fail(e)
            return None
        return self._db

    def _fail(self, e: Exception) -> None:
        self.last_error = f"{type(e).__name__}: {e}"[:200]
        log.warning("storage reader: %s", e)
        if self._db is not None:
            self._db.close()
        self._db = None
        self._next_try = self._clock() + self._retry_s

    def get_image(self, image_id: int) -> tuple[str, bytes] | None:
        with self._lock:
            db = self._get()
            if db is None:
                return None
            try:
                return db.get_image(image_id)
            except Exception as e:
                self._fail(e)
                return None

    def items(self, session_uid: str | None = None) -> list[dict]:
        with self._lock:
            db = self._get()
            if db is None:
                return []
            try:
                return db.items(session_uid)
            except Exception as e:
                self._fail(e)
                return []

    def close(self) -> None:
        with self._lock:
            if self._db is not None:
                self._db.close()
            self._db = None
