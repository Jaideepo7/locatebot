"""SQLite persistence (stdlib): processed webhook ids, missions and the reply outbox.

One transaction covers "remember this event id + create the mission + queue the replies", as
Photon's docs require, so a crash can neither lose an accepted event nor do it twice.
All access goes through `lock`; the service holds it for each whole operation.
"""
from __future__ import annotations

import sqlite3
import threading
from contextlib import contextmanager
from dataclasses import dataclass, fields
from typing import Iterator, List, Optional

from ..dispatch.protocol import TERMINAL, MissionState
from .messaging.base import Conversation


class StorageUnavailable(Exception):
    pass


@dataclass
class Mission:
    mission_id: str
    robot_id: str
    target: str
    state: MissionState
    issued_at: float
    expires_at: float
    created_at: float
    updated_at: float
    provider: Optional[str] = None
    conversation_id: Optional[str] = None
    reply_as: Optional[str] = None
    sender_id: Optional[str] = None
    last_message: str = ""
    fail_reason: Optional[str] = None
    cancel_requested: bool = False
    link_lost_at: Optional[float] = None
    idempotency_key: Optional[str] = None

    @property
    def conversation(self) -> Optional[Conversation]:
        if not self.conversation_id:
            return None
        return Conversation(self.provider, self.conversation_id, self.reply_as)

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL


@dataclass
class OutboxItem:
    id: int
    key: str
    conversation: Conversation
    text: str
    attempts: int
    next_attempt_at: float
    status: str  # pending | sent | failed
    last_error: str


_COLUMNS = [f.name for f in fields(Mission)]
_ACTIVE = tuple(s.value for s in MissionState if s not in TERMINAL)

_SCHEMA = """
CREATE TABLE IF NOT EXISTS events (event_id TEXT PRIMARY KEY, received_at REAL NOT NULL);
CREATE TABLE IF NOT EXISTS missions (
    mission_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, target TEXT NOT NULL, state TEXT NOT NULL,
    issued_at REAL NOT NULL, expires_at REAL NOT NULL, created_at REAL NOT NULL, updated_at REAL NOT NULL,
    provider TEXT, conversation_id TEXT, reply_as TEXT, sender_id TEXT, last_message TEXT NOT NULL DEFAULT '',
    fail_reason TEXT, cancel_requested INTEGER NOT NULL DEFAULT 0, link_lost_at REAL,
    idempotency_key TEXT UNIQUE);
CREATE INDEX IF NOT EXISTS missions_conv ON missions (provider, conversation_id, created_at);
CREATE TABLE IF NOT EXISTS outbox (
    id INTEGER PRIMARY KEY AUTOINCREMENT, key TEXT NOT NULL UNIQUE, provider TEXT NOT NULL,
    conversation_id TEXT NOT NULL, reply_as TEXT NOT NULL, text TEXT NOT NULL,
    attempts INTEGER NOT NULL DEFAULT 0, next_attempt_at REAL NOT NULL,
    status TEXT NOT NULL DEFAULT 'pending', last_error TEXT NOT NULL DEFAULT '');
CREATE INDEX IF NOT EXISTS outbox_due ON outbox (status, next_attempt_at);
"""


class Store:
    def __init__(self, path: str = ":memory:"):
        self.lock = threading.RLock()
        try:
            self._db = sqlite3.connect(path, check_same_thread=False, isolation_level=None, timeout=5)
            self._db.row_factory = sqlite3.Row
            if path != ":memory:":
                self._db.execute("PRAGMA journal_mode=WAL")
            self._db.executescript(_SCHEMA)
        except sqlite3.Error as e:
            raise StorageUnavailable(str(e)) from e
        self._depth = 0

    @contextmanager
    def transaction(self) -> Iterator[None]:
        with self.lock:
            outer = self._depth == 0
            try:
                if outer:
                    self._db.execute("BEGIN IMMEDIATE")
                self._depth += 1
                try:
                    yield
                finally:
                    self._depth -= 1
                if outer:
                    self._db.execute("COMMIT")
            except BaseException as e:
                if outer and self._db.in_transaction:
                    self._db.execute("ROLLBACK")
                if isinstance(e, sqlite3.Error):
                    raise StorageUnavailable(str(e)) from e
                raise

    def _q(self, sql: str, args=()) -> sqlite3.Cursor:
        try:
            with self.lock:
                return self._db.execute(sql, args)
        except sqlite3.Error as e:
            raise StorageUnavailable(str(e)) from e

    # --- events -------------------------------------------------------------------------------
    def insert_event(self, event_id: str, now: float) -> bool:
        return self._q("INSERT OR IGNORE INTO events VALUES (?, ?)", (event_id, now)).rowcount == 1

    def has_event(self, event_id: str) -> bool:
        return self._q("SELECT 1 FROM events WHERE event_id = ?", (event_id,)).fetchone() is not None

    # --- missions -----------------------------------------------------------------------------
    @staticmethod
    def _mission(row) -> Optional[Mission]:
        if row is None:
            return None
        d = dict(row)
        d["state"] = MissionState(d["state"])
        d["cancel_requested"] = bool(d["cancel_requested"])
        return Mission(**d)

    def insert_mission(self, m: Mission) -> None:
        values = [getattr(m, c) for c in _COLUMNS]
        values[_COLUMNS.index("state")] = m.state.value
        self._q(f"INSERT INTO missions ({','.join(_COLUMNS)}) VALUES ({','.join('?' * len(_COLUMNS))})", values)

    def update_mission(self, mission_id: str, **changes) -> None:
        if "state" in changes:
            changes["state"] = MissionState(changes["state"]).value
        bad = set(changes) - set(_COLUMNS)
        if bad:
            raise ValueError(f"unknown mission columns {bad}")
        sets = ", ".join(f"{k} = ?" for k in changes)
        self._q(f"UPDATE missions SET {sets} WHERE mission_id = ?", [*changes.values(), mission_id])

    def get_mission(self, mission_id: str) -> Optional[Mission]:
        return self._mission(self._q("SELECT * FROM missions WHERE mission_id = ?", (mission_id,)).fetchone())

    def find_by_idempotency_key(self, key: str) -> Optional[Mission]:
        return self._mission(self._q("SELECT * FROM missions WHERE idempotency_key = ?", (key,)).fetchone())

    def active_missions(self, robot_id: str) -> List[Mission]:
        rows = self._q(f"SELECT * FROM missions WHERE robot_id = ? AND state IN ({','.join('?' * len(_ACTIVE))}) "
                       "ORDER BY created_at", (robot_id, *_ACTIVE)).fetchall()
        return [self._mission(r) for r in rows]

    def active_mission(self, robot_id: str) -> Optional[Mission]:
        active = self.active_missions(robot_id)
        return active[-1] if active else None

    def latest_for(self, provider: str, conversation_id: str, sender_id: Optional[str] = None) -> Optional[Mission]:
        sql, args = "SELECT * FROM missions WHERE provider = ? AND conversation_id = ?", [provider, conversation_id]
        if sender_id is not None:
            sql += " AND sender_id = ?"
            args.append(sender_id)
        return self._mission(self._q(sql + " ORDER BY created_at DESC, rowid DESC LIMIT 1", args).fetchone())

    def count_created_since(self, provider: str, conversation_id: str, since: float) -> int:
        return self._q("SELECT COUNT(*) FROM missions WHERE provider = ? AND conversation_id = ? AND created_at > ?",
                       (provider, conversation_id, since)).fetchone()[0]

    # --- outbox -------------------------------------------------------------------------------
    def enqueue(self, key: str, conversation: Conversation, text: str, now: float) -> bool:
        return self._q("INSERT OR IGNORE INTO outbox (key, provider, conversation_id, reply_as, text, next_attempt_at) "
                       "VALUES (?, ?, ?, ?, ?, ?)",
                       (key, conversation.provider, conversation.conversation_id, conversation.reply_as, text,
                        now)).rowcount == 1

    @staticmethod
    def _item(r) -> OutboxItem:
        return OutboxItem(r["id"], r["key"], Conversation(r["provider"], r["conversation_id"], r["reply_as"]),
                          r["text"], r["attempts"], r["next_attempt_at"], r["status"], r["last_error"])

    def due_outbox(self, now: float, limit: int = 20) -> List[OutboxItem]:
        rows = self._q("SELECT * FROM outbox WHERE status = 'pending' AND next_attempt_at <= ? ORDER BY id LIMIT ?",
                       (now, limit)).fetchall()
        return [self._item(r) for r in rows]

    def list_outbox(self) -> List[OutboxItem]:
        return [self._item(r) for r in self._q("SELECT * FROM outbox ORDER BY id").fetchall()]

    def mark_outbox(self, item_id: int, status: str, attempts: int, next_attempt_at: float, error: str = "") -> None:
        self._q("UPDATE outbox SET status = ?, attempts = ?, next_attempt_at = ?, last_error = ? WHERE id = ?",
                (status, attempts, next_attempt_at, error[:500], item_id))
