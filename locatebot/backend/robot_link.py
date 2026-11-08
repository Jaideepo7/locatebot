"""Backend side of the Jetson link: authentication and the single live connection.

Transport-agnostic: the FastAPI WebSocket route wraps its socket in an object with
`send_text(str)` (non-blocking) and `close(code)`, then calls attach/detach. A newer connection from
the same device replaces the old one (e.g. after the Jetson rebooted before the old TCP socket timed out).
"""
from __future__ import annotations

import hmac
import json
import logging
import threading
from typing import Optional, Protocol

log = logging.getLogger(__name__)

CLOSE_POLICY_VIOLATION = 1008
CLOSE_REPLACED = 1012  # "service restart": a newer connection took over


class RobotConnection(Protocol):
    def send_text(self, text: str) -> None: ...
    def close(self, code: int) -> None: ...


class LinkListener(Protocol):
    def on_robot_connected(self) -> None: ...
    def on_robot_disconnected(self) -> None: ...


class RobotLink:
    def __init__(self, device_id: str, token: str, listener: Optional[LinkListener] = None):
        self.device_id = device_id
        self.__token = token.encode()
        self.listener = listener
        self._lock = threading.Lock()
        self._conn: Optional[RobotConnection] = None

    def authenticate(self, device_id: Optional[str], authorization: Optional[str]) -> bool:
        if not device_id or not authorization or not authorization.startswith("Bearer "):
            return False
        token_ok = hmac.compare_digest(authorization[len("Bearer "):].encode(), self.__token)
        device_ok = hmac.compare_digest(device_id.encode(), self.device_id.encode())
        return token_ok and device_ok

    @property
    def connected(self) -> bool:
        return self._conn is not None

    def attach(self, conn: RobotConnection) -> None:
        with self._lock:
            old, self._conn = self._conn, conn
        if old is not None:
            log.warning("new connection from %s replaces the old one", self.device_id)
            try:
                old.close(CLOSE_REPLACED)
            except Exception:
                pass
        log.info("robot %s connected", self.device_id)
        if self.listener:
            self.listener.on_robot_connected()

    def detach(self, conn: RobotConnection) -> None:
        with self._lock:
            if self._conn is not conn:
                return  # an already-replaced socket closing: the robot is still online
            self._conn = None
        log.warning("robot %s disconnected", self.device_id)
        if self.listener:
            self.listener.on_robot_disconnected()

    def send(self, msg: dict) -> bool:
        conn = self._conn
        if conn is None:
            return False
        try:
            conn.send_text(json.dumps(msg, separators=(",", ":")))
            return True
        except Exception as e:
            log.warning("send to robot failed: %s", e)
            return False
