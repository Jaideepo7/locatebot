"""`RemoteService`: everything the Pi runs for Telegram remote control, minus the HTTP server.

Wires the SQLite store, the messaging provider (Photon, or the console), `MissionService`, the
reply `OutboxSender`, the optional webhook handler and a `RobotLink`, plus one background thread
that runs `MissionService.tick()` (timeouts) and `OutboxSender.run_once()` (replies) every 0.5 s.
Replaces the FastAPI app factory of `feat/photon-dispatch` (`backend/app.py`).
"""
from __future__ import annotations

import logging
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Optional

from .config import Settings
from .messaging import ConsoleProvider, MessagingProvider
from .missions import MissionService
from .notifier import OutboxSender
from .photon import PhotonClient, PhotonWebhookHandler, WebhookVerifier
from .protocol import format_ts
from .robot import FakeRobotLink, RobotLink
from .store import Mission, Store

log = logging.getLogger(__name__)

TICK_S = 0.5


def mission_json(m: Mission) -> dict:
    iso = lambda t: format_ts(datetime.fromtimestamp(t, timezone.utc))
    return {"mission_id": m.mission_id, "state": m.state.value, "target_description": m.target,
            "issued_at": iso(m.issued_at), "expires_at": iso(m.expires_at), "updated_at": iso(m.updated_at),
            "last_message": m.last_message, "fail_reason": m.fail_reason, "cancel_requested": m.cancel_requested}


class RemoteService:
    def __init__(self, settings: Settings, *, provider: Optional[MessagingProvider] = None,
                 link: Optional[RobotLink] = None, clock: Callable[[], float] = time.time,
                 tick_s: float = TICK_S):
        self.settings = settings
        self.clock = clock
        self.store = Store(settings.db_path)
        self.link = link if link is not None else FakeRobotLink("happy", clock=clock)
        self.missions = MissionService(self.store, self.link, robot_id=settings.robot_id, clock=clock,
                                       ttl_s=settings.mission_ttl_s, ack_timeout_s=settings.ack_timeout_s,
                                       link_grace_s=settings.link_grace_s)
        self.link.listener = self.missions
        if provider is None:
            provider = ConsoleProvider() if settings.photon_mode == "console" else \
                PhotonClient(settings.photon_api_key, settings.photon_project_id)
        self.provider = provider
        self.outbox = OutboxSender(self.store, provider, clock=clock)
        self.webhook: Optional[PhotonWebhookHandler] = None
        if settings.webhook_enabled:
            self.webhook = PhotonWebhookHandler(WebhookVerifier(settings.photon_webhook_secret, clock=clock),
                                                settings.photon_project_id, self.missions.handle_inbound)
        self._tick_s = tick_s
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def __repr__(self) -> str:
        return f"RemoteService(robot_id={self.settings.robot_id!r}, provider={type(self.provider).__name__})"

    def step(self) -> None:
        """One background iteration (tests call this directly)."""
        for fn in (self.missions.tick, self.outbox.run_once):
            try:
                fn()
            except Exception:
                log.exception("remote background step failed")

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        start = getattr(self.link, "start", None)
        if callable(start):
            start()

        def loop():
            while not self._stop.wait(self._tick_s):
                self.step()

        self._thread = threading.Thread(target=loop, name="remote-background", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            self._thread = None
        stop = getattr(self.link, "stop", None)
        if callable(stop):
            stop()

    def health(self) -> dict:
        return {"ok": True, "robot_connected": bool(self.link.connected)}
