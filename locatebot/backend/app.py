"""FastAPI wiring. All decisions live in the framework-free services; this file only adapts HTTP/WebSocket.

Run (needs the `backend` extra):
    uvicorn --factory locatebot.backend.app:create_app --host 0.0.0.0 --port 8000 \
        --ws-ping-interval 10 --ws-ping-timeout 10
The ping settings make a half-open Jetson connection detectable in ~20 s.
"""
from __future__ import annotations

import asyncio
import hmac
import logging
import threading
import time
import uuid
from contextlib import asynccontextmanager
from typing import Callable, Optional

from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import JSONResponse, Response
from starlette.concurrency import run_in_threadpool

from ..dispatch.protocol import InvalidDescription, format_ts
from .config import Settings
from .messaging.base import ConsoleProvider, MessagingProvider
from .messaging.photon import PhotonClient, PhotonWebhookHandler, WebhookVerifier
from .missions import AlreadyFinished, MissionNotFound, MissionService, RobotBusy, RobotUnavailable
from .notifier import OutboxSender
from .robot_link import CLOSE_POLICY_VIOLATION, RobotLink
from .store import Mission, Store

log = logging.getLogger(__name__)
CLOSE_UNSUPPORTED_DATA = 1003
_SEND_QUEUE_MAX = 100
_MAX_WEBHOOK_BYTES = 256 * 1024  # a Telegram text event is a few KB


def mission_json(m: Mission) -> dict:
    from datetime import datetime, timezone
    iso = lambda t: format_ts(datetime.fromtimestamp(t, timezone.utc))
    return {"mission_id": m.mission_id, "state": m.state.value, "target_description": m.target,
            "issued_at": iso(m.issued_at), "expires_at": iso(m.expires_at), "updated_at": iso(m.updated_at),
            "last_message": m.last_message, "fail_reason": m.fail_reason, "cancel_requested": m.cancel_requested}


class _WsConnection:
    """RobotConnection for a Starlette WebSocket; send_text is safe to call from any thread."""

    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=_SEND_QUEUE_MAX)

    def send_text(self, text: str) -> None:
        if self.queue.qsize() >= _SEND_QUEUE_MAX - 1:
            raise ConnectionError("robot send queue full")
        self.loop.call_soon_threadsafe(self.queue.put_nowait, text)

    def close(self, code: int) -> None:
        self.loop.call_soon_threadsafe(self.queue.put_nowait, code)


def create_app(settings: Optional[Settings] = None, *, provider: Optional[MessagingProvider] = None,
               clock: Callable[[], float] = time.time, background: bool = True) -> FastAPI:
    settings = settings or Settings.from_env()
    store = Store(settings.db_path)
    link = RobotLink(settings.jetson_device_id, settings.jetson_auth_token)
    service = MissionService(store, link, robot_id=settings.jetson_device_id, clock=clock,
                             ttl_s=settings.mission_ttl_s, ack_timeout_s=settings.ack_timeout_s,
                             link_grace_s=settings.link_grace_s)
    link.listener = service
    if provider is None:
        provider = ConsoleProvider() if settings.photon_mode == "console" else \
            PhotonClient(settings.photon_api_key, settings.photon_project_id)
    outbox = OutboxSender(store, provider, clock=clock)
    webhook = PhotonWebhookHandler(WebhookVerifier(settings.photon_webhook_secret, clock=clock),
                                   settings.photon_project_id, service.handle_inbound)
    operator_token = settings.operator_api_token.encode()
    stop = threading.Event()

    def background_loop():
        while not stop.wait(0.5):
            for step in (service.tick, outbox.run_once):
                try:
                    step()
                except Exception:
                    log.exception("background step failed")

    @asynccontextmanager
    async def lifespan(_app):
        worker = threading.Thread(target=background_loop, name="dispatch-background", daemon=True)
        if background:
            worker.start()
        try:
            yield
        finally:
            stop.set()
            if background:
                worker.join(timeout=5)

    app = FastAPI(title="locatebot dispatch", lifespan=lifespan, docs_url=None, redoc_url=None)
    app.state.service, app.state.store, app.state.link, app.state.outbox = service, store, link, outbox

    def operator_ok(request: Request) -> bool:
        auth = request.headers.get("authorization", "")
        return auth.startswith("Bearer ") and hmac.compare_digest(auth[7:].encode(), operator_token)

    def unauthorized():
        return JSONResponse({"detail": "unauthorized"}, status_code=401, headers={"WWW-Authenticate": "Bearer"})

    def valid_uuid(value: str) -> bool:
        try:
            return str(uuid.UUID(value)) == value.lower()
        except ValueError:
            return False

    @app.get("/healthz")
    async def healthz():
        return {"ok": True, "robot_connected": link.connected}

    @app.post("/api/photon/webhook")
    async def photon_webhook(request: Request):
        body = await request.body()  # raw bytes: the signature covers them exactly
        if len(body) > _MAX_WEBHOOK_BYTES:
            return Response(status_code=413)
        status = await run_in_threadpool(webhook.handle, dict(request.headers), body)
        return Response(status_code=status)

    @app.post("/api/missions")
    async def create_mission(request: Request):
        if not operator_ok(request):
            return unauthorized()
        try:
            data = await request.json()
        except ValueError:
            return JSONResponse({"detail": "body must be JSON"}, status_code=422)
        target = data.get("target_description") if isinstance(data, dict) else None
        if not isinstance(target, str):
            return JSONResponse({"detail": "target_description (string) is required"}, status_code=422)
        try:
            m, created = await run_in_threadpool(service.create_mission, target,
                                                 request.headers.get("idempotency-key"))
        except InvalidDescription as e:
            return JSONResponse({"detail": str(e)}, status_code=422)
        except RobotBusy as e:
            return JSONResponse({"detail": "robot busy", "active_mission_id": e.mission.mission_id}, status_code=409)
        except RobotUnavailable:
            return JSONResponse({"detail": "robot unavailable"}, status_code=503)
        return JSONResponse(mission_json(m), status_code=201 if created else 200)

    @app.get("/api/missions/{mission_id}")
    async def get_mission(mission_id: str, request: Request):
        if not operator_ok(request):
            return unauthorized()
        if not valid_uuid(mission_id):
            return JSONResponse({"detail": "mission_id must be a UUID"}, status_code=422)
        m = await run_in_threadpool(service.get, mission_id)
        if m is None:
            return JSONResponse({"detail": "not found"}, status_code=404)
        return mission_json(m)

    @app.post("/api/missions/{mission_id}/cancel")
    async def cancel_mission(mission_id: str, request: Request):
        if not operator_ok(request):
            return unauthorized()
        if not valid_uuid(mission_id):
            return JSONResponse({"detail": "mission_id must be a UUID"}, status_code=422)
        try:
            m = await run_in_threadpool(service.cancel, mission_id)
        except MissionNotFound:
            return JSONResponse({"detail": "not found"}, status_code=404)
        except AlreadyFinished:
            return JSONResponse({"detail": "mission already finished"}, status_code=409)
        return JSONResponse(mission_json(m), status_code=202)  # requested; the robot confirms CANCELLED

    @app.websocket("/ws/robot")
    async def robot_ws(ws: WebSocket):
        # Credentials only in headers: query strings end up in proxy and access logs.
        if ws.query_params or not link.authenticate(ws.headers.get("x-device-id"), ws.headers.get("authorization")):
            log.warning("rejected robot connection from %s", ws.client)
            await ws.close(code=CLOSE_POLICY_VIOLATION)
            return
        await ws.accept()
        conn = _WsConnection(asyncio.get_running_loop())

        async def writer():
            while True:
                item = await conn.queue.get()
                if isinstance(item, int):
                    await ws.close(code=item)
                    return
                await ws.send_text(item)

        writer_task = asyncio.create_task(writer())
        await run_in_threadpool(link.attach, conn)
        try:
            while True:
                msg = await ws.receive()
                if msg["type"] == "websocket.disconnect":
                    break
                if msg.get("bytes") is not None:
                    await ws.close(code=CLOSE_UNSUPPORTED_DATA)
                    break
                if msg.get("text") is not None:
                    await run_in_threadpool(service.on_robot_frame, msg["text"])
        except Exception as e:
            log.warning("robot socket error: %s", e)
        finally:
            writer_task.cancel()
            await run_in_threadpool(link.detach, conn)

    return app
