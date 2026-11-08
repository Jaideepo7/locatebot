"""Local HTTP routes of the Pi's remote-control service, stdlib only (replaces the branch's FastAPI app).

Routes
- ``GET  /healthz``                    ``RemoteService.health()`` (no auth)
- ``POST /api/photon/webhook``         Standard Webhooks signature; 204 OK/duplicate/ignored,
                                       400 bad signature, 503 storage down (Photon retries),
                                       413 body too large, 404 when no webhook secret is configured
- ``POST /api/missions``               ``{"target_description": "..."}`` + optional ``Idempotency-Key``:
                                       201 created, 200 replay, 409 busy, 422 invalid, 503 robot offline
- ``GET  /api/missions/{id}``          200 / 404 / 422
- ``POST /api/missions/{id}/cancel``   202 cancel *requested* (the robot confirms CANCELLED); 404 / 409

The ``/api/missions*`` routes need ``Authorization: Bearer $OPERATOR_API_TOKEN`` (constant-time
compare); without a configured token they answer 403. Bind 127.0.0.1 (the default) unless the
webhook must be reachable through a tunnel.
"""
from __future__ import annotations

import hmac
import json
import logging
import re
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .missions import AlreadyFinished, MissionNotFound, RobotBusy, RobotUnavailable
from .protocol import InvalidDescription
from .service import RemoteService, mission_json

log = logging.getLogger(__name__)

MAX_WEBHOOK_BYTES = 256 * 1024  # a Telegram text event is a few KB
MAX_BODY_BYTES = 4096
_MISSION_RE = re.compile(r"^/api/missions/([^/]+)(/cancel)?$")


class _HttpError(Exception):
    def __init__(self, code: int, detail: str, headers: dict | None = None):
        super().__init__(detail)
        self.code, self.detail, self.headers = code, detail, headers or {}


class RemoteHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, service: RemoteService):
        self.service = service
        self._operator_token = service.settings.operator_api_token.encode()
        super().__init__(address, _Handler)

    def operator_ok(self, authorization: str) -> bool:
        if not self._operator_token:
            raise _HttpError(403, "operator API disabled (set OPERATOR_API_TOKEN)")
        if not authorization.startswith("Bearer "):
            return False
        return hmac.compare_digest(authorization[len("Bearer "):].encode(), self._operator_token)


def make_server(service: RemoteService, host: str = "127.0.0.1", port: int = 8070) -> RemoteHTTPServer:
    """Create (not start) the server; call ``serve_forever()`` in a thread, ``shutdown()`` to stop."""
    return RemoteHTTPServer((host, port), service)


def _valid_uuid(value: str) -> bool:
    try:
        return str(uuid.UUID(value)) == value.lower()
    except ValueError:
        return False


class _Handler(BaseHTTPRequestHandler):
    server: RemoteHTTPServer
    protocol_version = "HTTP/1.1"
    server_version = "locatebot-remote"

    def log_message(self, format, *args):  # noqa: A002
        log.debug("%s - %s", self.address_string(), format % args)

    # --- helpers --------------------------------------------------------------------------------
    def _send(self, code: int, obj=None, headers: dict | None = None) -> None:
        body = b"" if obj is None else json.dumps(obj, allow_nan=False).encode()
        self.send_response(code)
        if obj is not None:
            self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        if self.close_connection:
            self.send_header("Connection", "close")
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _read_body(self, limit: int) -> bytes:
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = -1
        if n < 0:
            self.close_connection = True
            raise _HttpError(400, "bad Content-Length")
        if n > limit:
            self.close_connection = True  # don't read it; drop the connection after replying
            raise _HttpError(413, "body too large")
        return self.rfile.read(n) if n else b""

    def _operator(self) -> None:
        if not self.server.operator_ok(self.headers.get("Authorization", "")):
            raise _HttpError(401, "unauthorized", {"WWW-Authenticate": "Bearer"})

    def _mission_id(self, value: str) -> str:
        if not _valid_uuid(value):
            raise _HttpError(422, "mission_id must be a UUID")
        return value

    # --- routes ---------------------------------------------------------------------------------
    def do_GET(self):  # noqa: N802
        path = self.path.split("?", 1)[0]
        svc = self.server.service
        try:
            if path == "/healthz":
                self._send(200, svc.health())
                return
            m = _MISSION_RE.match(path)
            if not m or m.group(2):
                raise _HttpError(404, "not found")
            self._operator()
            mission = svc.missions.get(self._mission_id(m.group(1)))
            if mission is None:
                raise _HttpError(404, "not found")
            self._send(200, mission_json(mission))
        except _HttpError as e:
            self._send(e.code, {"detail": e.detail}, e.headers)

    def do_POST(self):  # noqa: N802
        path = self.path.split("?", 1)[0]
        svc = self.server.service
        try:
            if path == "/api/photon/webhook":
                if svc.webhook is None:
                    raise _HttpError(404, "webhook delivery is not configured")
                body = self._read_body(MAX_WEBHOOK_BYTES)  # raw bytes: the signature covers them exactly
                self._send(svc.webhook.handle(dict(self.headers.items()), body))
                return
            if path == "/api/missions":
                self._operator()
                raw = self._read_body(MAX_BODY_BYTES)
                try:
                    data = json.loads(raw or b"null")
                except (ValueError, UnicodeDecodeError):
                    raise _HttpError(422, "body must be JSON") from None
                target = data.get("target_description") if isinstance(data, dict) else None
                if not isinstance(target, str):
                    raise _HttpError(422, "target_description (string) is required")
                try:
                    mission, created = svc.missions.create_mission(target, self.headers.get("Idempotency-Key"))
                except InvalidDescription as e:
                    raise _HttpError(422, str(e)) from None
                except RobotBusy as e:
                    self._send(409, {"detail": "robot busy", "active_mission_id": e.mission.mission_id})
                    return
                except RobotUnavailable:
                    raise _HttpError(503, "robot unavailable") from None
                self._send(201 if created else 200, mission_json(mission))
                return
            m = _MISSION_RE.match(path)
            if not m or not m.group(2):
                raise _HttpError(404, "not found")
            self._operator()
            self._read_body(MAX_BODY_BYTES)
            try:
                mission = svc.missions.cancel(self._mission_id(m.group(1)))
            except MissionNotFound:
                raise _HttpError(404, "not found") from None
            except AlreadyFinished:
                raise _HttpError(409, "mission already finished") from None
            self._send(202, mission_json(mission))  # requested; the robot confirms CANCELLED
        except _HttpError as e:
            self._send(e.code, {"detail": e.detail}, e.headers)
        except Exception:  # never leak internals; a 5xx makes Photon retry the webhook
            log.exception("remote route %s failed", path)
            self.close_connection = True
            self._send(500, {"detail": "internal error"})
