"""Photon WebSocket delivery: receive project events on the Pi without a public HTTPS endpoint.

Implemented from Photon's Beta docs, read live on 2026-11-08:
https://docs.photon.codes/docs/beta/message-delivery/websocket.md and the pages under
https://docs.photon.codes/docs/beta/websocket/ (session, events, cursors, errors, frames = the
AsyncAPI contract at https://api.photon.codes/asyncapi.yaml).

What the docs require, and how this file does it:

* "Photon uses **Socket.IO**, not a raw WebSocket protocol." There is no Socket.IO client in our
  dependencies (only `websockets`), so this file speaks the small subset we need of Engine.IO v4
  (https://socket.io/docs/v4/engine-io-protocol/) and Socket.IO v5 over a `websockets` connection:
  `0` open, `2`/`3` ping/pong, `1` close, `4` message; inside it `0` CONNECT, `1` DISCONNECT,
  `2` EVENT, `3` ACK, `4` CONNECT_ERROR. Binary packets are not used by this endpoint.
* Endpoint: `wss://api.photon.codes/v1/projects/{projectId}/events/ws?EIO=4&transport=websocket`
  ("Set the project route in `path`", `addTrailingSlash: false`, websocket transport only, default
  namespace). Auth: header `Authorization: Bearer <project API key>` (needs `project:read`).
* "On every `connect`, emit `subscribe` with `{ startSeq }` and an acknowledgement callback.
  Use `0` for new events. To resume, use the cursor of the last event you successfully processed."
  The ack is `{ok: true, apiVersion, connectionId, cursor, limits}` or `{ok: false, error: {code}}`.
* "Handle `delivery(event, metadata)`. The event is the same parsed public JSON object as a webhook";
  metadata is `{seq, replyExpected}`. "Deliveries have no processing acknowledgement callback":
  instead we save `seq` as our cursor (SQLite, same database) **after** the handler committed.
  A replayed `seq <= cursor` is skipped; a redelivered event id is skipped via the store's
  `events` table (the webhook path's dedupe table; the webhook's `webhook-id` is the event `id`).
* "If a handler fails, stop the stream before you process later events": a storage failure closes
  the connection and we resume from the last saved cursor after a backoff.
* `delivery_error {code, message, fatal, retryable, retryAfterMs?}` is logged and shown in `status()`;
  a fatal one is followed by the server's disconnect. `retryAfterMs` raises the next backoff.
* "Authentication failures before connection appear as `connect_error`; correct the credential or
  project rather than retrying indefinitely": an HTTP 401/403 on the upgrade or a Socket.IO
  CONNECT_ERROR waits `auth_retry_s` (5 min) before the next try, never a tight loop.
* Heartbeats: the server pings (`2`), we answer `3`; no packet for `pingInterval + pingTimeout`
  means the connection is dead. Reconnect backoff: 1 s doubling to a 30 s cap, with jitter.
* `replyExpected` / the `reply` command (synchronous HTTP replies, needs `project:write`) is not
  used: chat replies go through the messaging API (`PhotonClient`).
"""
from __future__ import annotations

import asyncio
import json
import logging
import random
import threading
import time
from typing import Awaitable, Callable, Optional

from .messaging import InboundMessage
from .photon import API_BASE, PhotonEventError, parse_event
from .store import Store

log = logging.getLogger(__name__)

AUTH_RETRY_S = 300.0
OPEN_TIMEOUT_S = 10.0
SUBSCRIBE_TIMEOUT_S = 10.0
MAX_FRAME_BYTES = 1 << 20
_SUB_ACK_ID = 0

Connect = Callable[[str, dict], Awaitable["object"]]  # (url, headers) -> websocket with send/recv/close


class AuthError(Exception):
    pass


class _Restart(Exception):
    """End this connection and reconnect after the usual backoff."""

    def __init__(self, reason: str, retry_after_s: float = 0.0, auth: bool = False):
        super().__init__(reason)
        self.retry_after_s, self.auth = retry_after_s, auth


class _RedactAuthorization(logging.Filter):
    """`websockets` logs every handshake header at DEBUG, including `Authorization: Bearer <key>`."""

    def filter(self, record: logging.LogRecord) -> bool:
        try:
            text = record.getMessage()
        except Exception:
            return True
        if "authorization" in text.lower() or "pho_sk_" in text:
            record.msg, record.args = "[handshake header redacted]", ()
        return True


_wire_log = logging.getLogger(__name__ + ".wire")
_wire_log.addFilter(_RedactAuthorization())


async def default_connect(url: str, headers: dict):
    """Open a websocket with the `websockets` package (imported lazily: optional dependency).
    Its logging goes to `locatebot.remote.photon_ws.wire`, with the Authorization header redacted."""
    try:
        from websockets.asyncio.client import connect  # websockets >= 13
        from websockets.exceptions import InvalidStatus
        kwargs = dict(additional_headers=headers, proxy=None)
    except ImportError:  # websockets 12: legacy client
        from websockets.client import connect  # type: ignore[no-redef]
        from websockets.exceptions import InvalidStatusCode as InvalidStatus  # type: ignore[no-redef]
        kwargs = dict(extra_headers=headers)
    try:
        return await connect(url, open_timeout=OPEN_TIMEOUT_S, close_timeout=0.5, max_size=MAX_FRAME_BYTES,
                             compression=None, logger=_wire_log, **kwargs)
    except InvalidStatus as e:
        response = getattr(e, "response", None)
        code = getattr(response, "status_code", None) or getattr(e, "status_code", None)
        if code in (401, 403):
            raise AuthError(f"HTTP {code}") from None
        raise ConnectionError(f"HTTP {code}") from None


class PhotonWebSocketInbound:
    """Receive Photon events over Socket.IO-on-WebSocket in a thread with its own asyncio loop.

    `on_message(InboundMessage)` is called once per new customer message, serially, in delivery
    order. It must record the event id itself (as `MissionService.handle_inbound` does, in the
    same transaction as the mission and replies) and raise `StorageUnavailable` if it could not.
    """

    def __init__(self, api_key: str, project_id: str, on_message: Callable[[InboundMessage], None], store: Store,
                 *, origin: str = API_BASE, api_version: Optional[str] = None,
                 clock: Callable[[], float] = time.time, connect: Connect = default_connect,
                 sleep: Callable[[float], Awaitable[None]] = asyncio.sleep, rng: Callable[[], float] = random.random,
                 backoff_initial_s: float = 1.0, backoff_max_s: float = 30.0, auth_retry_s: float = AUTH_RETRY_S,
                 subscribe_timeout_s: float = SUBSCRIBE_TIMEOUT_S):
        if not isinstance(api_key, str) or not api_key:
            raise ValueError("PhotonWebSocketInbound needs the project API key")
        self.__key = api_key
        self._project_id = project_id
        self._on_message, self._store = on_message, store
        base = origin.rstrip("/")
        base = "wss://" + base[len("https://"):] if base.startswith("https://") else \
            "ws://" + base[len("http://"):] if base.startswith("http://") else base
        self._url = f"{base}/v1/projects/{project_id}/events/ws?EIO=4&transport=websocket"
        self._api_version = api_version
        self._clock, self._connect, self._sleep, self._rng = clock, connect, sleep, rng
        self._b0, self._bmax, self._auth_retry_s = backoff_initial_s, backoff_max_s, auth_retry_s
        self._subscribe_timeout_s = subscribe_timeout_s
        self._cursor_name = f"photon:{project_id}"
        self._lock = threading.Lock()
        self._st = {"connected": False, "last_event_at": None, "reconnects": 0, "dropped_frames": 0,
                    "events": 0, "duplicates": 0, "last_error": None, "connection_id": None, "api_version": None}
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._task: Optional[asyncio.Task] = None
        self._stopping = False
        self._ws = None

    def __repr__(self) -> str:
        return f"PhotonWebSocketInbound(project_id={self._project_id!r})"

    @property
    def url(self) -> str:
        return self._url

    # --- lifecycle ------------------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stopping = False
        ready = threading.Event()

        def main():
            loop = asyncio.new_event_loop()
            self._loop = loop
            try:
                self._task = loop.create_task(self.run())
                ready.set()
                loop.run_until_complete(self._task)
            except asyncio.CancelledError:
                pass
            except Exception:
                log.exception("Photon inbound thread crashed")
            finally:
                ready.set()
                try:
                    loop.run_until_complete(loop.shutdown_asyncgens())
                finally:
                    loop.close()

        self._thread = threading.Thread(target=main, name="photon-ws", daemon=True)
        self._thread.start()
        ready.wait(2)

    def stop(self, timeout: float = 1.0) -> None:
        """Prompt: cancels the session (closing the socket) and joins the thread."""
        self._stopping = True
        loop, task = self._loop, self._task
        if loop is not None and task is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(task.cancel)
            except RuntimeError:
                pass
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None
        self._set(connected=False)

    def status(self) -> dict:
        with self._lock:
            st = dict(self._st)
        st["cursor"] = self._cursor()
        return st

    # --- main loop ------------------------------------------------------------------------------
    async def run(self) -> None:
        """Connect, subscribe, process deliveries; reconnect forever until cancelled/stopped."""
        attempt = 0
        while not self._stopping:
            try:
                subscribed = await self._session()
                delay, auth = 0.0, False
                reason = "connection closed"
            except _Restart as e:
                subscribed = getattr(e, "subscribed", False)
                delay, auth, reason = e.retry_after_s, e.auth, str(e)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # never let one bad connection kill the thread
                subscribed, delay, auth = False, 0.0, False
                reason = f"{type(e).__name__}: {e}"
            finally:
                self._set(connected=False)
                await self._close()
            if self._stopping:
                return
            if subscribed:
                attempt = 0
            reason = self._scrub(reason)
            self._set(last_error=reason)
            with self._lock:
                self._st["reconnects"] += 1
            if auth:
                wait = max(delay, self._auth_retry_s)
                log.error("Photon WebSocket authentication failed (%s); retrying in %.0f s. "
                          "Check PHOTON_API_KEY / PHOTON_PROJECT_ID.", reason, wait)
            else:
                base = min(self._bmax, self._b0 * 2 ** attempt)
                wait = max(delay, min(self._bmax, base * (0.8 + 0.4 * self._rng())))
                attempt += 1
                log.warning("Photon WebSocket disconnected (%s); reconnecting in %.1f s", reason, wait)
            await self._sleep(wait)

    async def _session(self) -> bool:
        headers = {"Authorization": f"Bearer {self.__key}"}
        try:
            self._ws = ws = await self._connect(self._url, headers)
        except AuthError as e:
            raise _Restart(f"authentication failed: {e}", auth=True) from None
        except (OSError, ConnectionError, asyncio.TimeoutError) as e:
            raise _Restart(f"connect failed: {type(e).__name__}: {e}") from None
        subscribed = False
        try:
            first = await asyncio.wait_for(ws.recv(), OPEN_TIMEOUT_S)
            if not isinstance(first, str) or not first.startswith("0"):
                raise _Restart("no Engine.IO open packet")
            try:
                hs = json.loads(first[1:])
                idle_s = (float(hs.get("pingInterval", 25000)) + float(hs.get("pingTimeout", 20000))) / 1000.0
            except (ValueError, TypeError, AttributeError):
                raise _Restart("bad Engine.IO open packet") from None
            await ws.send("40")  # Socket.IO CONNECT to the default namespace
            loop = asyncio.get_running_loop()
            sub_deadline = loop.time() + self._subscribe_timeout_s
            sub_sent = False
            while True:
                timeout = idle_s if subscribed else max(0.0, min(idle_s, sub_deadline - loop.time()))
                try:
                    pkt = await asyncio.wait_for(ws.recv(), timeout)
                except asyncio.TimeoutError:
                    if not subscribed and loop.time() >= sub_deadline:
                        raise _Restart("subscribe timed out") from None
                    raise _Restart("heartbeat timeout") from None
                if not isinstance(pkt, str) or not pkt:
                    self._drop("binary or empty frame")
                    continue
                kind, rest = pkt[0], pkt[1:]
                if kind == "2":
                    await ws.send("3" + rest)  # pong
                elif kind == "1":
                    raise _Restart("server closed the session")
                elif kind == "6":
                    pass
                elif kind == "4":
                    result = await self._socketio(ws, rest, sub_sent, subscribed)
                    if result == "connected":
                        sub = {"startSeq": self._cursor()}
                        if self._api_version:
                            sub["apiVersion"] = self._api_version
                        await ws.send(f"42{_SUB_ACK_ID}" + json.dumps(["subscribe", sub], separators=(",", ":")))
                        sub_sent = True
                    elif result == "subscribed":
                        subscribed = True
                else:
                    self._drop(f"unknown Engine.IO packet type {kind!r}")
        except _Restart as e:
            e.subscribed = subscribed  # type: ignore[attr-defined]
            raise
        except asyncio.CancelledError:
            raise
        except Exception as e:  # connection closed by the network, etc.
            r = _Restart(f"{type(e).__name__}: {e}")
            r.subscribed = subscribed  # type: ignore[attr-defined]
            raise r from None

    async def _socketio(self, ws, data: str, sub_sent: bool, subscribed: bool) -> Optional[str]:
        if not data:
            self._drop("empty Socket.IO packet")
            return None
        kind, rest = data[0], data[1:]
        if kind == "0":
            return None if sub_sent else "connected"
        if kind == "4":
            msg = ""
            try:
                msg = str(json.loads(rest).get("message", ""))[:200]
            except (ValueError, AttributeError):
                pass
            raise _Restart(f"connect_error {msg}".strip(), auth=True)
        if kind == "1":
            raise _Restart("server disconnected the socket")
        if kind not in "23":
            self._drop(f"unsupported Socket.IO packet type {kind!r}")
            return None
        i = 0
        while i < len(rest) and rest[i].isdigit():
            i += 1
        ack_id = int(rest[:i]) if i else None
        try:
            args = json.loads(rest[i:])
            if not isinstance(args, list):
                raise ValueError
        except ValueError:
            self._drop("Socket.IO payload is not a JSON array")
            return None
        if kind == "3":
            if ack_id != _SUB_ACK_ID or not sub_sent or subscribed:
                return None
            res = args[0] if args and isinstance(args[0], dict) else {}
            if res.get("ok") is True:
                self._set(connected=True, last_error=None, connection_id=res.get("connectionId"),
                          api_version=res.get("apiVersion"))
                log.info("Photon WebSocket subscribed (cursor %s)", self._cursor())
                return "subscribed"
            err = res.get("error") if isinstance(res.get("error"), dict) else {}
            raise _Restart(f"subscribe rejected: {err.get('code', 'unknown')}")
        if not args or not isinstance(args[0], str):
            self._drop("event without a name")
            return None
        name = args[0]
        if name == "delivery":
            if len(args) < 3 or not isinstance(args[1], dict) or not isinstance(args[2], dict):
                self._drop("malformed delivery")
                return None
            self._delivery(args[1], args[2])
        elif name == "delivery_error":
            err = args[1] if len(args) > 1 and isinstance(args[1], dict) else {}
            code = str(err.get("code", "unknown"))[:40]
            fatal = bool(err.get("fatal"))
            log.warning("Photon delivery_error %s (fatal=%s): %s", code, fatal, self._scrub(str(err.get("message", "")))[:200])
            self._set(last_error=f"delivery_error {code}")
            if fatal:
                retry_ms = err.get("retryAfterMs")
                retry = float(retry_ms) / 1000.0 if isinstance(retry_ms, (int, float)) and retry_ms > 0 else 0.0
                raise _Restart(f"delivery_error {code}", retry_after_s=retry)
        else:
            log.debug("ignoring Socket.IO event %r", name[:40])
        return None

    def _delivery(self, event: dict, meta: dict) -> None:
        seq = meta.get("seq")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
            self._drop("delivery without a valid seq")
            return
        cursor = self._cursor()
        if cursor and seq <= cursor:
            with self._lock:
                self._st["duplicates"] += 1
            return  # replay of something we already processed
        event_id = event.get("id")
        if not isinstance(event_id, str) or not event_id:
            self._drop("delivery without an event id")
            self._save_cursor(seq)
            return
        if self._store.has_event(event_id):
            with self._lock:
                self._st["duplicates"] += 1
            log.info("duplicate delivery %s ignored", event_id)
            self._save_cursor(seq)
            return
        try:
            parsed = parse_event(json.dumps(event).encode(), self._project_id, event_id)
        except PhotonEventError as e:
            log.error("Photon event %s has an unexpected shape: %s", event_id, e)
            self._drop("unexpected event shape")
            self._save_cursor(seq)  # a replay would not fix it
            return
        if parsed.message is not None:
            try:
                self._on_message(parsed.message)  # records the event id in the same transaction
            except Exception as e:
                # stop the stream; resume from the last saved cursor (docs: "Resume from the last cursor")
                raise _Restart(f"handler failed: {type(e).__name__}") from None
        else:
            log.debug("ignoring %s: %s", event_id, parsed.ignored)
        self._save_cursor(seq)
        with self._lock:
            self._st["events"] += 1
            self._st["last_event_at"] = self._clock()

    # --- helpers --------------------------------------------------------------------------------
    def _cursor(self) -> int:
        try:
            return self._store.get_cursor(self._cursor_name)
        except Exception:
            return 0

    def _save_cursor(self, seq: int) -> None:
        self._store.set_cursor(self._cursor_name, seq, self._clock())

    def _drop(self, why: str) -> None:
        with self._lock:
            self._st["dropped_frames"] += 1
        log.warning("dropped Photon frame: %s", why)

    def _set(self, **changes) -> None:
        with self._lock:
            self._st.update(changes)

    async def _close(self) -> None:
        ws, self._ws = self._ws, None
        if ws is None:
            return
        try:
            await asyncio.wait_for(ws.close(), 0.5)
        except BaseException:
            pass

    def _scrub(self, text: str) -> str:
        return text.replace(self.__key, "[redacted]")
