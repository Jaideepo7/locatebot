"""Photon (Beta 2.0 API) adapter: webhook verification, event parsing and sending replies.

Based on https://photon.codes/docs/beta/message-delivery/webhook.md and the live schema at
https://api.photon.codes/openapi.json (read 2026-11-07):

* Webhooks follow Standard Webhooks: headers webhook-id / webhook-timestamp / webhook-signature;
  signature = base64(HMAC-SHA256(base64decode(secret after "whsec_"), f"{id}.{ts}.{raw_body}")),
  one or more space-separated "v1,<sig>" values; reject timestamps more than 5 minutes off.
* The body is the event itself. Telegram text arrives as type "message.received",
  source "telegram.photon", data.object = ReceivedTelegramMessage.
* Replies: POST /v1/projects/{projectId}/messages with Authorization: Bearer pho_sk_...,
  an idempotency-key header, body {senderId, spaceId, content: {type: text, format: plain, text}}.

Not confirmed on a live project yet: that senderId should be the bot user from `receivedBy`.
"""
from __future__ import annotations

import base64
import binascii
import hashlib
import hmac
import json
import logging
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Callable, Mapping, Optional

from ..store import StorageUnavailable
from .base import Conversation, InboundMessage, SendResult

log = logging.getLogger(__name__)

API_BASE = "https://api.photon.codes"
TOLERANCE_S = 300
_PROJECT_RE = re.compile(r"^pho_prj_[0-9a-z]{26}$")
_SPACE_RE = re.compile(r"^pho_spc_[0-9a-z]{26}$")
_USER_RE = re.compile(r"^pho_usr_[0-9a-z]{26}$")


class WebhookVerificationError(Exception):
    pass


class PhotonEventError(ValueError):
    pass


def _decode_secret(secret: str) -> bytes:
    if not isinstance(secret, str) or not secret.startswith("whsec_"):
        raise ValueError("Photon webhook secret must start with whsec_")
    try:
        key = base64.b64decode(secret[len("whsec_"):], validate=True)
    except (binascii.Error, ValueError):
        raise ValueError("Photon webhook secret is not valid base64 after whsec_") from None
    if len(key) < 16:
        raise ValueError("Photon webhook secret is too short")
    return key


def sign_payload(secret: str, msg_id: str, timestamp: int, body: bytes) -> str:
    """Return a "v1,<base64>" signature. Used by tests and tools/send_fake_webhook.py."""
    mac = hmac.new(_decode_secret(secret), f"{msg_id}.{timestamp}.".encode() + body, hashlib.sha256)
    return "v1," + base64.b64encode(mac.digest()).decode()


class WebhookVerifier:
    def __init__(self, secret: str, tolerance_s: float = TOLERANCE_S, clock: Callable[[], float] = time.time):
        self._key = _decode_secret(secret)  # fail at startup, not on the first request
        self._tolerance_s = tolerance_s
        self._clock = clock

    def verify(self, headers: Mapping[str, str], body: bytes) -> None:
        h = {k.lower(): v for k, v in headers.items()}
        msg_id, ts, sigs = h.get("webhook-id"), h.get("webhook-timestamp"), h.get("webhook-signature")
        if not msg_id or not ts or not sigs:
            raise WebhookVerificationError("missing Standard Webhooks headers")
        try:
            ts_i = int(ts)
        except ValueError:
            raise WebhookVerificationError("bad timestamp") from None
        if abs(self._clock() - ts_i) > self._tolerance_s:
            raise WebhookVerificationError("timestamp outside tolerance")
        expected = hmac.new(self._key, f"{msg_id}.{ts_i}.".encode() + body, hashlib.sha256).digest()
        for part in sigs.split():
            version, _, sig = part.partition(",")
            if version != "v1":
                continue
            try:
                got = base64.b64decode(sig, validate=True)
            except (binascii.Error, ValueError):
                continue
            if hmac.compare_digest(got, expected):
                return
        raise WebhookVerificationError("no matching signature")


@dataclass(frozen=True)
class ParsedEvent:
    event_id: str
    message: Optional[InboundMessage] = None
    ignored: Optional[str] = None  # reason, when the event is not a customer message for us


def _get(d, *path):
    for key in path:
        if not isinstance(d, dict) or key not in d:
            raise PhotonEventError("missing " + ".".join(path))
        d = d[key]
    return d


def _id(value, pattern: re.Pattern, what: str) -> str:
    if not isinstance(value, str) or not pattern.match(value):
        raise PhotonEventError(f"invalid {what}")
    return value


def parse_event(body: bytes, project_id: str, event_id: Optional[str] = None) -> ParsedEvent:
    try:
        ev = json.loads(body)
    except ValueError:
        raise PhotonEventError("body is not JSON") from None
    if not isinstance(ev, dict):
        raise PhotonEventError("event is not an object")
    event_id = event_id or ev.get("id")
    if not isinstance(event_id, str) or not event_id:
        raise PhotonEventError("missing event id")
    if ev.get("type") != "message.received":
        return ParsedEvent(event_id, ignored=f"event type {ev.get('type')!r}")
    if ev.get("source") != "telegram.photon":
        return ParsedEvent(event_id, ignored=f"source {ev.get('source')!r}")
    if _get(ev, "data", "projectId") != project_id:
        return ParsedEvent(event_id, ignored="other project")
    obj = _get(ev, "data", "object")
    space = _id(_get(obj, "space", "id"), _SPACE_RE, "space id")
    sender = _id(_get(obj, "sentBy", "id"), _USER_RE, "sender id")
    handle = obj["sentBy"].get("handle") if isinstance(obj["sentBy"].get("handle"), str) else ""
    bot = obj.get("receivedBy") or obj.get("addressedTo")
    if not isinstance(bot, dict):
        raise PhotonEventError("missing receivedBy/addressedTo")
    bot_id = _id(bot.get("id"), _USER_RE, "bot user id")
    content = _get(obj, "content")
    text = content.get("text") if isinstance(content, dict) and content.get("type") == "text" else None
    if text is not None and not isinstance(text, str):
        raise PhotonEventError("text is not a string")
    return ParsedEvent(event_id, InboundMessage(event_id, Conversation("photon", space, bot_id), sender, handle, text))


class PhotonWebhookHandler:
    """Framework-free webhook logic. Returns the HTTP status to answer Photon with:
    400 bad signature (not processed), 503 storage down (Photon retries), else 204."""

    def __init__(self, verifier: WebhookVerifier, project_id: str, on_message: Callable[[InboundMessage], None]):
        self._verifier, self._project_id, self._on_message = verifier, project_id, on_message

    def handle(self, headers: Mapping[str, str], body: bytes) -> int:
        try:
            self._verifier.verify(headers, body)
        except WebhookVerificationError as e:
            log.warning("rejected webhook: %s", e)
            return 400
        event_id = {k.lower(): v for k, v in headers.items()}["webhook-id"]  # signed; stable across retries
        try:
            ev = parse_event(body, self._project_id, event_id)
        except PhotonEventError as e:
            log.error("verified webhook %s has an unexpected shape: %s", event_id, e)
            return 204  # a retry would not fix it
        if ev.ignored:
            log.debug("ignoring %s: %s", event_id, ev.ignored)
            return 204
        try:
            self._on_message(ev.message)
        except StorageUnavailable as e:
            log.error("cannot persist %s: %s", event_id, e)
            return 503
        return 204


HttpPost = Callable[[str, dict, bytes, float], tuple]  # -> (status, headers, body)


def urllib_post(url: str, headers: dict, body: bytes, timeout: float) -> tuple:
    req = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, {k.lower(): v for k, v in r.headers.items()}, r.read()
    except urllib.error.HTTPError as e:
        return e.code, {k.lower(): v for k, v in (e.headers or {}).items()}, e.read() or b""


class PhotonClient:
    name = "photon"

    def __init__(self, api_key: str, project_id: str, base_url: str = API_BASE, http: HttpPost = urllib_post,
                 timeout_s: float = 10.0):
        if not isinstance(api_key, str) or not api_key.startswith("pho_sk_"):
            raise ValueError("PHOTON_API_KEY must be a project API key (pho_sk_...)")
        if not isinstance(project_id, str) or not _PROJECT_RE.match(project_id):
            raise ValueError("PHOTON_PROJECT_ID must look like pho_prj_...")
        self.__key = api_key
        self._url = f"{base_url}/v1/projects/{project_id}/messages"
        self._http, self._timeout_s = http, timeout_s

    def __repr__(self) -> str:
        return f"PhotonClient(url={self._url!r})"

    def send_text(self, conversation: Conversation, text: str, idempotency_key: str) -> SendResult:
        body = json.dumps({"senderId": conversation.reply_as, "spaceId": conversation.conversation_id,
                           "content": {"type": "text", "format": "plain", "text": text}}).encode()
        headers = {"Authorization": f"Bearer {self.__key}", "idempotency-key": idempotency_key,
                   "Content-Type": "application/json", "Accept": "application/json"}
        try:
            status, rh, rb = self._http(self._url, headers, body, self._timeout_s)
        except Exception as e:  # timeouts, DNS, connection resets: all worth retrying
            return SendResult(False, retryable=True, detail=self._scrub(f"{type(e).__name__}: {e}"))
        rh = {k.lower(): v for k, v in (rh or {}).items()}
        if 200 <= status < 300:
            try:
                sent_status = json.loads(rb or b"{}").get("status")
            except (ValueError, AttributeError):
                sent_status = None
            if sent_status == "failed":
                return SendResult(False, detail="Photon reported status=failed")
            return SendResult(True, detail=str(sent_status or status))
        retry_after = None
        try:
            retry_after = float(rh.get("retry-after", ""))
        except ValueError:
            pass
        detail = f"HTTP {status}"
        try:
            problem = json.loads(rb)
            detail += f" {problem.get('code', '')} {problem.get('title', '')}".rstrip()
        except (ValueError, AttributeError, TypeError):
            pass
        retryable = status in (408, 429) or status >= 500
        return SendResult(False, retryable=retryable, retry_after_s=retry_after, detail=self._scrub(detail))

    def _scrub(self, text: str) -> str:
        return text.replace(self.__key, "[redacted]")
