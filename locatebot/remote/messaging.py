"""Provider-neutral messaging types. Mission logic only sees these, never Photon payloads,
so another provider can be added by writing one more adapter."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Protocol

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Conversation:
    """Where to send replies: the provider, its conversation id, and the identity we reply as."""
    provider: str
    conversation_id: str
    reply_as: str


@dataclass(frozen=True)
class InboundMessage:
    event_id: str                 # provider's delivery id, used for de-duplication
    conversation: Conversation
    sender_id: str
    sender_handle: str
    text: Optional[str]           # None for non-text content


@dataclass(frozen=True)
class SendResult:
    ok: bool
    retryable: bool = False
    retry_after_s: Optional[float] = None
    detail: str = ""


class MessagingProvider(Protocol):
    name: str

    def send_text(self, conversation: Conversation, text: str, idempotency_key: str) -> SendResult: ...


class ConsoleProvider:
    """Prints replies instead of sending them (PHOTON_MODE=console) for local testing."""
    name = "photon"

    def send_text(self, conversation: Conversation, text: str, idempotency_key: str) -> SendResult:
        print(f"[reply -> {conversation.conversation_id}] {text}", flush=True)
        return SendResult(True)
