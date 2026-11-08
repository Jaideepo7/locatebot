"""Customer-facing text and reliable reply delivery.

`status_text` turns the robot's *reported* state into words. Only TARGET_DETECTED produces a
"detected" message, so we never claim a find the robot did not report.
`OutboxSender` delivers queued replies with retries and backoff; it runs on a background thread so a
slow or failing provider never blocks webhook handling or the robot link.
"""
from __future__ import annotations

import logging
import time
from typing import Callable, Optional

from .protocol import MissionState as S
from .messaging import MessagingProvider
from .store import Store

log = logging.getLogger(__name__)

START_TEXT = ("Hello! I'm your autonomous robot assistant. Tell me what object you want me to find.\n"
              "Send /find followed by a short description, for example: /find blue toolbox")
HELP_TEXT = ("Commands:\n"
             "/start - introduction\n"
             "/find <description> - ask the robot to search for an object, e.g. /find blue toolbox\n"
             "/status - current mission status\n"
             "/cancel - stop the current mission\n"
             "/help - this list")
RECEIVED = "Request received. Sending your target to the robot."
UNAVAILABLE = "The robot is currently unavailable. Please try again later."
NOT_COMMAND = "Send /find <object> to start a search, or /help to see what I can do."
UNKNOWN = "Sorry, I don't know that command.\n\n" + HELP_TEXT
NOT_TEXT = "I can only read text messages. Send /find <object>, for example: /find blue toolbox"
OTHER_CUSTOMER_BUSY = "The robot is busy with another request. Please try again in a few minutes."
RATE_LIMITED = "You're sending requests too quickly. Please wait a minute and try again."
NO_ACTIVE = "You don't have an active mission."
NO_MISSIONS = "You don't have any missions yet. Send /find <object> to start one."
CANCELLING = "Cancelling... I'll confirm once the robot has stopped."
CANCEL_UNREACHABLE = "I can't reach the robot right now. It stops on its own when it loses contact."
LINK_LOST = "Lost contact with the robot. It will stop on its own for safety."
RECONNECTED = "Reconnected to the robot."


def already_searching(target: str, same_target: bool) -> str:
    if same_target:
        return f"I'm already searching for: {target}."
    return f"I'm already searching for: {target}. Send /cancel first if you want something else."


_REJECT_REASONS = {"BUSY": "it is busy", "EXPIRED": "the request expired", "CLOCK_SKEW": "its clock is out of sync",
                   "PERCEPTION_UNAVAILABLE": "its camera system is not ready"}


def rejected_text(reason: Optional[str]) -> str:
    return f"The robot couldn't start this search ({_REJECT_REASONS.get(reason or '', 'it declined the request')})."


def status_text(old: S, new: S, target: str, reason: Optional[str] = None,
                message: Optional[str] = None) -> Optional[str]:
    """Message for a state change, or None when the customer should not be messaged.
    For NOT_FOUND the robot's own summary (what it found / missed) is appended when given."""
    if old == new:
        return None
    if new == S.SEARCHING:
        if old in (S.TARGET_DETECTED, S.APPROACHING):
            return f"Lost sight of the target. Searching again for: {target}."
        return f"The robot has started searching for: {target}."
    if new == S.TARGET_DETECTED:
        return "Target detected. Preparing to approach it."
    if new == S.APPROACHING:
        return "The robot is approaching the target."
    if new == S.COMPLETED:
        return "Search mission completed."
    if new == S.CANCELLED:
        if reason == "LINK_LOST":
            return "The robot stopped the search after losing contact with the server."
        return "Mission cancelled."
    if new == S.FAILED:
        return {
            "NOT_FOUND": f"I couldn't spot {target}. The search has ended." + (f" ({message})" if message else ""),
            "EXPIRED": f"The search for {target} timed out.",
            "LINK_LOST": "Mission ended: the robot is offline.",
            "NO_ACK": "The robot didn't respond. Please try again.",
            "ROBOT_OFFLINE": UNAVAILABLE,
        }.get(reason or "", f"The search for {target} ended without success.")
    return None  # PENDING / ACCEPTED: nothing to tell the customer yet


def describe(state: S, target: str) -> str:
    """One-line /status summary."""
    return {
        S.PENDING: f"Current mission: sending {target} to the robot.",
        S.ACCEPTED: f"Current mission: the robot accepted the search for {target}.",
        S.SEARCHING: f"Current mission: searching for {target}.",
        S.TARGET_DETECTED: f"Current mission: target detected for {target}.",
        S.APPROACHING: f"Current mission: approaching {target}.",
        S.COMPLETED: f"Your last mission ({target}) is completed.",
        S.FAILED: f"Your last mission ({target}) ended without success.",
        S.CANCELLED: f"Your last mission ({target}) was cancelled.",
    }[state]


class OutboxSender:
    def __init__(self, store: Store, provider: MessagingProvider, clock: Callable[[], float] = time.time,
                 max_attempts: int = 6, base_backoff_s: float = 2.0, max_backoff_s: float = 300.0):
        self._store, self._provider, self._clock = store, provider, clock
        self._max_attempts, self._base, self._max = max_attempts, base_backoff_s, max_backoff_s

    def run_once(self) -> int:
        sent = 0
        for item in self._store.due_outbox(self._clock()):
            try:
                result = self._provider.send_text(item.conversation, item.text, item.key)
                ok, retryable, after, detail = result.ok, result.retryable, result.retry_after_s, result.detail
            except Exception as e:
                ok, retryable, after, detail = False, True, None, f"{type(e).__name__}"
                log.exception("provider raised while sending %s", item.key)
            attempts = item.attempts + 1
            now = self._clock()
            if ok:
                self._store.mark_outbox(item.id, "sent", attempts, now)
                sent += 1
            elif retryable and attempts < self._max_attempts:
                delay = min(self._max, max(after or 0.0, self._base * 2 ** (attempts - 1)))
                self._store.mark_outbox(item.id, "pending", attempts, now + delay, detail)
                log.warning("send %s failed (%s); retry in %.0fs", item.key, detail, delay)
            else:
                self._store.mark_outbox(item.id, "failed", attempts, now, detail)
                log.error("giving up on reply %s: %s", item.key, detail)
        return sent
