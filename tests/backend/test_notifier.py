import pytest

from locatebot.backend import notifier
from locatebot.backend.messaging.base import Conversation, SendResult
from locatebot.backend.notifier import OutboxSender
from locatebot.backend.store import Store
from locatebot.dispatch.protocol import MissionState as S

CONV = Conversation("photon", "pho_spc_x", "pho_usr_bot")
REASONS = [None, "NOT_FOUND", "EXPIRED", "LINK_LOST", "NO_ACK", "ROBOT_OFFLINE", "BUSY", "WHATEVER"]


@pytest.mark.parametrize("new", list(S))
@pytest.mark.parametrize("reason", REASONS)
def test_only_target_detected_claims_a_find(new, reason):  # N2
    for old in S:
        text = notifier.status_text(old, new, "blue toolbox", reason) or ""
        claims = any(w in text.lower() for w in ("found", "detected", "located", "spotted it"))
        assert claims == (new == S.TARGET_DETECTED and old != new), (old, new, reason, text)


def test_spec_texts():  # N1
    assert notifier.status_text(S.ACCEPTED, S.SEARCHING, "blue toolbox") == \
        "The robot has started searching for: blue toolbox."
    assert notifier.status_text(S.SEARCHING, S.TARGET_DETECTED, "x") == "Target detected. Preparing to approach it."
    assert notifier.status_text(S.APPROACHING, S.COMPLETED, "x") == "Search mission completed."
    assert notifier.status_text(S.PENDING, S.ACCEPTED, "x") is None


class FakeProvider:
    name = "photon"

    def __init__(self, *results):
        self.results = list(results)
        self.calls = []

    def send_text(self, conversation, text, idempotency_key):
        self.calls.append((conversation, text, idempotency_key))
        r = self.results.pop(0) if self.results else SendResult(True)
        if isinstance(r, Exception):
            raise r
        return r


class Clock:
    t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def store():
    return Store(":memory:")


def test_sends_pending_once(store):
    store.enqueue("k1", CONV, "hello", 1000.0)
    store.enqueue("k1", CONV, "hello again", 1000.0)  # same key: ignored
    prov = FakeProvider()
    s = OutboxSender(store, prov, clock=Clock())
    assert s.run_once() == 1
    assert s.run_once() == 0
    assert prov.calls == [(CONV, "hello", "k1")]


def test_retry_with_backoff_and_same_key(store):  # N6, N8, D7
    clock = Clock()
    store.enqueue("k1", CONV, "hello", clock.t)
    prov = FakeProvider(SendResult(False, retryable=True), SendResult(False, retryable=True, retry_after_s=30))
    s = OutboxSender(store, prov, clock=clock, base_backoff_s=2)
    s.run_once()
    assert len(prov.calls) == 1
    s.run_once()
    assert len(prov.calls) == 1  # waiting for backoff
    clock.t += 2
    s.run_once()
    assert len(prov.calls) == 2
    clock.t += 29
    s.run_once()
    assert len(prov.calls) == 2  # Retry-After honoured
    clock.t += 1
    s.run_once()
    assert [c[2] for c in prov.calls] == ["k1", "k1", "k1"]
    assert store.list_outbox()[0].status == "sent"


def test_gives_up_after_max_attempts(store):
    clock = Clock()
    store.enqueue("k1", CONV, "hello", clock.t)
    prov = FakeProvider(*[SendResult(False, retryable=True)] * 10)
    s = OutboxSender(store, prov, clock=clock, max_attempts=3, base_backoff_s=1)
    for _ in range(10):
        s.run_once()
        clock.t += 100
    assert len(prov.calls) == 3 and store.list_outbox()[0].status == "failed"


def test_permanent_failure_not_retried(store):  # N7
    store.enqueue("k1", CONV, "hello", 1000.0)
    prov = FakeProvider(SendResult(False, retryable=False, detail="VALIDATION_FAILED"))
    s = OutboxSender(store, prov, clock=Clock())
    s.run_once()
    s.run_once()
    item = store.list_outbox()[0]
    assert len(prov.calls) == 1 and item.status == "failed" and "VALIDATION_FAILED" in item.last_error


def test_provider_exception_is_retried_not_fatal(store):
    clock = Clock()
    store.enqueue("k1", CONV, "hello", clock.t)
    s = OutboxSender(store, FakeProvider(RuntimeError("x")), clock=clock, base_backoff_s=1)
    s.run_once()
    clock.t += 5
    s.run_once()
    assert store.list_outbox()[0].status == "sent"


def test_one_bad_item_does_not_block_others(store):
    store.enqueue("k1", CONV, "a", 1000.0)
    store.enqueue("k2", CONV, "b", 1000.0)
    prov = FakeProvider(SendResult(False, retryable=False), SendResult(True))
    OutboxSender(store, prov, clock=Clock()).run_once()
    assert [o.status for o in store.list_outbox()] == ["failed", "sent"]
