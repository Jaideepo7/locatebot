import base64
import json
import pathlib

import pytest

from locatebot.remote.messaging import Conversation
from locatebot.remote.photon import (PhotonClient, PhotonEventError, PhotonWebhookHandler,
                                                WebhookVerificationError, WebhookVerifier, parse_event,
                                                sign_payload)
from locatebot.remote.store import StorageUnavailable

FIXTURE = pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "photon_message_received.json"
PROJECT = "pho_prj_01jabcdefghjkmnpqrstvwxyz1"
SECRET = "whsec_" + base64.b64encode(b"locatebot-test-secret-32-bytes!!").decode()
T = 1_794_000_000


def fixture_body(**content):
    ev = json.loads(FIXTURE.read_text())
    if content:
        ev["data"]["object"]["content"] = content
    return ev


def signed(body: bytes, msg_id="pho_evt_01jabcdefghjkmnpqrstvwxyz0", ts=T, secret=SECRET):
    return {"webhook-id": msg_id, "webhook-timestamp": str(ts),
            "webhook-signature": sign_payload(secret, msg_id, ts, body)}


# --- W1-W9 signature verification -----------------------------------------------------------

def test_known_answer_vector():  # W1
    v = WebhookVerifier("whsec_MfKQ9r8GKYqrTwjUPD8ILPZIo2LaLaSw", clock=lambda: 1614265330)
    v.verify({"webhook-id": "msg_p5jXN8AQM9LWM0D4loKWxJek", "webhook-timestamp": "1614265330",
              "webhook-signature": "v1,g0hM9SsE+OTPJTGt/tmIKtSyZlE3uFJELVlNIOLJ1OE="}, b'{"test": 2432232314}')


def test_tampered_body_rejected():  # W2
    body = b'{"a":1}'
    h = signed(body)
    with pytest.raises(WebhookVerificationError):
        WebhookVerifier(SECRET, clock=lambda: T).verify(h, b'{"a":2}')


def test_reserialised_body_rejected():  # W3
    body = b'{"a": 1,  "b": 2}'
    h = signed(body)
    with pytest.raises(WebhookVerificationError):
        WebhookVerifier(SECRET, clock=lambda: T).verify(h, json.dumps(json.loads(body)).encode())


@pytest.mark.parametrize("skew,ok", [(-299, True), (299, True), (-301, False), (301, False)])
def test_timestamp_window(skew, ok):  # W4
    body = b"{}"
    v = WebhookVerifier(SECRET, clock=lambda: T + skew)
    if ok:
        v.verify(signed(body), body)
    else:
        with pytest.raises(WebhookVerificationError):
            v.verify(signed(body), body)


@pytest.mark.parametrize("missing", ["webhook-id", "webhook-timestamp", "webhook-signature"])
def test_missing_header_rejected(missing):  # W5
    h = signed(b"{}")
    del h[missing]
    with pytest.raises(WebhookVerificationError):
        WebhookVerifier(SECRET, clock=lambda: T).verify(h, b"{}")


def test_headers_case_insensitive():
    h = {k.title(): v for k, v in signed(b"{}").items()}
    WebhookVerifier(SECRET, clock=lambda: T).verify(h, b"{}")


def test_rotation_two_signatures_either_matches():  # W6
    other = "whsec_" + base64.b64encode(b"x" * 32).decode()
    h = signed(b"{}")
    old = sign_payload(other, h["webhook-id"], T, b"{}")
    h["webhook-signature"] = f"{old} {h['webhook-signature']}"
    WebhookVerifier(SECRET, clock=lambda: T).verify(h, b"{}")


def test_wrong_secret_or_version_rejected():  # W7
    other = "whsec_" + base64.b64encode(b"y" * 32).decode()
    with pytest.raises(WebhookVerificationError):
        WebhookVerifier(SECRET, clock=lambda: T).verify(signed(b"{}", secret=other), b"{}")
    h = signed(b"{}")
    h["webhook-signature"] = h["webhook-signature"].replace("v1,", "v2,")
    with pytest.raises(WebhookVerificationError):
        WebhookVerifier(SECRET, clock=lambda: T).verify(h, b"{}")


def test_constant_time_compare_used():  # W8
    src = pathlib.Path(__file__).resolve().parents[2].joinpath("locatebot/remote/photon.py").read_text()
    assert "hmac.compare_digest" in src


@pytest.mark.parametrize("bad", ["", "MfKQ9r8GKYqrTwjUPD8ILPZIo2LaLaSw", "whsec_", "whsec_!!!notbase64!!!"])
def test_bad_secret_refused_at_startup(bad):  # W9
    with pytest.raises(ValueError):
        WebhookVerifier(bad)


# --- W10-W14 event parsing -------------------------------------------------------------------

def test_parse_telegram_text_message():  # W10
    ev = parse_event(json.dumps(fixture_body()).encode(), PROJECT)
    m = ev.message
    assert ev.ignored is None
    assert m.event_id == "pho_evt_01jabcdefghjkmnpqrstvwxyz0"
    assert m.conversation == Conversation("photon", "pho_spc_01jabcdefghjkmnpqrstvwxyz2",
                                          "pho_usr_01jabcdefghjkmnpqrstvwxyz4")
    assert (m.sender_id, m.sender_handle, m.text) == ("pho_usr_01jabcdefghjkmnpqrstvwxyz3", "test_customer",
                                                      "/find blue toolbox")


def test_parse_non_text_content():  # P14
    ev = parse_event(json.dumps(fixture_body(type="attachment", attachmentId="x")).encode(), PROJECT)
    assert ev.message.text is None


def test_other_event_types_ignored():  # W11
    body = fixture_body()
    body["type"] = "message.delivered"
    assert parse_event(json.dumps(body).encode(), PROJECT).ignored


def test_other_sources_ignored():  # W12
    body = fixture_body()
    body["source"] = "sms.photon"
    assert parse_event(json.dumps(body).encode(), PROJECT).ignored


def test_other_project_ignored():  # W13
    assert parse_event(json.dumps(fixture_body()).encode(), "pho_prj_01jabcdefghjkmnpqrstvwxyz9").ignored


@pytest.mark.parametrize("mutate", [
    lambda b: b["data"]["object"].pop("space"),
    lambda b: b["data"]["object"]["space"].update(id="spc_bad"),
    lambda b: b["data"]["object"].pop("sentBy"),
    lambda b: b["data"]["object"].pop("receivedBy"),
    lambda b: b.pop("data"),
])
def test_schema_failures_raise(mutate):  # W14
    body = fixture_body()
    mutate(body)
    with pytest.raises(PhotonEventError):
        parse_event(json.dumps(body).encode(), PROJECT)


def test_addressed_to_used_when_received_by_missing():
    body = fixture_body()
    body["data"]["object"]["addressedTo"] = body["data"]["object"].pop("receivedBy")
    assert parse_event(json.dumps(body).encode(), PROJECT).message.conversation.reply_as.endswith("4")


# --- the framework-free webhook handler (HTTP status codes Photon reacts to) -----------------

class Sink:
    def __init__(self, exc=None):
        self.got, self.exc = [], exc

    def __call__(self, msg):
        if self.exc:
            raise self.exc
        self.got.append(msg)


def handler(sink):
    return PhotonWebhookHandler(WebhookVerifier(SECRET, clock=lambda: T), PROJECT, sink)


def test_handler_accepts_valid_event():
    s = Sink()
    body = json.dumps(fixture_body()).encode()
    assert handler(s).handle(signed(body), body) == 204
    assert s.got[0].text == "/find blue toolbox"


def test_handler_400_on_bad_signature_and_does_nothing():  # W2, S4
    s = Sink()
    body = json.dumps(fixture_body()).encode()
    assert handler(s).handle({"webhook-id": "x", "webhook-timestamp": str(T), "webhook-signature": "v1,AAAA"},
                             body) == 400
    assert s.got == []


def test_handler_2xx_on_schema_failure():  # W14: never 5xx, or Photon retries a bad event for days
    s = Sink()
    body = b'{"id":"pho_evt_x","type":"message.received","source":"telegram.photon","data":{}}'
    assert handler(s).handle(signed(body), body) == 204
    assert s.got == []


def test_handler_uses_signed_webhook_id_as_event_id():
    s = Sink()
    body = json.dumps(fixture_body()).encode()
    handler(s).handle(signed(body, msg_id="pho_evt_retry"), body)
    assert s.got[0].event_id == "pho_evt_retry"


def test_handler_503_when_storage_down():  # D5
    body = json.dumps(fixture_body()).encode()
    assert handler(Sink(StorageUnavailable("locked"))).handle(signed(body), body) == 503


def test_handler_ignored_event_is_2xx():
    body = fixture_body()
    body["type"] = "space.updated"
    raw = json.dumps(body).encode()
    s = Sink()
    assert handler(s).handle(signed(raw), raw) == 204 and s.got == []


# --- outbound send client --------------------------------------------------------------------

class FakeHttp:
    def __init__(self, *responses):
        self.calls, self.responses = [], list(responses)

    def __call__(self, url, headers, body, timeout):
        self.calls.append((url, headers, json.loads(body), timeout))
        r = self.responses.pop(0)
        if isinstance(r, Exception):
            raise r
        return r


CONV = Conversation("photon", "pho_spc_01jabcdefghjkmnpqrstvwxyz2", "pho_usr_01jabcdefghjkmnpqrstvwxyz4")
KEY = "pho_sk_" + "a" * 40


def client(http):
    return PhotonClient(KEY, PROJECT, http=http)


def test_send_builds_documented_request():
    http = FakeHttp((202, {}, b'{"status":"dispatched"}'))
    r = client(http).send_text(CONV, "Hello", "m1:SEARCHING")
    assert r.ok
    url, headers, body, _ = http.calls[0]
    assert url == f"https://api.photon.codes/v1/projects/{PROJECT}/messages"
    assert headers["Authorization"] == f"Bearer {KEY}"
    assert headers["idempotency-key"] == "m1:SEARCHING"
    assert headers["Content-Type"] == "application/json"
    assert body == {"senderId": CONV.reply_as, "spaceId": CONV.conversation_id,
                    "content": {"type": "text", "format": "plain", "text": "Hello"}}


@pytest.mark.parametrize("status", [408, 429, 500, 502, 503])
def test_retryable_statuses(status):  # N6
    r = client(FakeHttp((status, {"retry-after": "7"}, b""))).send_text(CONV, "x", "k")
    assert (r.ok, r.retryable) == (False, True)
    assert r.retry_after_s == 7.0


def test_network_errors_retryable():  # N8
    r = client(FakeHttp(TimeoutError("slow"))).send_text(CONV, "x", "k")
    assert (r.ok, r.retryable) == (False, True)


def test_problem_json_not_retried():  # N7
    body = json.dumps({"type": "https://photon.codes/docs/problems/validation-failed", "code": "VALIDATION_FAILED",
                       "status": 422, "title": "Validation failed"}).encode()
    r = client(FakeHttp((422, {"content-type": "application/problem+json"}, body))).send_text(CONV, "x", "k")
    assert (r.ok, r.retryable) == (False, False)
    assert "VALIDATION_FAILED" in r.detail


def test_failed_status_in_202_body_not_ok():
    r = client(FakeHttp((202, {}, b'{"status":"failed"}'))).send_text(CONV, "x", "k")
    assert not r.ok and not r.retryable


def test_api_key_never_in_result_detail():  # N9
    r = client(FakeHttp(RuntimeError(f"boom {KEY}"))).send_text(CONV, "x", "k")
    assert KEY not in r.detail
    assert KEY not in repr(client(FakeHttp()))


@pytest.mark.parametrize("key,project", [("sk_bad", PROJECT), (KEY, "prj_bad")])
def test_client_rejects_bad_credentials_format(key, project):
    with pytest.raises(ValueError):
        PhotonClient(key, project)
