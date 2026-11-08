import base64
import importlib.util
import json
import pathlib

from locatebot.remote.photon import WebhookVerifier, parse_event

ROOT = pathlib.Path(__file__).resolve().parents[2]
SECRET = "whsec_" + base64.b64encode(b"tool-test-secret-tool-test-secr!").decode()
PROJECT = "pho_prj_01jabcdefghjkmnpqrstvwxyz1"


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_fake_webhook_is_accepted_by_the_real_verifier_and_parser():
    tool = load("send_fake_webhook")
    ev = tool.build_event("/find blue toolbox", PROJECT, customer="b")
    headers, body = tool.build_request(ev, SECRET, now=1_794_000_000)
    WebhookVerifier(SECRET, clock=lambda: 1_794_000_000).verify(headers, body)
    msg = parse_event(body, PROJECT).message
    assert msg.text == "/find blue toolbox"
    assert msg.conversation.conversation_id == tool.CUSTOMERS["b"][0]


def test_fake_webhook_event_ids_are_photon_shaped_and_unique():
    tool = load("send_fake_webhook")
    ids = {tool.new_event_id() for _ in range(50)}
    assert len(ids) == 50
    import re
    assert all(re.match(r"^pho_evt_[0-7][0-9a-hjkmnp-tv-z]{25}$", i) for i in ids)


def test_fake_webhook_non_text_and_bad_signature():
    tool = load("send_fake_webhook")
    ev = tool.build_event(None, PROJECT)
    assert parse_event(json.dumps(ev).encode(), PROJECT).message.text is None
    headers, body = tool.build_request(ev, SECRET, now=1_794_000_000, bad_signature=True)
    import pytest
    from locatebot.remote.photon import WebhookVerificationError
    with pytest.raises(WebhookVerificationError):
        WebhookVerifier(SECRET, clock=lambda: 1_794_000_000).verify(headers, body)
