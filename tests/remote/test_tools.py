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


def test_env_file_parser(tmp_path):
    tool = load("remote_run")
    f = tmp_path / ".env"
    f.write_text("# comment\nA=1\nexport B = two \nC='quoted # not a comment'\nD=x # trailing\n\nnoequals\n")
    assert tool.parse_env_file(f) == {"A": "1", "B": "two", "C": "quoted # not a comment", "D": "x"}
    env = {"A": "keep"}
    tool.load_env(f, env)
    assert env["A"] == "keep" and env["B"] == "two"


def test_remote_run_bad_config_exits_2(capsys):
    tool = load("remote_run")
    import os
    old = dict(os.environ)
    try:
        for k in list(os.environ):
            if k.startswith("PHOTON_"):
                del os.environ[k]
        assert tool.main(["--env-file", "/nonexistent/.env", "--port", "0"]) == 2
    finally:
        os.environ.clear()
        os.environ.update(old)
    assert "PHOTON_PROJECT_ID" in capsys.readouterr().err


def test_remote_run_process_serves_webhook_and_stops_on_sigterm(tmp_path):
    import os
    import signal
    import subprocess
    import sys
    import time
    import urllib.request

    env = {k: v for k, v in os.environ.items() if not k.startswith(("PHOTON_", "OPERATOR_", "DISPATCH_"))}
    env.update({"PHOTON_INBOUND": "webhook", "PHOTON_WEBHOOK_SECRET": SECRET, "PHOTON_PROJECT_ID": PROJECT,
                "DISPATCH_DB_PATH": str(tmp_path / "d.db"), "PYTHONUNBUFFERED": "1"})
    proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "remote_run.py"), "--console", "--port", "0",
                             "--env-file", str(tmp_path / "missing.env"), "--fake-script", "stall"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        line = proc.stdout.readline()
        assert "listening on http://127.0.0.1:" in line, line
        port = int(line.split("http://127.0.0.1:")[1].split()[0])
        base = f"http://127.0.0.1:{port}"
        health = json.loads(urllib.request.urlopen(base + "/healthz", timeout=5).read())
        assert health["robot_connected"] is True and health["inbound"] == "webhook"
        tool = load("send_fake_webhook")
        headers, body = tool.build_request(tool.build_event("/find blue toolbox", PROJECT), SECRET)
        req = urllib.request.Request(base + "/api/photon/webhook", data=body, headers=headers, method="POST")
        assert urllib.request.urlopen(req, timeout=5).status == 204
        seen = []
        deadline = time.time() + 10
        while time.time() < deadline:
            seen.append(proc.stdout.readline())
            if "started searching for: blue toolbox" in seen[-1]:
                break
        assert any("Request received" in s for s in seen), seen
        assert "started searching for: blue toolbox" in seen[-1], seen
        t0 = time.time()
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(timeout=5) == 0
        assert time.time() - t0 < 3
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
