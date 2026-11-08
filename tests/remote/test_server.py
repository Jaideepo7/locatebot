"""HTTP layer: the stdlib server on 127.0.0.1 port 0, driven with urllib (replaces the FastAPI
TestClient tests of feat/photon-dispatch). No FastAPI, uvicorn, httpx or websockets needed."""
import base64
import json
import pathlib
import threading
import time
import urllib.error
import urllib.request

import pytest

from locatebot.remote.config import Settings
from locatebot.remote.messaging import SendResult
from locatebot.remote.photon import sign_payload
from locatebot.remote.protocol import MissionState as S
from locatebot.remote.robot import FakeRobotLink, Step
from locatebot.remote.server import make_server
from locatebot.remote.service import RemoteService

FIXTURE = json.loads((pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "photon_message_received.json").read_text())
SECRET = "whsec_" + base64.b64encode(b"app-test-secret-app-test-secret!").decode()
OTOKEN = "o" * 40
OP = {"Authorization": f"Bearer {OTOKEN}"}
MID = "3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70"


class Recorder:
    name = "photon"

    def __init__(self):
        self.sent = []

    def send_text(self, conversation, text, idempotency_key):
        self.sent.append(text)
        return SendResult(True)


def settings(tmp_path, **extra):
    env = {"PHOTON_MODE": "console", "PHOTON_PROJECT_ID": FIXTURE["data"]["projectId"],
           "PHOTON_WEBHOOK_SECRET": SECRET, "OPERATOR_API_TOKEN": OTOKEN, "PHOTON_INBOUND": "webhook",
           "DISPATCH_DB_PATH": str(tmp_path / "t.db")}
    env.update(extra)
    return Settings.from_env({k: v for k, v in env.items() if v is not None})


class Rig:
    def __init__(self, tmp_path, script="stall", online=True, **extra):
        self.rec = Recorder()
        self.link = FakeRobotLink(script, online=online)
        self.svc = RemoteService(settings(tmp_path, **extra), provider=self.rec, link=self.link)
        self.httpd = make_server(self.svc, "127.0.0.1", 0)
        self.base = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        self.thread = threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        self.thread.start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.svc.stop()

    def req(self, method, path, body=None, headers=None, raw=None):
        data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
        h = {"Content-Type": "application/json"} if data is not None else {}
        h.update(headers or {})
        r = urllib.request.Request(self.base + path, data=data, headers=h, method=method)
        try:
            with urllib.request.urlopen(r, timeout=5) as resp:
                payload = resp.read()
                return resp.status, json.loads(payload) if payload else None
        except urllib.error.HTTPError as e:
            payload = e.read()
            return e.code, json.loads(payload) if payload else None

    def webhook(self, text, event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz0", sign=True):
        ev = json.loads(json.dumps(FIXTURE))
        ev["id"] = event_id
        ev["data"]["object"]["content"]["text"] = text
        body = json.dumps(ev).encode()
        ts = int(time.time())
        sig = sign_payload(SECRET, event_id, ts, body) if sign else "v1,AAAA"
        return self.req("POST", "/api/photon/webhook", raw=body, headers={
            "webhook-id": event_id, "webhook-timestamp": str(ts), "webhook-signature": sig})[0]


@pytest.fixture
def rig(tmp_path):
    r = Rig(tmp_path)
    yield r
    r.close()


def test_healthz(rig):
    assert rig.req("GET", "/healthz") == (200, {"ok": True, "robot_connected": True, "inbound": "webhook",
                                               "photon_connected": None, "replies_pending": 0})


def test_webhook_signature_enforced(rig):
    assert rig.webhook("/start", sign=False) == 400
    assert rig.webhook("/start") == 204
    assert rig.webhook("/start") == 204  # duplicate: still 2xx
    rig.svc.outbox.run_once()
    assert len(rig.rec.sent) == 1 and rig.rec.sent[0].startswith("Hello!")


def test_webhook_too_large(rig):
    code, _ = rig.req("POST", "/api/photon/webhook", raw=b"x" * (256 * 1024 + 1))
    assert code == 413


def test_find_while_robot_offline(tmp_path):
    r = Rig(tmp_path, online=False)
    try:
        r.webhook("/find blue toolbox")
        r.svc.outbox.run_once()
        assert r.rec.sent == ["The robot is currently unavailable. Please try again later."]
        assert r.req("GET", "/healthz")[1]["robot_connected"] is False
    finally:
        r.close()


def test_full_flow_through_routes_and_background_thread(tmp_path):
    fast = [Step(0.1, S.SEARCHING), Step(0.2, S.TARGET_DETECTED), Step(0.3, S.APPROACHING),
            Step(0.4, S.COMPLETED)]
    r = Rig(tmp_path, script=fast)
    r.svc._tick_s = 0.05
    try:
        r.svc.start()  # also starts the fake robot's thread
        assert r.webhook("/find blue toolbox") == 204
        deadline = time.time() + 15
        while time.time() < deadline and (not r.rec.sent or r.rec.sent[-1] != "Search mission completed."):
            time.sleep(0.05)
        assert r.rec.sent[0] == "Request received. Sending your target to the robot."
        assert r.rec.sent[-1] == "Search mission completed."
        assert r.link.received[0]["target_description"] == "blue toolbox"
    finally:
        r.close()


def test_webhook_route_disabled_in_websocket_mode(tmp_path):
    r = Rig(tmp_path, PHOTON_INBOUND="websocket", PHOTON_API_KEY="pho_sk_" + "a" * 40)
    try:  # never started: no connection to Photon is attempted
        assert r.webhook("/start") == 404
        health = r.req("GET", "/healthz")[1]
        assert health["inbound"] == "websocket" and health["photon_connected"] is False
    finally:
        r.close()


def test_operator_api_disabled_without_token(tmp_path):
    r = Rig(tmp_path, OPERATOR_API_TOKEN=None)
    try:
        assert r.req("POST", "/api/missions", {"target_description": "red cup"}, OP)[0] == 403
    finally:
        r.close()


def test_operator_api(tmp_path):  # M8-M10, M12, D6
    r = Rig(tmp_path, script="stall")
    try:
        assert r.req("POST", "/api/missions", {"target_description": "red cup"})[0] == 401
        assert r.req("POST", "/api/missions", {"target_description": "red cup"},
                     {"Authorization": "Bearer " + "x" * 40})[0] == 401
        assert r.req("GET", f"/api/missions/{MID}")[0] == 401
        assert r.req("POST", "/api/missions", {"target_description": "; reboot"}, OP)[0] == 422
        assert r.req("POST", "/api/missions", {}, OP)[0] == 422
        assert r.req("POST", "/api/missions", raw=b"not json", headers=OP)[0] == 422
        h = dict(OP, **{"Idempotency-Key": "op-1"})
        c1, b1 = r.req("POST", "/api/missions", {"target_description": "red cup"}, h)
        c2, b2 = r.req("POST", "/api/missions", {"target_description": "red cup"}, h)
        assert (c1, c2) == (201, 200)
        mid = b1["mission_id"]
        assert b2["mission_id"] == mid
        assert r.link.received[-1]["mission_id"] == mid
        code, body = r.req("POST", "/api/missions", {"target_description": "mug"}, OP)
        assert code == 409 and body["active_mission_id"] == mid
        assert r.req("GET", "/api/missions/not-a-uuid", headers=OP)[0] == 422
        assert r.req("GET", f"/api/missions/{MID}", headers=OP)[0] == 404
        code, body = r.req("POST", f"/api/missions/{mid}/cancel", headers=OP)
        assert code == 202 and body["cancel_requested"] is True and body["state"] == "PENDING"
        assert r.link.received[-1] == {"type": "MISSION_CANCEL", "mission_id": mid}
        r.link.poll()  # ACK
        time.sleep(0.6)
        r.link.poll()  # CANCELLED after the fake robot's cancel delay
        assert r.req("GET", f"/api/missions/{mid}", headers=OP)[1]["state"] == "CANCELLED"
        assert r.req("POST", f"/api/missions/{mid}/cancel", headers=OP)[0] == 409
        assert r.req("POST", f"/api/missions/{MID}/cancel", headers=OP)[0] == 404
    finally:
        r.close()


def test_operator_api_robot_offline(tmp_path):
    r = Rig(tmp_path, online=False)
    try:
        assert r.req("POST", "/api/missions", {"target_description": "red cup"}, OP)[0] == 503
    finally:
        r.close()


def test_unknown_routes(rig):
    assert rig.req("GET", "/nope")[0] == 404
    assert rig.req("POST", "/nope", {}, OP)[0] == 404


def test_operator_token_compared_in_constant_time():
    src = (pathlib.Path(__file__).resolve().parents[2] / "locatebot/remote/server.py").read_text()
    assert "hmac.compare_digest" in src


def test_no_web_framework_imported():
    import sys
    import locatebot.remote.server  # noqa: F401
    for mod in ("fastapi", "uvicorn", "httpx", "starlette"):
        assert mod not in sys.modules or "locatebot" not in str(sys.modules[mod])
    src = "".join(p.read_text() for p in (pathlib.Path(__file__).resolve().parents[2] / "locatebot/remote").glob("*.py"))
    for mod in ("fastapi", "uvicorn", "httpx", "starlette"):
        assert f"import {mod}" not in src and f"from {mod}" not in src
