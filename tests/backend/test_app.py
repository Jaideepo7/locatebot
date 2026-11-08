"""HTTP/WebSocket layer tests (level L2). Skipped unless the `backend` extra is installed."""
import base64
import json
import pathlib
import time

import pytest

pytest.importorskip("fastapi")
from fastapi.testclient import TestClient  # noqa: E402
from starlette.websockets import WebSocketDisconnect  # noqa: E402

from locatebot.backend.app import create_app  # noqa: E402
from locatebot.backend.config import Settings  # noqa: E402
from locatebot.backend.messaging.base import SendResult  # noqa: E402
from locatebot.backend.messaging.photon import sign_payload  # noqa: E402

FIXTURE = json.loads((pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "photon_message_received.json").read_text())
SECRET = "whsec_" + base64.b64encode(b"app-test-secret-app-test-secret!").decode()
JTOKEN, OTOKEN = "j" * 40, "o" * 40
ROBOT_HEADERS = {"Authorization": f"Bearer {JTOKEN}", "X-Device-Id": "jetson-01"}
OP = {"Authorization": f"Bearer {OTOKEN}"}


class Recorder:
    name = "photon"

    def __init__(self):
        self.sent = []

    def send_text(self, conversation, text, idempotency_key):
        self.sent.append(text)
        return SendResult(True)


@pytest.fixture
def env(tmp_path):
    settings = Settings.from_env({
        "PHOTON_MODE": "console", "PHOTON_PROJECT_ID": FIXTURE["data"]["projectId"], "PHOTON_WEBHOOK_SECRET": SECRET,
        "JETSON_DEVICE_ID": "jetson-01", "JETSON_AUTH_TOKEN": JTOKEN, "OPERATOR_API_TOKEN": OTOKEN,
        "BACKEND_PUBLIC_URL": "https://example.test", "DISPATCH_DB_PATH": str(tmp_path / "t.db")})
    rec = Recorder()
    app = create_app(settings, provider=rec, background=False)
    with TestClient(app) as client:
        yield client, app, rec


def post_webhook(client, text, event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz0", sign=True):
    ev = json.loads(json.dumps(FIXTURE))
    ev["id"] = event_id
    ev["data"]["object"]["content"]["text"] = text
    body = json.dumps(ev).encode()
    ts = int(time.time())
    sig = sign_payload(SECRET, event_id, ts, body) if sign else "v1,AAAA"
    return client.post("/api/photon/webhook", content=body, headers={
        "content-type": "application/json", "webhook-id": event_id, "webhook-timestamp": str(ts),
        "webhook-signature": sig})


def wait_for(fn, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.02)
    raise AssertionError("condition not met")


def test_webhook_signature_enforced(env):
    client, app, rec = env
    assert post_webhook(client, "/start", sign=False).status_code == 400
    assert post_webhook(client, "/start").status_code == 204
    assert post_webhook(client, "/start").status_code == 204  # duplicate: still 2xx
    app.state.outbox.run_once()
    assert len(rec.sent) == 1 and rec.sent[0].startswith("Hello!")


def test_find_while_robot_offline(env):
    client, app, rec = env
    post_webhook(client, "/find blue toolbox")
    app.state.outbox.run_once()
    assert rec.sent == ["The robot is currently unavailable. Please try again later."]


@pytest.mark.parametrize("headers,url", [({}, "/ws/robot"),
                                         ({"Authorization": "Bearer wrong", "X-Device-Id": "jetson-01"}, "/ws/robot"),
                                         ({"Authorization": f"Bearer {JTOKEN}", "X-Device-Id": "other"}, "/ws/robot"),
                                         (ROBOT_HEADERS, f"/ws/robot?token={JTOKEN}")])
def test_robot_auth_rejected(env, headers, url):  # L1, L2
    client, app, _ = env
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect(url, headers=headers) as ws:
            ws.receive_text()
    assert not app.state.link.connected


def test_full_flow_over_websocket(env):  # E1 through the real routes
    client, app, rec = env
    with client.websocket_connect("/ws/robot", headers=ROBOT_HEADERS) as ws:
        wait_for(lambda: app.state.link.connected)
        assert client.get("/healthz").json()["robot_connected"] is True
        assert post_webhook(client, "/find blue toolbox").status_code == 204
        req = ws.receive_json()
        assert req["type"] == "MISSION_REQUEST" and req["target_description"] == "blue toolbox"
        mid = req["mission_id"]
        ws.send_json({"type": "MISSION_ACK", "mission_id": mid, "status": "ACCEPTED"})
        for s in ["SEARCHING", "TARGET_DETECTED", "APPROACHING", "COMPLETED"]:
            ws.send_json({"type": "MISSION_STATUS", "mission_id": mid, "status": s})
        wait_for(lambda: client.get(f"/api/missions/{mid}", headers=OP).json()["state"] == "COMPLETED")
    app.state.outbox.run_once()
    assert rec.sent[0] == "Request received. Sending your target to the robot."
    assert rec.sent[-1] == "Search mission completed."


def test_disconnect_tells_customer(env):  # L6
    client, app, rec = env
    with client.websocket_connect("/ws/robot", headers=ROBOT_HEADERS) as ws:
        wait_for(lambda: app.state.link.connected)
        post_webhook(client, "/find blue toolbox")
        mid = ws.receive_json()["mission_id"]
        ws.send_json({"type": "MISSION_ACK", "mission_id": mid, "status": "ACCEPTED"})
        wait_for(lambda: client.get(f"/api/missions/{mid}", headers=OP).json()["state"] == "ACCEPTED")
    wait_for(lambda: not app.state.link.connected)
    app.state.outbox.run_once()
    assert rec.sent[-1] == "Lost contact with the robot. It will stop on its own for safety."


def test_binary_frame_closes_connection(env):  # L10
    client, app, _ = env
    with client.websocket_connect("/ws/robot", headers=ROBOT_HEADERS) as ws:
        ws.send_bytes(b"\x00\x01")
        with pytest.raises(WebSocketDisconnect):
            ws.receive_text()


def test_operator_api(env):  # M8-M10, M12, D6
    client, app, _ = env
    assert client.post("/api/missions", json={"target_description": "red cup"}).status_code == 401
    assert client.get("/api/missions/3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70").status_code == 401
    assert client.post("/api/missions", json={"target_description": "red cup"}, headers=OP).status_code == 503
    with client.websocket_connect("/ws/robot", headers=ROBOT_HEADERS) as ws:
        wait_for(lambda: app.state.link.connected)
        assert client.post("/api/missions", json={"target_description": "; reboot"}, headers=OP).status_code == 422
        assert client.post("/api/missions", json={}, headers=OP).status_code == 422
        h = dict(OP, **{"Idempotency-Key": "op-1"})
        r1 = client.post("/api/missions", json={"target_description": "red cup"}, headers=h)
        r2 = client.post("/api/missions", json={"target_description": "red cup"}, headers=h)
        assert (r1.status_code, r2.status_code) == (201, 200)
        mid = r1.json()["mission_id"]
        assert r2.json()["mission_id"] == mid
        assert ws.receive_json()["mission_id"] == mid
        assert client.post("/api/missions", json={"target_description": "mug"}, headers=OP).status_code == 409
        assert client.get("/api/missions/not-a-uuid", headers=OP).status_code == 422
        assert client.get("/api/missions/3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70", headers=OP).status_code == 404
        r = client.post(f"/api/missions/{mid}/cancel", headers=OP)
        assert r.status_code == 202 and r.json()["cancel_requested"] is True and r.json()["state"] == "PENDING"
        assert ws.receive_json() == {"type": "MISSION_CANCEL", "mission_id": mid}
        ws.send_json({"type": "MISSION_STATUS", "mission_id": mid, "status": "CANCELLED"})
        wait_for(lambda: client.get(f"/api/missions/{mid}", headers=OP).json()["state"] == "CANCELLED")
        assert client.post(f"/api/missions/{mid}/cancel", headers=OP).status_code == 409
