"""Level L3 automated: real uvicorn server + real websockets client + JetsonAgent + ScriptedSink on localhost.
Skipped unless fastapi, uvicorn and websockets are installed."""
import asyncio
import base64
import json
import socket
import threading
import time

import pytest

pytest.importorskip("fastapi")
uvicorn = pytest.importorskip("uvicorn")
pytest.importorskip("websockets")

from locatebot.backend.app import create_app  # noqa: E402
from locatebot.backend.config import Settings  # noqa: E402
from locatebot.backend.messaging.base import InboundMessage, Conversation, SendResult  # noqa: E402
from locatebot.dispatch.agent import JetsonAgent  # noqa: E402
from locatebot.dispatch.sink import SCRIPTS, ScriptedSink  # noqa: E402
from locatebot.dispatch.ws_client import run_client  # noqa: E402

TOKEN = "j" * 40


class Recorder:
    name = "photon"

    def __init__(self):
        self.sent = []

    def send_text(self, conversation, text, idempotency_key):
        self.sent.append(text)
        return SendResult(True)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_scripted_jetson_over_real_websocket(tmp_path):
    settings = Settings.from_env({
        "PHOTON_MODE": "console", "PHOTON_PROJECT_ID": "pho_prj_01jabcdefghjkmnpqrstvwxyz1",
        "PHOTON_WEBHOOK_SECRET": "whsec_" + base64.b64encode(b"k" * 32).decode(), "JETSON_DEVICE_ID": "jetson-01",
        "JETSON_AUTH_TOKEN": TOKEN, "OPERATOR_API_TOKEN": "o" * 40, "BACKEND_PUBLIC_URL": "https://example.test",
        "DISPATCH_DB_PATH": str(tmp_path / "t.db")})
    rec = Recorder()
    app = create_app(settings, provider=rec)
    port = free_port()
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning",
                                           ws_ping_interval=1, ws_ping_timeout=1))
    threading.Thread(target=server.run, daemon=True).start()
    while not server.started:
        time.sleep(0.05)

    sink = ScriptedSink(SCRIPTS["happy"])
    stop_holder = {}

    def jetson():
        async def main():
            stop_holder["stop"] = stop = asyncio.Event()
            stop_holder["loop"] = asyncio.get_running_loop()
            await run_client(f"ws://127.0.0.1:{port}/ws/robot", "jetson-01", TOKEN,
                             lambda send: JetsonAgent(sink, send=send), poll=sink.poll, stop=stop, ping_s=1)
        asyncio.run(main())

    t = threading.Thread(target=jetson, daemon=True)
    t.start()
    try:
        deadline = time.time() + 5
        while not app.state.link.connected and time.time() < deadline:
            time.sleep(0.05)
        assert app.state.link.connected
        app.state.service.handle_inbound(InboundMessage(
            "pho_evt_live", Conversation("photon", "pho_spc_01jaaaaaaaaaaaaaaaaaaaaaaa", "pho_usr_bot"),
            "pho_usr_01jaaaaaaaaaaaaaaaaaaaaaaa", "a", "/find blue toolbox"))
        deadline = time.time() + 15
        while "Search mission completed." not in rec.sent and time.time() < deadline:
            time.sleep(0.1)
        assert rec.sent[0] == "Request received. Sending your target to the robot."
        assert rec.sent[-1] == "Search mission completed."
    finally:
        if "loop" in stop_holder:
            stop_holder["loop"].call_soon_threadsafe(stop_holder["stop"].set)
        t.join(timeout=5)
        server.should_exit = True
