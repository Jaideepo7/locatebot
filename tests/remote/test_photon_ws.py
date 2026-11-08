"""PhotonWebSocketInbound against a fake Photon Socket.IO-over-WebSocket server on 127.0.0.1.

The fake speaks the documented protocol (Engine.IO v4 open/ping/pong, Socket.IO CONNECT,
`subscribe` with an ack, `delivery(event, {seq, replyExpected})`, `delivery_error`), checks the
endpoint path and the Authorization header, and answers HTTP 401 to a wrong key. No real network.
"""
import asyncio
import json
import logging
import pathlib
import threading
import time
from collections import deque

import pytest

pytest.importorskip("websockets")
from websockets.asyncio.server import serve  # noqa: E402
from websockets.datastructures import Headers  # noqa: E402
from websockets.http11 import Response  # noqa: E402

from locatebot.remote.config import Settings  # noqa: E402
from locatebot.remote.photon_ws import PhotonWebSocketInbound  # noqa: E402
from locatebot.remote.robot import FakeRobotLink  # noqa: E402
from locatebot.remote.service import RemoteService  # noqa: E402
from locatebot.remote.store import StorageUnavailable, Store  # noqa: E402

FIXTURE = json.loads((pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "photon_message_received.json").read_text())
PROJECT = FIXTURE["data"]["projectId"]
KEY = "pho_sk_" + "s3cr3tk3y" * 4
QUIET = logging.getLogger("test.fake_photon_server")  # the fake server's own logs (it sees the key)
QUIET.disabled = True
OPEN = '0{"sid":"eio1","upgrades":[],"pingInterval":25000,"pingTimeout":20000,"maxPayload":1000000}'


async def drain(ws):
    """Keep a fake-server handler alive until the client goes away (closes promptly on shutdown)."""
    try:
        async for _ in ws:
            pass
    except Exception:
        pass


def event(text="/find blue toolbox", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz0"):
    ev = json.loads(json.dumps(FIXTURE))
    ev["id"] = event_id
    ev["data"]["object"]["content"]["text"] = text
    return ev


def delivery(seq, **kw):
    return "42" + json.dumps(["delivery", event(**kw), {"seq": seq, "replyExpected": False}])


class FakePhoton:
    """One plan (async fn(ws)) per connection, run after the subscribe ack."""

    def __init__(self, key=KEY, sub_ok=True):
        self.key, self.sub_ok = key, sub_ok
        self.plans = deque()
        self.connections = 0
        self.rejected = 0
        self.subscribes = []
        self.paths = []
        self.pongs = 0

    def process_request(self, connection, request):
        self.paths.append(request.path)
        ok_path = request.path == f"/v1/projects/{PROJECT}/events/ws?EIO=4&transport=websocket"
        if not ok_path or request.headers.get("Authorization") != f"Bearer {self.key}":
            self.rejected += 1
            return Response(401, "Unauthorized", Headers([("Content-Length", "0")]), b"")
        return None

    async def handler(self, ws):
        self.connections += 1
        await ws.send(OPEN)
        assert await ws.recv() == "40"
        await ws.send('40{"sid":"sio1"}')
        sub = await ws.recv()
        assert sub.startswith("420[")
        name, args = json.loads(sub[3:])
        assert name == "subscribe"
        self.subscribes.append(args)
        if not self.sub_ok:
            await ws.send('430[{"ok":false,"error":{"code":"frame_invalid","message":"bad"}}]')
            await drain(ws)
            return
        await ws.send('430[{"ok":true,"apiVersion":"2026-07-01","connectionId":"c%d","cursor":%d,'
                      '"limits":{"maxFrameBytes":65536,"maxReplyBodyBytes":4096}}]'
                      % (self.connections, args["startSeq"]))
        plan = self.plans.popleft() if self.plans else None
        if plan is not None:
            await plan(ws)
        try:
            while True:  # keep the connection open; count pongs
                msg = await ws.recv()
                if msg == "3":
                    self.pongs += 1
        except Exception:
            pass


class Sleeps:
    def __init__(self, block=False):
        self.delays, self.block = [], block
        self.forever = asyncio.Event()

    async def __call__(self, s):
        self.delays.append(s)
        if self.block:
            await self.forever.wait()
        await asyncio.sleep(0)


def recorder(store):
    got = []

    def on_message(msg):  # the MissionService contract: record the event id with the work
        with store.transaction():
            if store.insert_event(msg.event_id, 0.0):
                got.append(msg)
    return got, on_message


async def until(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return
        await asyncio.sleep(0.01)
    raise AssertionError("condition not met")


async def scenario(fake, body, *, sleeps=None, store=None, on_message=None, **kw):
    store = store or Store(":memory:")
    got, rec = recorder(store)
    async with serve(fake.handler, "127.0.0.1", 0, process_request=fake.process_request, logger=QUIET) as server:
        port = server.sockets[0].getsockname()[1]
        inbound = PhotonWebSocketInbound(KEY, PROJECT, on_message or rec, store, origin=f"http://127.0.0.1:{port}",
                                         sleep=sleeps or Sleeps(), rng=lambda: 0.5, **kw)
        task = asyncio.create_task(inbound.run())
        try:
            await body(inbound, got, store)
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
    return inbound, got


def test_happy_path_event_delivered_once_and_cursor_saved():
    fake = FakePhoton()

    async def plan(ws):
        await ws.send(delivery(7))

    fake.plans.append(plan)

    async def body(inbound, got, store):
        await until(lambda: len(got) == 1)
        st = inbound.status()
        assert st["connected"] and st["cursor"] == 7 and st["events"] == 1
        assert st["connection_id"] == "c1" and st["api_version"] == "2026-07-01"
        assert got[0].text == "/find blue toolbox"
        assert got[0].event_id == "pho_evt_01jabcdefghjkmnpqrstvwxyz0"
        assert store.get_cursor(f"photon:{PROJECT}") == 7

    asyncio.run(scenario(fake, body))
    assert fake.subscribes == [{"startSeq": 0}]
    assert fake.paths == [f"/v1/projects/{PROJECT}/events/ws?EIO=4&transport=websocket"]


def test_duplicate_event_id_and_replayed_seq_delivered_once():
    fake = FakePhoton()

    async def plan(ws):
        await ws.send(delivery(1))
        await ws.send(delivery(1))  # same cursor replayed
        await ws.send(delivery(2))  # same event id redelivered with a new cursor
        await ws.send(delivery(3, text="/status", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz9"))

    fake.plans.append(plan)

    async def body(inbound, got, store):
        await until(lambda: len(got) == 2)
        await until(lambda: inbound.status()["cursor"] == 3)
        assert [m.text for m in got] == ["/find blue toolbox", "/status"]
        assert inbound.status()["duplicates"] == 2

    asyncio.run(scenario(fake, body))


def test_malformed_frames_are_counted_and_connection_survives():
    fake = FakePhoton()

    async def plan(ws):
        for bad in ["4", "42not json", '42{"a":1}', "42[]", '42["delivery"]', '42["delivery",{"id":"x"},{"seq":-1}]',
                    "9zzz", b"\x00\x01", '42["delivery",{"type":"message.received"},{"seq":4,"replyExpected":false}]',
                    '42["delivery",{"id":"pho_evt_1","type":"message.received","source":"telegram.photon",'
                    '"data":{"projectId":"%s","object":{}}},{"seq":5,"replyExpected":false}]' % PROJECT]:
            await ws.send(bad)
        await ws.send(delivery(6))

    fake.plans.append(plan)

    async def body(inbound, got, store):
        await until(lambda: len(got) == 1)
        st = inbound.status()
        assert st["dropped_frames"] == 10
        assert st["reconnects"] == 0 and st["connected"] and st["cursor"] == 6

    asyncio.run(scenario(fake, body))
    assert fake.connections == 1


def test_server_drop_reconnects_with_backoff_and_resumes_from_cursor():
    fake = FakePhoton()

    async def first(ws):
        await ws.send(delivery(10))
        await asyncio.sleep(0.05)
        await ws.close()

    async def second_fails(ws):
        await ws.close()

    async def third(ws):
        await ws.send(delivery(11, text="/help", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz8"))

    fake.plans.extend([first, second_fails, third])
    sleeps = Sleeps()

    async def body(inbound, got, store):
        await until(lambda: len(got) == 2)
        assert inbound.status()["reconnects"] == 2

    asyncio.run(scenario(fake, body, sleeps=sleeps))
    assert [s["startSeq"] for s in fake.subscribes] == [0, 10, 10]
    # backoff resets after a successful subscription: 1 s (jitter 0.5 -> factor 1.0), then 1 s again
    assert sleeps.delays == [1.0, 1.0]


def test_backoff_grows_to_the_cap_when_photon_is_unreachable():
    async def refuse(url, headers):
        raise ConnectionRefusedError("refused")

    store = Store(":memory:")
    sleeps = Sleeps()
    inbound = PhotonWebSocketInbound(KEY, PROJECT, lambda m: None, store, connect=refuse, sleep=sleeps,
                                     rng=lambda: 0.5)

    async def go():
        task = asyncio.create_task(inbound.run())
        await until(lambda: len(sleeps.delays) >= 8)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(go())
    assert sleeps.delays[:8] == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0, 30.0]
    st = inbound.status()
    assert not st["connected"] and "refused" in st["last_error"]


def test_jitter_stays_within_bounds():
    for r, expect in ((0.0, 0.8), (1.0, 1.2)):
        sleeps = Sleeps()

        async def refuse(url, headers):
            raise OSError("down")

        inbound = PhotonWebSocketInbound(KEY, PROJECT, lambda m: None, Store(":memory:"), connect=refuse,
                                         sleep=sleeps, rng=lambda r=r: r)

        async def go():
            task = asyncio.create_task(inbound.run())
            await until(lambda: len(sleeps.delays) >= 1)
            task.cancel()

        asyncio.run(go())
        assert sleeps.delays[0] == pytest.approx(expect)


def test_auth_failure_no_tight_loop_and_status_shows_error(caplog):
    caplog.set_level(logging.DEBUG)
    fake = FakePhoton(key="pho_sk_someoneelse")
    sleeps = Sleeps(block=True)

    async def body(inbound, got, store):
        await until(lambda: len(sleeps.delays) == 1)
        await asyncio.sleep(0.2)
        st = inbound.status()
        assert not st["connected"] and "authentication failed" in st["last_error"]

    inbound, _ = asyncio.run(scenario(fake, body, sleeps=sleeps))
    assert fake.rejected == 1 and fake.connections == 0
    assert sleeps.delays == [300.0]
    assert "authentication failed" in caplog.text
    assert KEY not in caplog.text and KEY not in repr(inbound) and KEY not in json.dumps(inbound.status())


def test_socketio_connect_error_is_treated_as_auth_failure():
    async def handler(ws):
        await ws.send(OPEN)
        await ws.recv()
        await ws.send('44{"message":"unauthorized"}')
        await drain(ws)

    sleeps = Sleeps(block=True)

    async def go():
        async with serve(handler, "127.0.0.1", 0, logger=QUIET) as server:
            port = server.sockets[0].getsockname()[1]
            inbound = PhotonWebSocketInbound(KEY, PROJECT, lambda m: None, Store(":memory:"),
                                             origin=f"http://127.0.0.1:{port}", sleep=sleeps)
            task = asyncio.create_task(inbound.run())
            await until(lambda: len(sleeps.delays) == 1)
            task.cancel()
            return inbound

    inbound = asyncio.run(go())
    assert sleeps.delays == [300.0]
    assert "connect_error unauthorized" in inbound.status()["last_error"]


def test_subscribe_rejected_reconnects_with_backoff():
    fake = FakePhoton(sub_ok=False)
    sleeps = Sleeps(block=True)

    async def body(inbound, got, store):
        await until(lambda: len(sleeps.delays) == 1)
        assert "subscribe rejected: frame_invalid" in inbound.status()["last_error"]

    asyncio.run(scenario(fake, body, sleeps=sleeps))
    assert sleeps.delays == [1.0]


def test_subscribe_timeout():
    async def handler(ws):
        await ws.send(OPEN)
        await ws.recv()
        await ws.send('40{"sid":"x"}')
        await ws.recv()  # subscribe, never acknowledged
        await drain(ws)

    sleeps = Sleeps(block=True)

    async def go():
        async with serve(handler, "127.0.0.1", 0, logger=QUIET) as server:
            port = server.sockets[0].getsockname()[1]
            inbound = PhotonWebSocketInbound(KEY, PROJECT, lambda m: None, Store(":memory:"),
                                             origin=f"http://127.0.0.1:{port}", sleep=sleeps, subscribe_timeout_s=0.2)
            task = asyncio.create_task(inbound.run())
            await until(lambda: len(sleeps.delays) == 1)
            task.cancel()
            return inbound

    assert "subscribe timed out" in asyncio.run(go()).status()["last_error"]


def test_heartbeat_ping_answered_and_silence_detected():
    async def handler(ws):
        await ws.send('0{"sid":"e","upgrades":[],"pingInterval":100,"pingTimeout":100,"maxPayload":1000}')
        await ws.recv()
        await ws.send('40{"sid":"x"}')
        await ws.recv()
        await ws.send('430[{"ok":true}]')
        await ws.send("2")
        assert await ws.recv() == "3"
        pongs.append(1)
        await drain(ws)

    pongs = []
    sleeps = Sleeps(block=True)

    async def go():
        async with serve(handler, "127.0.0.1", 0, logger=QUIET) as server:
            port = server.sockets[0].getsockname()[1]
            inbound = PhotonWebSocketInbound(KEY, PROJECT, lambda m: None, Store(":memory:"),
                                             origin=f"http://127.0.0.1:{port}", sleep=sleeps)
            task = asyncio.create_task(inbound.run())
            await until(lambda: len(sleeps.delays) == 1, timeout=3)
            task.cancel()
            return inbound

    inbound = asyncio.run(go())
    assert pongs == [1]
    assert inbound.status()["last_error"] == "heartbeat timeout"


def test_fatal_delivery_error_uses_retry_after():
    fake = FakePhoton()

    async def plan(ws):
        await ws.send('42["delivery_error",{"code":"slow_consumer","message":"too slow","fatal":true,'
                      '"retryable":true,"retryAfterMs":5000}]')

    fake.plans.append(plan)
    sleeps = Sleeps(block=True)

    async def body(inbound, got, store):
        await until(lambda: len(sleeps.delays) == 1)

    inbound, _ = asyncio.run(scenario(fake, body, sleeps=sleeps))
    assert sleeps.delays == [5.0]
    assert inbound.status()["last_error"] == "delivery_error slow_consumer"


def test_handler_failure_stops_stream_and_resumes_from_last_saved_cursor():
    fake = FakePhoton()
    calls = []

    def flaky(msg):
        calls.append(msg.text)
        if len(calls) == 2:
            raise StorageUnavailable("disk full")

    async def plan1(ws):
        await ws.send(delivery(1, text="/help", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz1"))
        await ws.send(delivery(2, text="/status", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz2"))
        await ws.send(delivery(3, text="/start", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz3"))

    async def plan2(ws):  # Photon replays from the cursor we resubscribe with
        await ws.send(delivery(2, text="/status", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz2"))
        await ws.send(delivery(3, text="/start", event_id="pho_evt_01jabcdefghjkmnpqrstvwxyz3"))

    fake.plans.extend([plan1, plan2])

    async def body(inbound, got, store):
        await until(lambda: inbound.status()["cursor"] == 3)

    asyncio.run(scenario(fake, body, on_message=flaky))
    assert calls == ["/help", "/status", "/status", "/start"]  # /start never processed before /status
    assert [s["startSeq"] for s in fake.subscribes] == [0, 1]


def test_start_stop_thread_promptly_and_key_never_logged(caplog):
    caplog.set_level(logging.DEBUG)
    fake = FakePhoton()
    store = Store(":memory:")
    got, rec = recorder(store)

    async def plan(ws):
        await ws.send(delivery(1))

    fake.plans.append(plan)

    async def go():
        async with serve(fake.handler, "127.0.0.1", 0, process_request=fake.process_request, logger=QUIET) as server:
            port = server.sockets[0].getsockname()[1]
            inbound = PhotonWebSocketInbound(KEY, PROJECT, rec, store, origin=f"http://127.0.0.1:{port}")
            inbound.start()
            await until(lambda: len(got) == 1)
            assert any(t.name == "photon-ws" for t in threading.enumerate())
            t0 = time.monotonic()
            await asyncio.to_thread(inbound.stop)
            elapsed = time.monotonic() - t0
            assert not any(t.name == "photon-ws" for t in threading.enumerate())
            return inbound, elapsed

    inbound, elapsed = asyncio.run(go())
    assert elapsed < 1.0
    assert not inbound.status()["connected"]
    assert KEY not in caplog.text and KEY not in repr(inbound)


def test_remote_service_end_to_end_over_websocket(tmp_path):
    """Photon WS delivery -> MissionService -> FakeRobotLink, and /healthz-style health."""
    fake = FakePhoton()

    async def plan(ws):
        await ws.send(delivery(1))

    fake.plans.append(plan)

    async def go():
        async with serve(fake.handler, "127.0.0.1", 0, process_request=fake.process_request, logger=QUIET) as server:
            port = server.sockets[0].getsockname()[1]
            settings = Settings.from_env({"PHOTON_MODE": "console", "PHOTON_API_KEY": KEY, "PHOTON_PROJECT_ID": PROJECT,
                                          "DISPATCH_DB_PATH": str(tmp_path / "d.db")})

            class Rec:
                name = "photon"
                sent = []

                def send_text(self, conversation, text, key):
                    from locatebot.remote.messaging import SendResult
                    self.sent.append(text)
                    return SendResult(True)

            link = FakeRobotLink("stall")
            svc = RemoteService(settings, provider=Rec(), link=link, tick_s=0.05)
            svc.inbound = PhotonWebSocketInbound(KEY, PROJECT, svc.missions.handle_inbound, svc.store,
                                                 origin=f"http://127.0.0.1:{port}")
            assert svc.health()["photon_connected"] is False
            svc.start()
            try:
                await until(lambda: Rec.sent[:2] == ["Request received. Sending your target to the robot.",
                                                     "The robot has started searching for: blue toolbox."])
                h = svc.health()
                assert h["photon_connected"] is True and h["robot_connected"] is True
                assert link.received[0]["target_description"] == "blue toolbox"
            finally:
                await asyncio.to_thread(svc.stop)
            assert svc.health()["photon_connected"] is False

    asyncio.run(go())
