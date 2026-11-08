"""Jetson side of the link: dial out to the backend, authenticate, feed frames to the JetsonAgent.

Needs the `robot-link` extra (`websockets`). The agent keeps ticking while disconnected, so mission
expiry and the sink's link-loss safety behaviour do not depend on the network.

    python tools/fake_jetson.py --url wss://<backend>/ws/robot --script happy
"""
from __future__ import annotations

import asyncio
import logging
import random
from typing import Callable, Optional

from .agent import JetsonAgent
from .protocol import MAX_FRAME_BYTES

log = logging.getLogger(__name__)


def _connect(url: str, headers: dict, ping_s: float):
    try:  # websockets >= 13
        from websockets.asyncio.client import connect
        return connect(url, additional_headers=headers, ping_interval=ping_s, ping_timeout=ping_s,
                       max_size=MAX_FRAME_BYTES, open_timeout=10)
    except ImportError:  # older releases
        import websockets
        return websockets.connect(url, extra_headers=headers, ping_interval=ping_s, ping_timeout=ping_s,
                                  max_size=MAX_FRAME_BYTES, open_timeout=10)


class _Sender:
    """The agent's `send`: thread-safe, raises when there is no live socket."""

    def __init__(self):
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.queue: Optional[asyncio.Queue] = None

    def __call__(self, text: str) -> None:
        if self.queue is None:
            raise ConnectionError("not connected")
        self.loop.call_soon_threadsafe(self.queue.put_nowait, text)


async def run_client(url: str, device_id: str, token: str, make_agent: Callable[[Callable[[str], None]], JetsonAgent],
                     poll: Callable[[], None] = lambda: None, stop: Optional[asyncio.Event] = None,
                     ping_s: float = 10.0, max_backoff_s: float = 30.0, drop_after_s: Optional[float] = None,
                     ignore_frames: bool = False) -> None:
    """Run until `stop` is set. `poll` is called every 0.2 s (e.g. ScriptedSink.poll).
    `drop_after_s` / `ignore_frames` exist only for fake_jetson.py failure drills."""
    stop = stop or asyncio.Event()
    sender = _Sender()
    agent = make_agent(sender)
    headers = {"Authorization": f"Bearer {token}", "X-Device-Id": device_id}

    async def ticker():
        while not stop.is_set():
            try:
                poll()
                agent.tick()
            except Exception:
                log.exception("tick failed")
            await asyncio.sleep(0.2)

    tick_task = asyncio.create_task(ticker())
    backoff, dropped_once = 1.0, False
    try:
        while not stop.is_set():
            connected = False
            try:
                async with _connect(url, headers, ping_s) as ws:
                    connected, backoff = True, 1.0
                    sender.loop, sender.queue = asyncio.get_running_loop(), asyncio.Queue()
                    log.info("connected to %s", url)
                    agent.on_connected()

                    async def reader():
                        async for frame in ws:
                            if ignore_frames:
                                log.info("ignoring frame (silent drill): %s", frame)
                            else:
                                agent.on_frame(frame)

                    async def writer():
                        while True:
                            await ws.send(await sender.queue.get())

                    tasks = [asyncio.create_task(reader()), asyncio.create_task(writer()),
                             asyncio.create_task(stop.wait())]
                    if drop_after_s is not None and not dropped_once:
                        dropped_once = True
                        tasks.append(asyncio.create_task(asyncio.sleep(drop_after_s)))
                    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                    for t in pending:
                        t.cancel()
                    for t in done:
                        if t.exception():
                            log.warning("link error: %s", t.exception())
            except Exception as e:  # refused, 403 on bad token, DNS, TLS, timeouts...
                log.warning("connection failed: %s", e)
            finally:
                sender.queue = None
                if connected:
                    agent.on_disconnected()
                    log.warning("disconnected from %s", url)
            if stop.is_set():
                break
            delay = backoff * (0.5 + random.random() / 2)
            log.info("reconnecting in %.1fs", delay)
            try:
                await asyncio.wait_for(stop.wait(), timeout=delay)
            except asyncio.TimeoutError:
                pass
            backoff = min(max_backoff_s, backoff * 2)
    finally:
        stop.set()
        tick_task.cancel()
