"""Tests for locatebot.vision.service with a fake engine (no model, no threads left behind).

These prove the worker's scheduling and bookkeeping, not real model latency.
"""
from __future__ import annotations

import math
import struct
import threading
import time

import pytest

from locatebot.vision.engine import EngineError
from locatebot.vision.postprocess import Detection
from locatebot.vision.service import VisionService


def jpeg(w=640, h=480, tag=0) -> bytes:
    sof = b"\xff\xc0" + struct.pack(">HBHHB", 11, 8, h, w, 1) + b"\x01\x11\x00"
    return b"\xff\xd8" + sof + b"\xff\xd9" + bytes([tag])  # tag makes frames distinguishable


class FakeEngine:
    """Records calls; optionally blocks each call until released; returns a box per call."""

    def __init__(self, block=False, boxes=None, fail_items=()):
        self.calls: list[tuple[bytes, str, str]] = []
        self.block = block
        self.release = threading.Semaphore(0)
        self.entered = threading.Semaphore(0)
        self.boxes = boxes if boxes is not None else [(310.0, 0.0, 330.0, 10.0)]
        self.fail_items = fail_items

    def locate(self, image, prompt, mode="hybrid"):
        self.calls.append((image, prompt, mode))
        self.entered.release()
        if self.block:
            assert self.release.acquire(timeout=5), "test never released the fake engine"
        if any(f"description: {it}." in prompt for it in self.fail_items):
            raise EngineError("locate failed: boom")
        return [Detection("x", b) for b in self.boxes]

    def items_called(self):
        return [p.rsplit(": ", 1)[1].rstrip(".") for _, p, _ in self.calls]


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.005)
    return False


@pytest.fixture
def make_service():
    services = []

    def make(engine, **kw):
        results = []
        kw.setdefault("heartbeat_s", 0.05)
        svc = VisionService(engine, on_result=results.append, hfov_rad=math.radians(90), **kw)
        svc.results = results
        services.append(svc)
        svc.start()
        return svc

    yield make
    for s in services:
        if hasattr(s._engine, "release"):
            for _ in range(10):
                s._engine.release.release()
        s.stop(timeout=2)


def test_rotates_items_on_one_frame_then_idles(make_service):
    eng = FakeEngine()
    svc = make_service(eng)
    svc.set_items(["marker", "bolt"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert wait_for(lambda: len(svc.results) == 2)
    time.sleep(0.05)
    assert eng.items_called() == ["marker", "bolt"]  # each item once on this frame, then wait


def test_new_frame_continues_rotation(make_service):
    eng = FakeEngine()
    svc = make_service(eng)
    svc.set_items(["a", "b", "c"])
    svc.submit_frame(jpeg(tag=1), capture_s=1.0)
    assert wait_for(lambda: len(svc.results) == 3)
    svc.submit_frame(jpeg(tag=2), capture_s=2.0)
    assert wait_for(lambda: len(svc.results) == 6)
    assert eng.items_called() == ["a", "b", "c", "a", "b", "c"]


def test_keeps_only_newest_frame_while_busy(make_service):
    eng = FakeEngine(block=True)
    svc = make_service(eng)
    svc.set_items(["a"])
    svc.submit_frame(jpeg(tag=1), capture_s=1.0)
    assert eng.entered.acquire(timeout=2)
    svc.submit_frame(jpeg(tag=2), capture_s=2.0)
    svc.submit_frame(jpeg(tag=3), capture_s=3.0)
    eng.release.release()  # finish frame 1
    assert eng.entered.acquire(timeout=2)
    eng.release.release()  # finish frame 3
    assert wait_for(lambda: len(svc.results) == 2)
    assert [r.capture_s for r in svc.results] == [1.0, 3.0]
    assert eng.calls[1][0] == jpeg(tag=3)
    assert svc.status()["frames_dropped"] == 1


def test_submit_frame_stays_responsive_during_slow_call(make_service):
    eng = FakeEngine(block=True)
    svc = make_service(eng)
    svc.set_items(["a"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert eng.entered.acquire(timeout=2)
    t = time.perf_counter()
    for i in range(100):
        svc.submit_frame(jpeg(tag=i % 256), capture_s=2.0 + i)
    svc.status()
    svc.set_items(["a", "b"])
    assert time.perf_counter() - t < 0.2


def test_result_has_capture_time_bearing_and_cleaned_boxes(make_service):
    # duplicate box (merged) + whole-image box (dropped) + box at the left edge
    eng = FakeEngine(boxes=[(-10, 0, 10, 10), (-9, 0, 10, 10), (0, 0, 640, 480)])
    svc = make_service(eng, mode="fast")
    svc.set_items(["bolt"])
    svc.submit_frame(jpeg(640, 480), capture_s=12.5)
    assert wait_for(lambda: len(svc.results) == 1)
    r = svc.results[0]
    assert r.item == "bolt" and r.capture_s == 12.5
    assert (r.width, r.height) == (640, 480)
    assert len(r.found) == 1
    assert r.found[0].bearing_rad == pytest.approx(math.radians(45))
    assert r.latency_s >= 0
    assert eng.calls[0][2] == "fast"


def test_empty_result_is_still_reported(make_service):
    svc = make_service(FakeEngine(boxes=[]))
    svc.set_items(["remote"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert wait_for(lambda: len(svc.results) == 1)
    assert svc.results[0].found == []  # "looked here, not seen" matters to the mission


def test_result_for_item_removed_mid_call_is_discarded(make_service):
    eng = FakeEngine(block=True)
    svc = make_service(eng)
    svc.set_items(["a", "b"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert eng.entered.acquire(timeout=2)  # "a" running
    svc.set_items(["b"])
    eng.release.release()
    assert eng.entered.acquire(timeout=2)  # next call starts: "b"
    eng.release.release()
    assert wait_for(lambda: len(svc.results) == 1)
    assert [r.item for r in svc.results] == ["b"]
    assert svc.status()["results_discarded"] == 1


def test_pause_stops_new_calls_and_resume_continues(make_service):
    eng = FakeEngine()
    svc = make_service(eng)
    svc.pause()
    svc.set_items(["a"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    time.sleep(0.1)
    assert eng.calls == [] and svc.status()["paused"] is True
    svc.resume()
    assert wait_for(lambda: len(svc.results) == 1)


def test_no_items_means_no_calls(make_service):
    eng = FakeEngine()
    svc = make_service(eng)
    svc.submit_frame(jpeg(), capture_s=1.0)
    time.sleep(0.1)
    assert eng.calls == []


def test_engine_error_is_reported_and_worker_continues(make_service):
    eng = FakeEngine(fail_items=("bad",))
    svc = make_service(eng)
    svc.set_items(["bad", "good"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert wait_for(lambda: len(svc.results) == 1)
    assert svc.results[0].item == "good"
    st = svc.status()
    assert "boom" in st["last_error"] and st["errors"] == 1


def test_undecodable_frame_is_rejected_without_calling_engine(make_service):
    eng = FakeEngine()
    svc = make_service(eng)
    svc.set_items(["a"])
    svc.submit_frame(b"not a jpeg", capture_s=1.0)
    time.sleep(0.1)
    assert eng.calls == []
    assert "JPEG" in svc.status()["last_error"]


def test_callback_exception_does_not_kill_worker(make_service):
    eng = FakeEngine()
    calls = []

    def bad_callback(r):
        calls.append(r)
        raise RuntimeError("consumer bug")

    svc = make_service(eng)
    svc._on_result = bad_callback
    svc.set_items(["a", "b"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert wait_for(lambda: len(calls) == 2)
    assert svc.status()["alive"] is True


def test_heartbeat_reports_health_periodically():
    beats = []
    svc = VisionService(FakeEngine(), on_result=lambda r: None, on_heartbeat=beats.append,
                        hfov_rad=1.0, heartbeat_s=0.02)
    svc.start()
    try:
        svc.set_items(["a"])
        svc.submit_frame(jpeg(), capture_s=1.0)
        assert wait_for(lambda: len(beats) >= 3 and beats[-1]["calls"] >= 1)
        b = beats[-1]
        assert b["alive"] is True and b["items"] == ["a"]
        assert b["last_latency_s"] is not None
        assert b["frames_received"] == 1
    finally:
        svc.stop(timeout=2)
    assert svc.status()["alive"] is False


def test_set_items_rejects_bad_names_and_strips_duplicates(make_service):
    svc = make_service(FakeEngine())
    with pytest.raises(ValueError):
        svc.set_items(["ok", "a</c>b"])
    svc.set_items([" bolt ", "bolt", "marker"])
    assert svc.status()["items"] == ["bolt", "marker"]


# --- review fixes ------------------------------------------------------------

def test_worker_survives_postprocessing_error(make_service):
    eng = FakeEngine(boxes=[(math.nan, 0.0, 10.0, 10.0)])
    svc = make_service(eng)
    svc.set_items(["a", "b"])
    svc.submit_frame(jpeg(), capture_s=1.0)
    assert wait_for(lambda: svc.status()["errors"] == 2)
    assert svc.status()["alive"] is True
    eng.boxes = [(310.0, 0.0, 330.0, 10.0)]
    svc.submit_frame(jpeg(tag=1), capture_s=2.0)
    assert wait_for(lambda: len(svc.results) == 2)


@pytest.mark.parametrize("hfov", [0.0, -1.0, math.pi, math.nan])
def test_service_rejects_bad_hfov(hfov):
    with pytest.raises(ValueError):
        VisionService(FakeEngine(), on_result=lambda r: None, hfov_rad=hfov)


def test_truncated_frame_is_rejected_not_raised(make_service):
    svc = make_service(FakeEngine())
    svc.submit_frame(b"\xff\xd8\xff\xc0\x00\x11\x08\x01", capture_s=1.0)
    assert "JPEG" in svc.status()["last_error"]
