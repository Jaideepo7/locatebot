"""Vision worker: runs LocateAnything in the background on the newest camera frame.

Rules (docs/PROJECT.md §7):
- Vision never blocks anything else. ``submit_frame``, ``set_items``, ``pause`` and
  ``status`` only touch shared state under a lock; the model runs on a worker thread.
- Only the newest frame is kept. A frame replaced before any call used it counts as dropped.
- One item per call, rotating through the list. Each item runs at most once per frame;
  when every item has been tried on the current frame the worker waits for a new one.
- A running call cannot be cancelled. If its item was removed meanwhile, the result is
  discarded.
- Every finished call produces a ``VisionResult``, including "nothing found", stamped with
  the frame's *capture* time so the mapper can look up the robot pose at that moment.

Transport (receiving frames from the Pi, publishing results) is not part of this module.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from .postprocess import Box, Camera, Detection, bearing_rad, build_prompt, check_hfov, clean_detections, jpeg_size

log = logging.getLogger(__name__)


class Engine(Protocol):
    def locate(self, image: bytes, prompt: str, mode: str = "hybrid") -> list[Detection]: ...


@dataclass(frozen=True)
class Found:
    label: str
    box: Box  # pixels
    bearing_rad: float  # robot frame, left positive


@dataclass(frozen=True)
class VisionResult:
    item: str
    capture_s: float  # monotonic capture time of the frame (sender's clock)
    width: int
    height: int
    found: list[Found] = field(default_factory=list)
    latency_s: float = 0.0
    mode: str = "hybrid"


@dataclass
class _Frame:
    seq: int
    jpeg: bytes
    capture_s: float
    width: int
    height: int


class VisionService:
    def __init__(
        self,
        engine: Engine,
        on_result: Callable[[VisionResult], None],
        hfov_rad: float,
        mode: str = "hybrid",
        on_heartbeat: Callable[[dict], None] | None = None,
        heartbeat_s: float = 1.0,
    ):
        check_hfov(hfov_rad)
        self._engine = engine
        self._on_result = on_result
        self._on_heartbeat = on_heartbeat
        self._hfov = hfov_rad
        self._mode = mode
        self._heartbeat_s = heartbeat_s

        self._cond = threading.Condition()
        self._stop = False
        self._paused = False
        self._items: list[str] = []
        self._rot = 0
        self._frame: _Frame | None = None
        self._tried: set[str] = set()
        self._seq = 0
        self._busy_item: str | None = None
        self._stats = {
            "frames_received": 0, "frames_dropped": 0, "calls": 0, "errors": 0,
            "results_discarded": 0, "last_latency_s": None, "last_error": "",
        }
        self._worker = threading.Thread(target=self._run, name="vision-worker", daemon=True)
        self._beater = threading.Thread(target=self._beat, name="vision-heartbeat", daemon=True)

    # --- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        self._worker.start()
        if self._on_heartbeat:
            self._beater.start()

    def stop(self, timeout: float | None = None) -> None:
        """Stop the threads. A model call already running finishes first (it can't be cancelled)."""
        with self._cond:
            self._stop = True
            self._cond.notify_all()
        for t in (self._worker, self._beater):
            if t.is_alive():
                t.join(timeout)

    # --- control (never blocks on the model) ------------------------------
    def submit_frame(self, jpeg: bytes, capture_s: float) -> None:
        try:
            width, height = jpeg_size(jpeg)
        except ValueError as e:
            with self._cond:
                self._stats["last_error"] = f"bad frame: {e} (not a decodable JPEG)"
            return
        with self._cond:
            self._stats["frames_received"] += 1
            if self._frame is not None and not self._tried:
                self._stats["frames_dropped"] += 1
            self._seq += 1
            self._frame = _Frame(self._seq, jpeg, capture_s, width, height)
            self._tried = set()
            self._cond.notify_all()

    def set_items(self, items: list[str]) -> None:
        clean: list[str] = []
        for it in items:
            build_prompt(it)  # validates: non-empty, single item
            it = it.strip()
            if it not in clean:
                clean.append(it)
        with self._cond:
            self._items = clean
            self._rot = 0
            self._cond.notify_all()

    def pause(self) -> None:
        with self._cond:
            self._paused = True

    def resume(self) -> None:
        with self._cond:
            self._paused = False
            self._cond.notify_all()

    def status(self) -> dict:
        with self._cond:
            return {
                **self._stats,
                "alive": self._worker.is_alive() and not self._stop,
                "paused": self._paused,
                "items": list(self._items),
                "busy_item": self._busy_item,
                "mode": self._mode,
            }

    # --- worker ------------------------------------------------------------
    def _next_job(self) -> tuple[str, _Frame] | None:
        if self._paused or not self._items or self._frame is None:
            return None
        n = len(self._items)
        for k in range(n):
            idx = (self._rot + k) % n
            item = self._items[idx]
            if item not in self._tried:
                self._tried.add(item)
                self._rot = (idx + 1) % n
                return item, self._frame
        return None

    def _run(self) -> None:
        while True:
            with self._cond:
                job = None
                while not self._stop and (job := self._next_job()) is None:
                    self._cond.wait(0.5)
                if self._stop:
                    return
                item, frame = job
                self._busy_item = item

            t0 = time.perf_counter()
            found: list[Found] = []
            try:
                dets = self._engine.locate(frame.jpeg, build_prompt(item), mode=self._mode)
                cam = Camera.from_hfov(frame.width, self._hfov)
                found = [Found(d.label, d.box, bearing_rad(d.box, cam))
                         for d in clean_detections(dets, frame.width, frame.height)]
                if not all(math.isfinite(f.bearing_rad) for f in found):
                    raise ValueError("non-finite bearing")
                error = None
            except Exception as e:  # keep the worker alive whatever the engine or post-processing does
                error = e
            latency = time.perf_counter() - t0

            with self._cond:
                self._busy_item = None
                self._stats["calls"] += 1
                self._stats["last_latency_s"] = round(latency, 3)
                if error is not None:
                    self._stats["errors"] += 1
                    self._stats["last_error"] = f"{item}: {error}"
                    log.warning("vision call failed for %r: %s", item, error)
                    continue
                if item not in self._items:
                    self._stats["results_discarded"] += 1
                    continue

            result = VisionResult(item, frame.capture_s, frame.width, frame.height, found,
                                  round(latency, 3), self._mode)
            try:
                self._on_result(result)
            except Exception:
                log.exception("on_result callback failed")

    def _beat(self) -> None:
        while True:
            with self._cond:
                if self._cond.wait_for(lambda: self._stop, timeout=self._heartbeat_s):
                    return
            try:
                self._on_heartbeat(self.status())
            except Exception:
                log.exception("on_heartbeat callback failed")
