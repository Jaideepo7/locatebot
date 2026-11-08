"""The LLM "brain": decides *what* the robot does next, one skill at a time.

``Brain.on_world_state`` is called for every new ``WorldState`` (scan rate). It is cheap and
never waits on the LLM: it decides whether a decision is due and, if so, starts at most one
LLM request in a background thread. The result is validated (``tools.parse_tool_call``) and
then emitted through ``on_skill`` (auto mode) or held for the operator (approve mode).

Failures (timeout, ``LLMError``, invalid call, no tool call) are counted; after
``max_failures`` consecutive failures, or with no LLM at all, the brain runs the fallback
routine (plain skill list, no AI) one step at a time and keeps probing the LLM every
``decide_interval_s``; the first valid LLM decision ends the fallback.

An operator ``STOP`` (``submit_operator``) always wins: it is emitted immediately, cancels any
pending proposal, abandons an in-flight request and the fallback routine, and pauses all
LLM decisions until ``resume()``.

``on_skill`` is called while the brain's lock is held (so a late LLM result can never be
emitted after an operator STOP); it must be quick (e.g. put the call on a queue).
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Callable, Sequence

from locatebot.mission.types import SkillCall, SkillName, SkillState, WorldState

from .llm import LLMClient, LLMError, LLMTimeout
from .summary import format_call, one_line, render_world_state
from .tools import CONTINUE_TOOL, InvalidSkillCall, parse_tool_call, tool_definitions, validate_skill_call

log = logging.getLogger(__name__)

TERMINAL = (SkillState.SUCCEEDED, SkillState.FAILED, SkillState.CANCELLED)
BUSY = (SkillState.RUNNING, SkillState.PENDING_APPROVAL)
QUIET_SAFETY = ("ok", "slowed")

DEFAULT_SYSTEM_PROMPT = """\
You are the high-level controller of a small indoor robot car (LiDAR + camera). You act only by \
calling exactly one of the provided skill tools per turn; the robot plans paths and drives \
itself. You never control wheels or speed.

Rules:
- A safety layer on the car can stop or slow it at any time and overrides you. If the state \
shows a safety stop or "blocked", do not repeat the same move blindly: look around, pick \
another target, or stop.
- Prefer short steps: one skill, then look at the result before the next. Use look_around \
to re-check the scene, stop when unsure or when the task is done.
- The vision model (LocateAnything) only finds items you name. Name items as short common \
nouns ("marker", "bolt", "remote"), not descriptions ("dry erase marker pen"). Search for \
the items the operator listed under "looking for".
- An item marked unconfirmed may be a false detection; prefer confirmed items for go_to and \
follow.
- Directions are relative to the robot (left/right/behind); positions are map metres.
- When you are woken while a skill is running and it is still the right thing to do, call \
`continue`. Calling the running skill again restarts it from scratch.
- Add a one-sentence `reason` to each call; the operator sees it.
Always answer with a tool call.
"""


@dataclass
class Decision:
    """One history entry: what was decided and what came of it."""

    t: float
    call: SkillCall | None
    outcome: str        # emitted | pending | approved | rejected | replaced | operator | fallback | failed: ...
    result: str = ""    # skill result once the world state reports it, e.g. "failed: no path"

    def line(self) -> str:
        c = format_call(self.call) if self.call is not None else "(no call)"
        out = f"- {c} [{self.outcome}]"
        if self.result:
            out += f" -> {self.result}"
        return out


@dataclass
class _Request:
    rid: int
    started: float
    want_seq: int
    thread: threading.Thread | None = None


class Brain:
    def __init__(
        self,
        llm: LLMClient | None,
        *,
        mode: str = "approve",
        system_prompt: str = DEFAULT_SYSTEM_PROMPT,
        llm_timeout_s: float = 15.0,
        decide_interval_s: float = 5.0,
        max_failures: int = 3,
        fallback_routine: Sequence[SkillCall] | None = None,
        on_skill: Callable[[SkillCall], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        retry_delay_s: float = 1.0,
        history_len: int = 8,
        max_abandoned: int = 4,
    ) -> None:
        if mode not in ("approve", "auto"):
            raise ValueError("mode must be 'approve' or 'auto'")
        if max_failures < 1:
            raise ValueError("max_failures must be >= 1")
        if not (llm_timeout_s > 0 and decide_interval_s > 0 and retry_delay_s >= 0):
            raise ValueError("llm_timeout_s and decide_interval_s must be > 0, retry_delay_s >= 0")
        if history_len < 1 or max_abandoned < 1:
            raise ValueError("history_len and max_abandoned must be >= 1")
        routine = [SkillCall(SkillName.STOP)] if fallback_routine is None else list(fallback_routine)
        if not routine:
            raise ValueError("fallback_routine must not be empty")
        self._routine = [replace(validate_skill_call(c), source="fallback") for c in routine]

        self._llm = llm
        self._mode = mode
        self.system_prompt = system_prompt
        self.llm_timeout_s = float(llm_timeout_s)
        self.decide_interval_s = float(decide_interval_s)
        self.max_failures = int(max_failures)
        self.retry_delay_s = float(retry_delay_s)
        self.max_abandoned = int(max_abandoned)
        self._on_skill = on_skill or (lambda call: None)
        self._clock = clock
        self._tools = tool_definitions()

        self._lock = threading.RLock()
        self._ws: WorldState | None = None
        self._want = ""                 # why a decision is wanted ("" = none)
        self._want_seq = 0
        self._inflight: _Request | None = None
        self._abandoned: list[threading.Thread] = []
        self._rid = 0
        self._last_request_t = -math.inf
        self._next_retry_t = -math.inf
        self._failures = 0
        self._total_failures = 0
        self._last_error = ""
        self._last_latency: float | None = None
        self._paused = False
        self._pending: SkillCall | None = None
        self._pending_rec: Decision | None = None
        self._history: deque[Decision] = deque(maxlen=history_len)
        self._in_fallback = False
        self._fb_index: int | None = None
        self._fb_emitted: SkillCall | None = None
        self._fb_stamp = -math.inf

    # ------------------------------------------------------------------ public API

    @property
    def mode(self) -> str:
        return self._mode

    def set_mode(self, mode: str) -> None:
        if mode not in ("approve", "auto"):
            raise ValueError("mode must be 'approve' or 'auto'")
        with self._lock:
            self._mode = mode

    def on_world_state(self, ws: WorldState) -> None:
        """Feed the newest world state. Never blocks on the LLM."""
        with self._lock:
            prev, self._ws = self._ws, ws
            self._record_results(ws)
            reason = self._trigger(prev, ws)
            if reason:
                self._want = reason
                self._want_seq += 1
            self._step(ws)

    def poll(self) -> None:
        """Re-check timeouts / due decisions / fallback with the last world state."""
        with self._lock:
            if self._ws is not None:
                self._step(self._ws)

    def pending(self) -> SkillCall | None:
        with self._lock:
            return self._pending

    def approve(self) -> SkillCall | None:
        """Emit the pending proposal (approve mode). Returns it, or None if nothing pending."""
        with self._lock:
            call = self._pending
            if call is None:
                return None
            self._pending = None
            if self._pending_rec is not None:
                self._pending_rec.outcome = "approved"
            self._pending_rec = None
            self._emit(call)
            return call

    def reject(self) -> bool:
        with self._lock:
            if self._pending is None:
                return False
            if self._pending_rec is not None:
                self._pending_rec.outcome = "rejected"
            self._pending = None
            self._pending_rec = None
            self._last_request_t = self._clock()  # next periodic decision after decide_interval_s
            return True

    def submit_operator(self, call: SkillCall) -> SkillCall:
        """Emit an operator call immediately (validated; raises ``InvalidSkillCall``).

        STOP also pauses LLM decisions until ``resume()``. Any operator call cancels the
        pending proposal and the fallback routine.
        """
        call = validate_skill_call(replace(call, source="operator"))
        with self._lock:
            if self._pending_rec is not None:
                self._pending_rec.outcome = "cancelled by operator"
            self._pending = None
            self._pending_rec = None
            self._fb_index = None
            self._fb_emitted = None
            if call.name is SkillName.STOP:
                self._paused = True
                self._abandon_inflight()
                self._want = ""
            self._history.append(Decision(self._clock(), call, "operator"))
            self._emit(call)
            return call

    def resume(self) -> None:
        """Allow LLM decisions again after an operator STOP; asks for a decision soon."""
        with self._lock:
            if not self._paused:
                return
            self._paused = False
            self._want = "operator resumed"
            self._want_seq += 1
            self._next_retry_t = -math.inf

    def history(self) -> list[Decision]:
        with self._lock:
            return [replace(d) for d in self._history]

    def status(self) -> dict[str, Any]:
        with self._lock:
            p = self._pending
            return {
                "mode": self._mode,
                "paused": self._paused,
                "pending": None if p is None else {
                    "name": p.name.value, "args": dict(p.args), "reason": p.reason, "source": p.source},
                "failures": self._failures,
                "total_failures": self._total_failures,
                "in_fallback": self._in_fallback,
                "fallback_step": self._fb_index,
                "last_error": self._last_error,
                "last_latency_s": self._last_latency,
                "inflight": self._inflight is not None,
                "history": [d.line() for d in self._history],
            }

    def wait_idle(self, timeout: float = 5.0) -> bool:
        """Wait (real time) until no request thread is alive, including abandoned ones."""
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                threads = [t for t in self._abandoned if t.is_alive()]
                if self._inflight is not None and self._inflight.thread is not None:
                    threads.append(self._inflight.thread)
            if not threads:
                return True
            for t in threads:
                t.join(max(0.0, deadline - time.monotonic()))
            if time.monotonic() >= deadline:
                with self._lock:
                    busy = self._inflight is not None or any(t.is_alive() for t in self._abandoned)
                return not busy

    # ------------------------------------------------------------------ internals (lock held)

    def _step(self, ws: WorldState) -> None:
        now = self._clock()
        self._check_timeout(now)
        if self._paused:
            return
        self._maybe_request(now, ws)
        self._run_fallback(ws)

    def _trigger(self, prev: WorldState | None, ws: WorldState) -> str:
        if prev is None:
            return "start"
        sk, psk = ws.skill, prev.skill
        if sk.state in TERMINAL and (sk.state != psk.state or sk.call != psk.call):
            return f"skill {format_call(sk.call)} ended: {sk.state.value}"
        new = {i.name for i in ws.items if i.confirmed} - {i.name for i in prev.items if i.confirmed}
        if new:
            return "newly confirmed: " + ", ".join(sorted(new))
        if ws.safety_reason not in QUIET_SAFETY and ws.safety_reason != prev.safety_reason:
            return f"safety: {ws.safety_reason}"
        if ws.last_event and ws.last_event != prev.last_event:
            return f"event: {one_line(ws.last_event, 60)}"
        return ""

    def _record_results(self, ws: WorldState) -> None:
        sk = ws.skill
        if sk.state not in TERMINAL or sk.call is None:
            return
        for d in reversed(self._history):
            if d.call == sk.call and not d.result and d.outcome in ("emitted", "approved", "operator", "fallback"):
                d.result = sk.state.value + (f": {one_line(sk.detail, 60)}" if sk.detail else "")
                break

    def _check_timeout(self, now: float) -> None:
        req = self._inflight
        if req is not None and now - req.started > self.llm_timeout_s:
            self._abandon_inflight()
            self._last_request_t = now
            self._last_latency = now - req.started
            self._fail(f"timeout after {self.llm_timeout_s:g} s", now)

    def _abandon_inflight(self) -> None:
        req = self._inflight
        if req is None:
            return
        self._inflight = None
        if req.thread is not None and req.thread.is_alive():
            self._abandoned.append(req.thread)

    def _maybe_request(self, now: float, ws: WorldState) -> None:
        if self._inflight is not None:
            return
        if self._llm is None:
            if not self._in_fallback:
                self._last_error = "no LLM configured"
                self._enter_fallback("no LLM configured")
            return
        self._abandoned = [t for t in self._abandoned if t.is_alive()]
        if len(self._abandoned) >= self.max_abandoned:
            if not self._in_fallback:
                self._last_error = f"LLM unavailable: {len(self._abandoned)} requests stuck"
                self._enter_fallback(self._last_error)
            return
        if self._in_fallback:
            due = now - self._last_request_t >= self.decide_interval_s
            reason = "probing the LLM during fallback"
        elif self._want:
            due = now >= self._next_retry_t
            reason = self._want
        else:
            idle = ws.skill.state not in BUSY
            due = idle and self._pending is None and now - self._last_request_t >= self.decide_interval_s
            reason = "periodic check while idle"
        if due:
            self._start_request(now, ws, reason)

    def _start_request(self, now: float, ws: WorldState, reason: str) -> None:
        self._rid += 1
        req = _Request(self._rid, now, self._want_seq)
        user = self._build_prompt(ws, reason)
        t = threading.Thread(target=self._worker, args=(req, user), name=f"brain-llm-{req.rid}", daemon=True)
        req.thread = t
        self._inflight = req
        t.start()

    def _build_prompt(self, ws: WorldState, reason: str) -> str:
        parts = [render_world_state(ws)]
        if self._history:
            parts.append("recent decisions (oldest first):\n" + "\n".join(d.line() for d in self._history))
        parts.append(f"asked now because: {reason}")
        if self._pending is not None:
            parts.append(f"waiting for operator approval: {format_call(self._pending)} "
                         "(a new call replaces it)")
        if self._in_fallback:
            parts.append(f"note: your last {self._failures} answers failed ({one_line(self._last_error, 80)}); "
                         "a fallback routine is controlling the robot until you answer with a valid tool call")
        parts.append("Choose the next skill: call exactly one tool.")
        return "\n\n".join(parts)

    def _worker(self, req: _Request, user: str) -> None:
        choice = None
        err = ""
        try:
            choice = self._llm.decide(self.system_prompt, user, self._tools, self.llm_timeout_s)
        except LLMTimeout as e:
            err = f"timeout: {e}"
        except LLMError as e:
            err = f"llm error: {e}"
        except Exception as e:  # noqa: BLE001 - a broken client must not kill the brain
            err = f"llm crashed: {type(e).__name__}: {e}"
        with self._lock:
            if self._inflight is not req:
                log.info("discarding result of abandoned LLM request %d", req.rid)
                return
            self._inflight = None
            now = self._clock()
            self._last_latency = now - req.started
            self._last_request_t = now
            if not err and choice is None:
                err = "no tool call"
            call = None
            if not err and choice.name == CONTINUE_TOOL:
                if self._ws is not None and self._ws.skill.state in BUSY:
                    self._continue(req, now, one_line(choice.text, 200))
                    return
                err = "continue, but no skill is running"
            if not err:
                try:
                    call = parse_tool_call(choice.name, choice.args, source="llm",
                                           reason=one_line(choice.text, 200))
                except InvalidSkillCall as e:
                    err = f"invalid call: {e}"
            if err:
                self._fail(err, now)
                return
            running = self._ws.skill if self._ws is not None else None
            if (running is not None and running.state == SkillState.RUNNING and running.call is not None
                    and running.call.name == call.name and dict(running.call.args) == dict(call.args)):
                self._continue(req, now, "same skill as the one running")  # don't restart it
                return
            self._succeed(call, req, now)

    def _continue(self, req: _Request, now: float, why: str) -> None:
        """The LLM chose to let the running skill go on: a successful answer, nothing to emit."""
        self._failures = 0
        if self._in_fallback:
            log.info("LLM answered again; leaving fallback (current step keeps running)")
            self._in_fallback = False
            self._fb_index = None
            self._fb_emitted = None
            self._want = ""
        if self._want_seq == req.want_seq:
            self._want = ""
        self._history.append(Decision(now, None, "continue" + (f": {why}" if why else "")))

    def _fail(self, err: str, now: float) -> None:
        self._failures += 1
        self._total_failures += 1
        self._last_error = err
        self._history.append(Decision(now, None, f"failed: {one_line(err, 80)}"))
        log.warning("LLM decision failed (%d in a row): %s", self._failures, err)
        self._next_retry_t = now + self.retry_delay_s
        if not self._want:
            self._want = "retry after failure"
        if self._failures >= self.max_failures and not self._in_fallback:
            self._enter_fallback(err)

    def _succeed(self, call: SkillCall, req: _Request, now: float) -> None:
        self._failures = 0
        if self._in_fallback:
            log.info("LLM answered again; leaving fallback")
            self._in_fallback = False
            self._fb_index = None
            self._fb_emitted = None
            self._want = ""
        if self._want_seq == req.want_seq:
            self._want = ""
        if self._mode == "auto":
            self._history.append(Decision(now, call, "emitted"))
            self._emit(call)
        else:
            if self._pending_rec is not None:
                self._pending_rec.outcome = "replaced"
            rec = Decision(now, call, "pending")
            self._history.append(rec)
            self._pending, self._pending_rec = call, rec

    def _enter_fallback(self, why: str) -> None:
        log.warning("entering fallback routine: %s", why)
        self._in_fallback = True
        self._fb_index = 0
        self._fb_emitted = None

    def _run_fallback(self, ws: WorldState) -> None:
        if not self._in_fallback or self._fb_index is None:
            return
        if self._fb_emitted is not None:
            sk = ws.skill
            if not (sk.call == self._fb_emitted and sk.state in TERMINAL and ws.stamp_s > self._fb_stamp):
                return
            self._fb_index += 1
            self._fb_emitted = None
            if self._fb_index >= len(self._routine):
                log.info("fallback routine finished")
                self._fb_index = None
                return
        if ws.skill.state in BUSY:
            return  # let the current skill finish first
        n = len(self._routine)
        call = replace(self._routine[self._fb_index], reason=f"fallback routine step {self._fb_index + 1}/{n}")
        self._fb_emitted = call
        self._fb_stamp = ws.stamp_s
        self._history.append(Decision(self._clock(), call, "fallback"))
        self._emit(call)

    def _emit(self, call: SkillCall) -> None:
        log.info("emit %s (%s)", format_call(call), call.source)
        try:
            self._on_skill(call)
        except Exception as e:  # noqa: BLE001 - executor problems are reported, not fatal
            self._last_error = f"on_skill failed: {type(e).__name__}: {e}"
            log.exception("on_skill raised")
