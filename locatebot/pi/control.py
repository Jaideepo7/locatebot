"""Pi control loop: pick a drive command, gate it through the safety supervisor, drive the motor.

Priority (docs/PROJECT.md §3): E-stop > LiDAR safety stop > manual > mission.

- ``update_scan`` keeps only the newest scan (an older-stamped scan is ignored).
- ``submit_manual`` (local dashboard) and ``submit_mission`` (decoded ``DriveCmd`` from the
  Orin) are re-stamped with the Pi clock **on receipt**; the supervisor stops the car when
  the command is older than ``SafetyConfig.command_timeout_s`` (hold-to-drive).
- Manual takeover: while a manual command arrived within ``manual_hold_s``, mission commands
  are ignored and manual stays the active source (a stale manual command means "stop", not
  "hand back"). After the hold, only mission commands received *after* the last manual
  command can drive.
- Mission commands with ``seq <= last_seq`` are dropped; ``reset_mission_seq()`` is for a
  restarted Orin.
- E-stop latches from any source and stops the motor immediately. A network
  ``EStopCmd(engage=False)`` never releases it; only ``release_estop_local()`` does, and it
  clears every held command so the car does not drive on release.
- ``tick()`` never lets an exception escape: any error stops the motor and reports
  ``Reason.FAULT``.

All public methods are thread-safe (HTTP server thread + control loop thread).
"""
from __future__ import annotations

import enum
import math
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from typing import Callable

from locatebot.lidar.types import Scan
from locatebot.link.messages import DriveCmd, EStopCmd, PiStatus
from locatebot.safety.supervisor import DriveCommand, Reason, SafetyDecision, SafetySupervisor

from .motor import MotorDriver


class PiReason(enum.Enum):
    """Stop reasons produced by the controller itself that are not in the supervisor's ``Reason``.

    A fault inside ``tick`` reports ``Reason.FAULT`` (supervisor enum).
    """

    CLOSED = "closed"  # controller closed: motor stopped and no longer driven


@dataclass(frozen=True)
class ControlConfig:
    manual_hold_s: float = 1.0      # manual keeps control this long after its last command
    speed_limit: float = 0.3        # initial manual throttle scale, (0, 1]; the page slider changes it
    scan_rate_window_s: float = 2.0  # window for the scans/s estimate

    def __post_init__(self) -> None:
        if not (math.isfinite(self.manual_hold_s) and self.manual_hold_s >= 0.0):
            raise ValueError("manual_hold_s must be finite and >= 0")
        _check_limit(self.speed_limit)
        if not self.scan_rate_window_s > 0.0:
            raise ValueError("scan_rate_window_s must be > 0")


def _check_limit(v) -> float:
    if not (_is_number(v) and 0.0 < v <= 1.0):
        raise ValueError(f"speed limit must be a number in (0, 1], got {v!r}")
    return float(v)


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


class PiController:
    def __init__(self, supervisor: SafetySupervisor, motor: MotorDriver, config: ControlConfig | None = None,
                 clock: Callable[[], float] = time.monotonic):
        self.supervisor = supervisor
        self.motor = motor
        self.config = config or ControlConfig()
        self._clock = clock
        self._lock = threading.RLock()
        self._speed_limit = self.config.speed_limit
        self._scan: Scan | None = None
        self._scan_rx: deque[float] = deque()
        self._manual: DriveCommand | None = None
        self._manual_rx: float | None = None
        self._mission: DriveCommand | None = None
        self._mission_rx: float | None = None
        self._last_seq: int | None = None
        self._estop_source: str | None = None
        self._source = "none"
        self._reason: str | None = None
        self._fault: str | None = None
        self._fault_trace: str | None = None  # last fault traceback, for debugging
        self._out = (0.0, 0.0)
        self._closed = False
        self.mission_dropped = 0   # replayed / reordered / invalid
        self.mission_ignored = 0   # arrived during manual takeover
        self.network_release_ignored = 0

    # --- inputs ------------------------------------------------------------------

    def update_scan(self, scan: Scan) -> None:
        with self._lock:
            if self._scan is not None and scan.stamp_s < self._scan.stamp_s:
                return
            self._scan = scan
            now = self._clock()
            self._scan_rx.append(now)
            self._trim_scan_rx(now)

    def submit_manual(self, throttle: float, steering: float) -> None:
        """Hold-to-drive command from the local dashboard. Invalid values raise ValueError
        and replace the held manual command with a stop."""
        with self._lock:
            now = self._clock()
            if not (_is_number(throttle) and _is_number(steering)):
                self._manual = DriveCommand(0.0, 0.0, now)
                self._manual_rx = now
                raise ValueError(f"throttle and steering must be finite numbers, got {throttle!r}, {steering!r}")
            t = _clamp(throttle) * self._speed_limit
            self._manual = DriveCommand(t, _clamp(steering), now)
            self._manual_rx = now

    def submit_mission(self, cmd: DriveCmd) -> bool:
        """Mission command from the Orin. Returns True if it was accepted."""
        if not isinstance(cmd, DriveCmd):
            raise TypeError(f"expected DriveCmd, got {type(cmd).__name__}")
        with self._lock:
            now = self._clock()
            seq = cmd.seq
            if not isinstance(seq, int) or isinstance(seq, bool) or (
                    self._last_seq is not None and seq <= self._last_seq):
                self.mission_dropped += 1
                return False
            self._last_seq = seq
            t, s = cmd.throttle, cmd.steering
            if not (_is_number(t) and _is_number(s) and abs(t) <= 1.0 and abs(s) <= 1.0):
                self.mission_dropped += 1
                return False
            if self._manual_active(now):
                self.mission_ignored += 1
                return False
            self._mission = DriveCommand(float(t), float(s), now)
            self._mission_rx = now
            return True

    def reset_mission_seq(self) -> None:
        """Accept any seq again (the Orin restarted). Also drops the held mission command."""
        with self._lock:
            self._last_seq = None
            self._mission = None
            self._mission_rx = None

    def set_speed_limit(self, limit: float) -> None:
        with self._lock:
            self._speed_limit = _check_limit(limit)

    # --- e-stop ------------------------------------------------------------------

    def engage_estop(self, source: str = "unknown") -> None:
        with self._lock:
            self.supervisor.engage_estop()
            if self._estop_source is None:
                self._estop_source = str(source)
            self._clear_commands()
            self._safe_stop()
            self._out = (0.0, 0.0)
            self._reason = "estop"

    def handle_estop(self, msg: EStopCmd) -> None:
        """Network e-stop message: engage latches; engage=False is ignored (never releases)."""
        if not isinstance(msg, EStopCmd):
            raise TypeError(f"expected EStopCmd, got {type(msg).__name__}")
        if msg.engage is True:
            self.engage_estop("network")
        else:
            with self._lock:
                self.network_release_ignored += 1

    def release_estop_local(self) -> None:
        """Release the latched e-stop. Only call this from a local action on the Pi."""
        with self._lock:
            self._clear_commands()
            self.supervisor.release_estop()
            self._estop_source = None

    # --- loop --------------------------------------------------------------------

    def tick(self) -> SafetyDecision:
        with self._lock:
            if self._closed:
                return SafetyDecision(0.0, 0.0, PiReason.CLOSED)  # type: ignore[arg-type]
            try:
                now = self._clock()
                command, self._source = self._select(now)
                decision = self.supervisor.evaluate(command, self._scan, now)
                self.motor.set(decision.throttle, decision.steering)
                self._out = (decision.throttle, decision.steering)
                self._reason = decision.reason.value
                self._fault = None
                return decision
            except Exception as e:  # noqa: BLE001 - fail closed on anything
                self._fault = f"{type(e).__name__}: {e}"
                self._fault_trace = traceback.format_exc()
                self._safe_stop()
                self._out = (0.0, 0.0)
                self._reason = Reason.FAULT.value
                return SafetyDecision(0.0, 0.0, Reason.FAULT)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._clear_commands()
            self._safe_stop()
            self._out = (0.0, 0.0)
            self._reason = PiReason.CLOSED.value
            try:
                self.motor.close()
            except Exception as e:  # noqa: BLE001
                self._fault = f"{type(e).__name__}: {e}"

    # --- status ------------------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            try:
                now = self._clock()
            except Exception:  # noqa: BLE001
                now = float("nan")
            age = None
            if self._scan is not None and math.isfinite(now):
                age = now - self._scan.stamp_s
            return {
                "estop": self.supervisor.estop_engaged,
                "estop_source": self._estop_source,
                "active_source": self._source,
                "reason": self._reason,
                "fault": self._fault,
                "throttle": self._out[0],
                "steering": self._out[1],
                "scan_age_s": age,
                "scans_per_s": self._scans_per_s(now),
                "speed_limit": self._speed_limit,
                "manual_hold_s": self.config.manual_hold_s,
                "command_timeout_s": self.supervisor.config.command_timeout_s,
                "mission_last_seq": self._last_seq,
                "mission_dropped": self.mission_dropped,
                "mission_ignored": self.mission_ignored,
                "network_release_ignored": self.network_release_ignored,
                "closed": self._closed,
            }

    def pi_status(self, camera_ok: bool = False, cpu_temp_c: float | None = None) -> PiStatus:
        st = self.status()
        age = st["scan_age_s"]
        now = self._clock()
        return PiStatus(
            stamp_s=float(now),
            lidar_ok=age is not None and age <= self.supervisor.config.scan_timeout_s,
            scans_per_s=float(st["scans_per_s"]),
            camera_ok=bool(camera_ok),
            estop=bool(st["estop"]),
            safety_reason=st["reason"] or "none",
            throttle=float(st["throttle"]),
            steering=float(st["steering"]),
            cpu_temp_c=cpu_temp_c,
        )

    # --- internals (call with the lock held) ---------------------------------------

    def _manual_active(self, now: float) -> bool:
        return self._manual_rx is not None and now - self._manual_rx <= self.config.manual_hold_s

    def _select(self, now: float) -> tuple[DriveCommand | None, str]:
        if self._manual_active(now):
            return self._manual, "manual"
        if self._mission is not None and (self._manual_rx is None or self._mission_rx > self._manual_rx):
            return self._mission, "mission"
        return None, "none"

    def _clear_commands(self) -> None:
        self._manual = None
        self._manual_rx = None
        self._mission = None
        self._mission_rx = None
        self._source = "none"

    def _safe_stop(self) -> None:
        try:
            self.motor.stop()
        except Exception as e:  # noqa: BLE001 - nothing more we can do in software
            msg = f"motor.stop failed: {type(e).__name__}: {e}"
            self._fault = f"{self._fault}; {msg}" if self._fault else msg

    def _trim_scan_rx(self, now: float) -> None:
        while self._scan_rx and (now - self._scan_rx[0] > self.config.scan_rate_window_s or len(self._scan_rx) > 1000):
            self._scan_rx.popleft()

    def _scans_per_s(self, now: float) -> float:
        if not math.isfinite(now):
            return 0.0
        self._trim_scan_rx(now)
        if len(self._scan_rx) < 2:
            return 0.0
        span = self._scan_rx[-1] - self._scan_rx[0]
        return (len(self._scan_rx) - 1) / span if span > 0 else 0.0
