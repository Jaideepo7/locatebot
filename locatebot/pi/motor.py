"""Motor/steering output interface.

``MotorDriver`` is what ``PiController`` writes the safety-gated command to. Throttle and
steering are normalised to [-1, 1] (positive forward / positive left), exactly as the
``SafetyDecision`` returns them.

The real car's motor and steering hardware (ESC + servo PWM, H-bridge, or a motor HAT) is
not known yet, so there is **no hardware driver here**. A real driver must:
- map [-1, 1] to the hardware and clamp anything outside it,
- make ``stop()`` put the drive motor in neutral/brake and centre steering, and never raise
  if it can avoid it,
- have its own hardware-side failsafe (ESC signal-loss failsafe or a watchdog that
  neutralises the motor when ``set`` stops being called), because a hung Python process
  cannot stop anything.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class MotorDriver(Protocol):
    def set(self, throttle: float, steering: float) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...


class FakeMotor:
    """Records every output. For tests and bench runs without motors."""

    def __init__(self, max_records: int = 10_000) -> None:
        self.max_records = max_records  # oldest outputs are dropped so long bench runs stay small
        self.outputs: list[tuple[float, float]] = []
        self.stop_count = 0
        self.closed = False

    def _record(self, out: tuple[float, float]) -> None:
        self.outputs.append(out)
        if len(self.outputs) > self.max_records:
            del self.outputs[: len(self.outputs) - self.max_records]

    def set(self, throttle: float, steering: float) -> None:
        self._record((float(throttle), float(steering)))

    def stop(self) -> None:
        self.stop_count += 1
        self._record((0.0, 0.0))

    def close(self) -> None:
        self.closed = True

    @property
    def last(self) -> tuple[float, float] | None:
        return self.outputs[-1] if self.outputs else None
