"""Skid-steer ``MotorDriver`` for a PCA9685 board driving four DC motors through H-bridges.

Hardware this was measured on (2026-11-08, see ``prototypes/pi5_freenove/README.md``): a
Raspberry Pi 5 with a Freenove-style 4WD board. PCA9685 at I2C 0x40, 50 Hz. Each wheel has
two channels feeding one H-bridge: PWM on one channel and 0 on the other drives the wheel;
both full on is a short brake; both 0 lets it coast. Channels 8/9 are the camera pan/tilt
servos and are never written here, so ``stop()`` keeps the camera where it is.

Mapping from ``set(throttle, steering)`` (both [-1, 1], +throttle forward, +steering left =
counter-clockwise, as ``SafetyDecision`` returns them):

    left  = throttle - steer_gain * steering
    right = throttle + steer_gain * steering

Both sides are divided by ``max(1, |left|, |right|)`` so the turn ratio survives saturation.
A side below ``deadband`` is off (a motor fed a tiny duty only buzzes). Otherwise its duty is
``min_duty + (max_duty - min_duty) * |power|``: the board runs the motors from a 3S pack
(~12 V), so even ~30 % duty is quick and the floor stays low.

Fail-closed rules for ``PiController``:
- ``set`` clamps out-of-range values and turns non-numeric or non-finite input into a stop.
- ``set`` lets I2C errors propagate, so ``PiController.tick`` sees the fault and calls ``stop``.
- ``stop`` never raises; the error is kept in ``last_error``.
- After ``close`` nothing is written by ``set``.

**No hardware failsafe.** The PCA9685 keeps its last outputs if this process hangs or is
killed with SIGKILL. Before unattended runs, add one: a watchdog that pulls the PCA9685 /OE
pin high or cuts motor power when ``set`` stops being called.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Mapping

MAX_DUTY = 4095
MOTOR_CHANNELS = tuple(range(8))
LEFT_WHEELS = ("front_left", "back_left")
RIGHT_WHEELS = ("front_right", "back_right")

_MODE1, _PRESCALE, _LED0 = 0x00, 0xFE, 0x06
_OSC_HZ = 25_000_000


@dataclass(frozen=True)
class Wheel:
    """The two PCA9685 channels of one wheel's H-bridge."""

    forward_ch: int
    reverse_ch: int


# Verified on the car: PWM on forward_ch spins that wheel forward (camera side = front).
DEFAULT_WHEELS: Mapping[str, Wheel] = {
    "front_left": Wheel(forward_ch=1, reverse_ch=0),
    "back_left": Wheel(forward_ch=2, reverse_ch=3),
    "back_right": Wheel(forward_ch=5, reverse_ch=4),
    "front_right": Wheel(forward_ch=6, reverse_ch=7),
}


def _finite_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _duty_bytes(duty: int) -> list[int]:
    if duty <= 0:
        return [0, 0, 0, 0x10]          # full off
    if duty >= MAX_DUTY:
        return [0, 0x10, 0, 0]          # full on
    return [0, 0, duty & 0xFF, duty >> 8]


class PCA9685SkidSteer:
    """``MotorDriver`` for four H-bridge DC motors on a PCA9685 (skid steering).

    ``bus`` is any smbus2-compatible object (``read_byte_data``, ``write_byte_data``,
    ``write_i2c_block_data``); use :func:`open_pca9685_motor` on the Pi.
    """

    def __init__(self, bus, address: int = 0x40, wheels: Mapping[str, Wheel] = DEFAULT_WHEELS,
                 *, min_duty: int = 700, max_duty: int = MAX_DUTY, steer_gain: float = 1.0,
                 deadband: float = 0.02, freq_hz: float = 50.0,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self._check_wheels(wheels)
        if not 0 <= min_duty < max_duty <= MAX_DUTY:
            raise ValueError("need 0 <= min_duty < max_duty <= 4095")
        self.bus, self.address, self.wheels = bus, address, dict(wheels)
        self.min_duty, self.max_duty = int(min_duty), int(max_duty)
        self.steer_gain, self.deadband = float(steer_gain), float(deadband)
        self.last_error: Exception | None = None
        self.closed = False
        self._sleep = sleep
        self._all_motors_off()          # before anything else: never start with motors running
        self._set_frequency(freq_hz)

    # ---- MotorDriver ----
    def set(self, throttle: float, steering: float) -> None:
        if self.closed:
            return
        if not (_finite_number(throttle) and _finite_number(steering)):
            self.stop()
            return
        t = max(-1.0, min(1.0, float(throttle)))
        s = max(-1.0, min(1.0, float(steering)))
        left = t - self.steer_gain * s
        right = t + self.steer_gain * s
        scale = max(1.0, abs(left), abs(right))
        left, right = left / scale, right / scale
        for name in LEFT_WHEELS:
            self._drive_wheel(self.wheels[name], left)
        for name in RIGHT_WHEELS:
            self._drive_wheel(self.wheels[name], right)

    def stop(self) -> None:
        """All motor channels off (coast). Never raises; leaves servo channels untouched."""
        try:
            self._all_motors_off()
        except Exception as e:          # noqa: BLE001 - stop must not raise
            self.last_error = e

    def close(self) -> None:
        self.stop()
        self.closed = True

    # ---- extras ----
    def brake(self) -> None:
        """Short-brake: both H-bridge inputs full on. Follow with ``stop()`` (~0.15 s later)."""
        if self.closed:
            return
        self.bus.write_i2c_block_data(self.address, _LED0, _duty_bytes(MAX_DUTY) * len(MOTOR_CHANNELS))

    # ---- internals ----
    @staticmethod
    def _check_wheels(wheels: Mapping[str, Wheel]) -> None:
        if set(wheels) != set(LEFT_WHEELS + RIGHT_WHEELS):
            raise ValueError(f"wheels must be exactly {LEFT_WHEELS + RIGHT_WHEELS}")
        chans = [c for w in wheels.values() for c in (w.forward_ch, w.reverse_ch)]
        if len(set(chans)) != len(chans) or not all(c in MOTOR_CHANNELS for c in chans):
            raise ValueError("wheel channels must be 8 distinct channels in 0..7 (8+ are servos)")

    def _drive_wheel(self, wheel: Wheel, power: float) -> None:
        if abs(power) < self.deadband:
            on_ch, off_ch, duty = wheel.forward_ch, wheel.reverse_ch, 0
        else:
            duty = int(self.min_duty + (self.max_duty - self.min_duty) * min(abs(power), 1.0))
            on_ch, off_ch = (wheel.forward_ch, wheel.reverse_ch) if power > 0 else (wheel.reverse_ch, wheel.forward_ch)
        # switch the opposite input off first so both are never driven at once
        self.bus.write_i2c_block_data(self.address, _LED0 + 4 * off_ch, _duty_bytes(0))
        self.bus.write_i2c_block_data(self.address, _LED0 + 4 * on_ch, _duty_bytes(duty))

    def _all_motors_off(self) -> None:
        # channels 0-7 are 32 consecutive registers: one transaction (smbus block limit is 32)
        self.bus.write_i2c_block_data(self.address, _LED0, _duty_bytes(0) * len(MOTOR_CHANNELS))

    def _set_frequency(self, freq_hz: float) -> None:
        prescale = round(_OSC_HZ / (4096 * freq_hz)) - 1
        if not 3 <= prescale <= 255:
            raise ValueError(f"PWM frequency {freq_hz} Hz out of range")
        old = self.bus.read_byte_data(self.address, _MODE1)
        self.bus.write_byte_data(self.address, _MODE1, (old & 0x7F) | 0x10)   # sleep to change prescale
        self.bus.write_byte_data(self.address, _PRESCALE, prescale)
        self.bus.write_byte_data(self.address, _MODE1, 0x20)                  # wake, auto-increment
        self._sleep(0.005)                                                    # oscillator start-up
        self.bus.write_byte_data(self.address, _MODE1, 0xA0)                  # restart PWM outputs


def open_pca9685_motor(bus_number: int = 1, address: int = 0x40, **kwargs) -> PCA9685SkidSteer:
    """Open the real board on the Pi (needs ``smbus2``, preinstalled on Raspberry Pi OS)."""
    from smbus2 import SMBus  # imported here so tests and non-Pi hosts don't need it

    return PCA9685SkidSteer(SMBus(bus_number), address, **kwargs)
