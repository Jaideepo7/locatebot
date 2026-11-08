"""Tests for locatebot.pi.pca9685_motor (skid-steer MotorDriver on a PCA9685 board).

The I2C bus is a fake that decodes PCA9685 register writes into per-channel duty, so these
prove the throttle/steering -> channel mapping and the fail-closed behaviour in software
only, not that the real car moves or stops.
"""
from __future__ import annotations

import math

import pytest

from locatebot.pi.motor import MotorDriver
from locatebot.pi.pca9685_motor import (
    MAX_DUTY,
    PCA9685SkidSteer,
    Wheel,
    DEFAULT_WHEELS,
)

ADDR = 0x40
MOTOR_CHANNELS = range(8)


class FakeBus:
    """smbus2-like bus that keeps a PCA9685 register file and can be told to fail."""

    def __init__(self):
        self.regs: dict[int, int] = {0x00: 0x11, 0xFE: 30}   # power-on MODE1 (sleep) + prescale
        self.fail = False
        self.writes = 0

    def _check(self, addr):
        assert addr == ADDR
        if self.fail:
            raise OSError(121, "Remote I/O error")
        self.writes += 1

    def read_byte_data(self, addr, reg):
        self._check(addr)
        return self.regs.get(reg, 0)

    def write_byte_data(self, addr, reg, value):
        self._check(addr)
        self.regs[reg] = value & 0xFF

    def write_i2c_block_data(self, addr, reg, data):
        self._check(addr)
        for i, b in enumerate(data):
            self.regs[reg + i] = b & 0xFF

    def duty(self, ch: int) -> int:
        """Effective duty 0..4095 of a channel, decoded like the chip does."""
        base = 0x06 + 4 * ch
        on_l, on_h, off_l, off_h = (self.regs.get(base + i, 0) for i in range(4))
        if off_h & 0x10:
            return 0
        if on_h & 0x10:
            return MAX_DUTY
        return ((off_h & 0x0F) << 8 | off_l) - ((on_h & 0x0F) << 8 | on_l)


def wheel_power(bus: FakeBus, wheel: Wheel) -> int:
    """+duty when the wheel's forward channel drives, -duty for reverse, 0 when off."""
    f, r = bus.duty(wheel.forward_ch), bus.duty(wheel.reverse_ch)
    assert not (f and r) or (f == r == MAX_DUTY), "both H-bridge inputs on (only allowed as brake)"
    return f - r


def sides(bus: FakeBus):
    w = DEFAULT_WHEELS
    left = (wheel_power(bus, w["front_left"]), wheel_power(bus, w["back_left"]))
    right = (wheel_power(bus, w["front_right"]), wheel_power(bus, w["back_right"]))
    return left, right


@pytest.fixture
def bus():
    return FakeBus()


@pytest.fixture
def motor(bus):
    return PCA9685SkidSteer(bus, address=ADDR)


def test_is_a_motor_driver(motor):
    assert isinstance(motor, MotorDriver)


def test_init_sets_50hz_and_all_motor_channels_off(bus):
    for ch in MOTOR_CHANNELS:                       # pretend something left the motors running
        bus.write_i2c_block_data(ADDR, 0x06 + 4 * ch, [0, 0, 0xFF, 0x0F])
    PCA9685SkidSteer(bus, address=ADDR)
    assert bus.regs[0xFE] == 121                    # 25 MHz / (4096 * 50 Hz) - 1
    assert not bus.regs[0x00] & 0x10                # awake
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)


def test_full_forward_drives_every_forward_channel(motor, bus):
    motor.set(1.0, 0.0)
    (fl, bl), (fr, br) = sides(bus)
    assert fl == bl == fr == br == MAX_DUTY


def test_full_reverse_drives_every_reverse_channel(motor, bus):
    motor.set(-1.0, 0.0)
    (fl, bl), (fr, br) = sides(bus)
    assert fl == bl == fr == br == -MAX_DUTY


def test_positive_steering_pivots_left(motor, bus):
    # robot frame: +steering = left (counter-clockwise) -> left side back, right side forward
    motor.set(0.0, 1.0)
    (fl, bl), (fr, br) = sides(bus)
    assert fl < 0 and bl < 0
    assert fr > 0 and br > 0


def test_negative_steering_pivots_right(motor, bus):
    motor.set(0.0, -1.0)
    (fl, bl), (fr, br) = sides(bus)
    assert fl > 0 and bl > 0
    assert fr < 0 and br < 0


def test_forward_with_left_steering_arcs_left(motor, bus):
    motor.set(0.6, 0.2)
    (fl, bl), (fr, br) = sides(bus)
    assert 0 < fl == bl < fr == br


def test_side_ratio_is_kept_when_a_side_saturates(motor, bus):
    motor.set(1.0, 1.0)                              # left 0, right 2 -> scaled to left 0, right 1
    (fl, bl), (fr, br) = sides(bus)
    assert fl == bl == 0
    assert fr == br == MAX_DUTY


def test_out_of_range_inputs_are_clamped(motor, bus):
    motor.set(5.0, 0.0)
    a = sides(bus)
    motor.set(1.0, 0.0)
    assert sides(bus) == a


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, None, "fast"])
def test_non_numeric_or_non_finite_input_stops(motor, bus, bad):
    motor.set(1.0, 0.0)
    motor.set(bad, 0.0)
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)
    motor.set(1.0, 0.0)
    motor.set(0.0, bad)
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)


def test_tiny_commands_are_off_not_a_stalled_buzz(motor, bus):
    motor.set(0.01, 0.0)
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)


def test_small_command_starts_at_the_duty_floor(bus):
    m = PCA9685SkidSteer(bus, address=ADDR, min_duty=700)
    m.set(0.1, 0.0)
    (fl, _), _ = sides(bus)
    assert 700 <= fl < 1200


def test_stop_turns_motors_off_and_leaves_camera_servos_alone(motor, bus):
    bus.write_i2c_block_data(ADDR, 0x06 + 4 * 8, [0, 0, 0x33, 0x01])   # pan servo pulse
    bus.write_i2c_block_data(ADDR, 0x06 + 4 * 9, [0, 0, 0x34, 0x01])   # tilt servo pulse
    servo_before = {r: bus.regs[r] for r in range(0x06 + 32, 0x06 + 40)}
    motor.set(0.8, 0.3)
    motor.stop()
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)
    assert {r: bus.regs[r] for r in range(0x06 + 32, 0x06 + 40)} == servo_before


def test_stop_never_raises_and_records_the_error(motor, bus):
    bus.fail = True
    motor.stop()                                     # must not raise
    assert isinstance(motor.last_error, OSError)


def test_set_bus_error_raises_so_the_controller_fails_closed(motor, bus):
    # PiController.tick() catches exceptions from set() and calls stop(); set must not hide them
    bus.fail = True
    with pytest.raises(OSError):
        motor.set(0.5, 0.0)


def test_close_stops_and_later_commands_do_nothing(motor, bus):
    motor.set(0.7, 0.0)
    motor.close()
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)
    writes = bus.writes
    motor.set(1.0, 0.0)
    assert bus.writes == writes
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)


def test_brake_shorts_both_inputs_of_every_motor(motor, bus):
    motor.set(0.7, 0.0)
    motor.brake()
    assert all(bus.duty(ch) == MAX_DUTY for ch in MOTOR_CHANNELS)
    motor.stop()
    assert all(bus.duty(ch) == 0 for ch in MOTOR_CHANNELS)


def test_wheel_table_must_cover_two_wheels_per_side(bus):
    bad = dict(DEFAULT_WHEELS)
    del bad["back_right"]
    with pytest.raises(ValueError):
        PCA9685SkidSteer(bus, address=ADDR, wheels=bad)


def test_wheel_channels_must_be_distinct_motor_channels(bus):
    bad = dict(DEFAULT_WHEELS)
    bad["back_right"] = Wheel(forward_ch=8, reverse_ch=4)          # ch 8 is the pan servo
    with pytest.raises(ValueError):
        PCA9685SkidSteer(bus, address=ADDR, wheels=bad)
