"""Tests for locatebot.pi.control and locatebot.pi.motor.

Scans are synthetic (locatebot.lidar.sim) and the motor is a FakeMotor. These prove the
arbitration and fail-closed logic in software only, not physical safety on the car.
"""
from __future__ import annotations

import math

import pytest

from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.link.messages import DriveCmd, EStopCmd, PiStatus
from locatebot.pi.control import ControlConfig, PiController
from locatebot.pi.motor import FakeMotor
from locatebot.safety.supervisor import Reason, SafetyConfig, SafetySupervisor

POSE = Pose2D(1.0, 3.0, 0.0)
ROOM = Room.box(12.0, 6.0)
SCFG = SafetyConfig(max_speed=0.6, command_timeout_s=0.3, scan_timeout_s=0.5)


class Clock:
    def __init__(self, t: float = 100.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def make_scan(t, room=ROOM, seq=0):
    return simulate_scan(room, POSE, n_beams=720, noise_std_m=0.0, stamp_s=t, seq=seq)


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def motor():
    return FakeMotor()


@pytest.fixture
def ctl(clock, motor):
    c = PiController(SafetySupervisor(SCFG), motor, ControlConfig(manual_hold_s=1.0, speed_limit=1.0),
                     clock=clock)
    c.update_scan(make_scan(clock.t))
    return c


def mission(throttle=0.3, steering=0.0, seq=1):
    return DriveCmd(throttle=throttle, steering=steering, seq=seq, sent_s=5.0)


# --- motor --------------------------------------------------------------------

def test_fake_motor_records_outputs():
    m = FakeMotor()
    m.set(0.2, -0.1)
    m.stop()
    m.close()
    assert m.outputs == [(0.2, -0.1), (0.0, 0.0)]
    assert m.stop_count == 1 and m.closed
    assert m.last == (0.0, 0.0)


# --- arbitration ----------------------------------------------------------------

def test_no_command_stops(ctl, motor):
    d = ctl.tick()
    assert d.reason is Reason.NO_COMMAND
    assert motor.last == (0.0, 0.0)
    assert ctl.status()["active_source"] == "none"


def test_mission_drives_when_alone(ctl, motor):
    assert ctl.submit_mission(mission(0.3, 0.1))
    d = ctl.tick()
    assert d.reason is Reason.OK
    assert motor.last == (0.3, 0.1)
    assert ctl.status()["active_source"] == "mission"


def test_manual_beats_mission(ctl, motor, clock):
    ctl.submit_mission(mission(0.3, 0.0, seq=1))
    ctl.submit_manual(0.2, 0.5)
    d = ctl.tick()
    assert d.reason is Reason.OK and motor.last == (0.2, 0.5)
    assert ctl.status()["active_source"] == "manual"
    # mission commands are ignored while manual holds control
    clock.t += 0.1
    assert not ctl.submit_mission(mission(0.4, 0.0, seq=2))
    ctl.submit_manual(0.1, 0.0)
    ctl.tick()
    assert motor.last == (0.1, 0.0)


def test_estop_beats_manual_and_mission(ctl, motor):
    ctl.submit_mission(mission())
    ctl.submit_manual(0.3, 0.0)
    ctl.engage_estop("test")
    ctl.submit_manual(0.3, 0.0)
    d = ctl.tick()
    assert d.reason is Reason.ESTOP and motor.last == (0.0, 0.0)


def test_safety_stop_beats_manual(clock, motor):
    room = ROOM.with_box(POSE.x + 0.3, POSE.y - 0.1, POSE.x + 0.4, POSE.y + 0.1)
    c = PiController(SafetySupervisor(SCFG), motor, ControlConfig(speed_limit=1.0), clock=clock)
    c.update_scan(make_scan(clock.t, room))
    c.submit_manual(0.3, 0.0)
    d = c.tick()
    assert d.reason is Reason.OBSTACLE and motor.last == (0.0, 0.0)


def test_safety_stop_beats_mission(clock, motor):
    room = ROOM.with_box(POSE.x + 0.3, POSE.y - 0.1, POSE.x + 0.4, POSE.y + 0.1)
    c = PiController(SafetySupervisor(SCFG), motor, clock=clock)
    c.update_scan(make_scan(clock.t, room))
    c.submit_mission(mission(0.3))
    assert c.tick().reason is Reason.OBSTACLE and motor.last == (0.0, 0.0)


def test_reverse_blocked(ctl, motor):
    ctl.submit_manual(-0.5, 0.0)
    d = ctl.tick()
    assert d.reason is Reason.REVERSE_BLOCKED and motor.last == (0.0, 0.0)


def test_no_scan_stops(clock, motor):
    c = PiController(SafetySupervisor(SCFG), motor, clock=clock)
    c.submit_manual(0.3, 0.0)
    assert c.tick().reason is Reason.NO_SCAN and motor.last == (0.0, 0.0)


def test_stale_scan_stops(ctl, motor, clock):
    clock.t += SCFG.scan_timeout_s + 0.1
    ctl.submit_manual(0.3, 0.0)
    assert ctl.tick().reason is Reason.SCAN_STALE and motor.last == (0.0, 0.0)


def test_older_scan_does_not_replace_newer(ctl, clock):
    ctl.update_scan(make_scan(clock.t - 1.0, seq=9))
    assert ctl.status()["scan_age_s"] == pytest.approx(0.0)


# --- hold to drive ----------------------------------------------------------------

def test_manual_expires_after_command_timeout(ctl, motor, clock):
    ctl.submit_manual(0.3, 0.0)
    ctl.update_scan(make_scan(clock.t))
    clock.t += SCFG.command_timeout_s - 0.05
    ctl.update_scan(make_scan(clock.t))
    assert ctl.tick().reason is Reason.OK
    clock.t += 0.1
    ctl.update_scan(make_scan(clock.t))
    d = ctl.tick()
    assert d.reason is Reason.COMMAND_STALE and motor.last == (0.0, 0.0)


def test_manual_is_restamped_on_receipt(ctl, clock):
    clock.t += 50.0
    ctl.update_scan(make_scan(clock.t))
    ctl.submit_manual(0.3, 0.0)
    assert ctl.tick().reason is Reason.OK


def test_mission_is_restamped_on_receipt_not_sent_s(ctl):
    # sent_s is the Orin's clock (far from the Pi's); it must not be used for staleness
    ctl.submit_mission(DriveCmd(throttle=0.3, steering=0.0, seq=1, sent_s=-1e6))
    assert ctl.tick().reason is Reason.OK


def test_manual_hold_then_hand_back_to_mission(ctl, motor, clock):
    ctl.submit_manual(0.2, 0.0)
    ctl.tick()
    # manual stale but still inside manual_hold_s: car stops, mission does not take over
    clock.t += 0.5
    ctl.update_scan(make_scan(clock.t))
    assert not ctl.submit_mission(mission(0.4, seq=1))
    d = ctl.tick()
    assert d.reason is Reason.COMMAND_STALE and motor.last == (0.0, 0.0)
    assert ctl.status()["active_source"] == "manual"
    # after the hold, fresh mission commands drive again
    clock.t += 0.6
    ctl.update_scan(make_scan(clock.t))
    assert ctl.submit_mission(mission(0.4, seq=2))
    d = ctl.tick()
    assert d.reason is Reason.OK and motor.last == (0.4, 0.0)
    assert ctl.status()["active_source"] == "mission"


def test_mission_from_before_takeover_not_used_after_hold(ctl, motor, clock):
    ctl.submit_mission(mission(0.4, seq=1))
    ctl.submit_manual(0.0, 0.0)
    clock.t += 1.1
    ctl.update_scan(make_scan(clock.t))
    d = ctl.tick()
    assert d.reason is Reason.NO_COMMAND and motor.last == (0.0, 0.0)


def test_zero_manual_command_stops_and_holds(ctl, motor, clock):
    ctl.submit_mission(mission(0.4, seq=1))
    ctl.submit_manual(0.0, 0.0)
    d = ctl.tick()
    assert d.reason is Reason.OK and motor.last == (0.0, 0.0)


def test_speed_limit_scales_manual_throttle(clock, motor):
    c = PiController(SafetySupervisor(SCFG), motor, ControlConfig(speed_limit=0.5), clock=clock)
    c.update_scan(make_scan(clock.t))
    c.submit_manual(0.8, 0.2)
    c.tick()
    assert motor.last == pytest.approx((0.4, 0.2))
    c.set_speed_limit(0.25)
    c.submit_manual(1.0, 0.0)
    c.tick()
    assert motor.last == pytest.approx((0.25, 0.0))
    with pytest.raises(ValueError):
        c.set_speed_limit(1.5)
    with pytest.raises(ValueError):
        c.set_speed_limit(float("nan"))


@pytest.mark.parametrize("bad", [(float("nan"), 0.0), (0.0, float("inf")), ("x", 0.0), (None, 0.0), (True, 0.0)])
def test_invalid_manual_rejected_and_stops(ctl, motor, bad):
    ctl.submit_manual(0.3, 0.0)
    with pytest.raises(ValueError):
        ctl.submit_manual(*bad)
    ctl.tick()
    assert motor.last == (0.0, 0.0)


def test_manual_clamped(ctl, motor):
    ctl.submit_manual(0.5, 3.0)
    ctl.tick()
    assert motor.last == (0.5, 1.0)


# --- mission seq ------------------------------------------------------------------

def test_mission_replay_and_reorder_dropped(ctl, motor):
    assert ctl.submit_mission(mission(0.3, seq=5))
    assert not ctl.submit_mission(mission(0.5, seq=5))
    assert not ctl.submit_mission(mission(0.5, seq=4))
    ctl.tick()
    assert motor.last == (0.3, 0.0)
    assert ctl.status()["mission_dropped"] == 2


def test_reset_mission_seq_accepts_restarted_orin(ctl, motor):
    ctl.submit_mission(mission(0.3, seq=100))
    ctl.reset_mission_seq()
    assert ctl.submit_mission(mission(0.2, seq=0))
    ctl.tick()
    assert motor.last == (0.2, 0.0)


def test_reset_mission_seq_clears_held_mission(ctl, motor):
    ctl.submit_mission(mission(0.3, seq=100))
    ctl.reset_mission_seq()
    assert ctl.tick().reason is Reason.NO_COMMAND


def test_mission_not_drivecmd_rejected(ctl):
    with pytest.raises(TypeError):
        ctl.submit_mission((0.3, 0.0))


# --- e-stop -----------------------------------------------------------------------

def test_estop_latches(ctl, motor, clock):
    ctl.engage_estop("dashboard")
    for _ in range(3):
        clock.t += 0.1
        ctl.update_scan(make_scan(clock.t))
        ctl.submit_manual(0.3, 0.0)
        assert ctl.tick().reason is Reason.ESTOP
    assert motor.last == (0.0, 0.0)
    st = ctl.status()
    assert st["estop"] is True and st["estop_source"] == "dashboard"


def test_network_estop_engages_but_never_releases(ctl, motor):
    ctl.handle_estop(EStopCmd(engage=True, seq=1, sent_s=0.0))
    assert ctl.status()["estop"] and ctl.status()["estop_source"] == "network"
    ctl.handle_estop(EStopCmd(engage=False, seq=2, sent_s=0.0))
    ctl.submit_manual(0.3, 0.0)
    assert ctl.tick().reason is Reason.ESTOP
    assert ctl.status()["estop"]


def test_network_estop_engage_ignores_seq(ctl):
    ctl.handle_estop(EStopCmd(engage=False, seq=100, sent_s=0.0))
    ctl.handle_estop(EStopCmd(engage=True, seq=1, sent_s=0.0))
    assert ctl.status()["estop"]


def test_local_release_clears_held_commands(ctl, motor):
    ctl.submit_mission(mission(0.4, seq=1))
    ctl.submit_manual(0.3, 0.0)
    ctl.engage_estop("dashboard")
    ctl.release_estop_local()
    assert not ctl.status()["estop"]
    d = ctl.tick()
    assert d.reason is Reason.NO_COMMAND and motor.last == (0.0, 0.0)
    # a new command after release drives again
    ctl.submit_manual(0.2, 0.0)
    assert ctl.tick().reason is Reason.OK


def test_commands_held_during_estop_do_not_drive_on_release(ctl, motor):
    ctl.engage_estop("dashboard")
    ctl.submit_manual(0.3, 0.0)
    ctl.submit_mission(mission(0.4, seq=1))
    ctl.release_estop_local()
    assert ctl.tick().reason is Reason.NO_COMMAND


# --- faults -----------------------------------------------------------------------

class BoomSupervisor(SafetySupervisor):
    def evaluate(self, command, scan, now):
        raise RuntimeError("boom")


class BoomMotor(FakeMotor):
    def set(self, throttle, steering):
        raise OSError("i2c gone")


def test_supervisor_exception_stops_motor(clock, motor):
    c = PiController(BoomSupervisor(SCFG), motor, clock=clock)
    c.update_scan(make_scan(clock.t))
    c.submit_manual(0.3, 0.0)
    d = c.tick()
    assert d.stopped and d.throttle == 0.0 and d.steering == 0.0
    assert d.reason.value == "fault"
    assert motor.stop_count == 1
    st = c.status()
    assert st["reason"] == "fault" and "boom" in st["fault"]


def test_motor_exception_calls_stop(clock):
    m = BoomMotor()
    c = PiController(SafetySupervisor(SCFG), m, clock=clock)
    c.update_scan(make_scan(clock.t))
    c.submit_manual(0.3, 0.0)
    d = c.tick()
    assert d.reason.value == "fault" and m.stop_count == 1


def test_motor_stop_exception_does_not_propagate(clock):
    class Worse(BoomMotor):
        def stop(self):
            super().stop()
            raise OSError("still gone")

    m = Worse()
    c = PiController(SafetySupervisor(SCFG), m, clock=clock)
    c.update_scan(make_scan(clock.t))
    c.submit_manual(0.3, 0.0)
    assert c.tick().reason.value == "fault" and m.stop_count == 1


def test_clock_exception_stops(motor):
    def bad_clock():
        raise RuntimeError("clock")

    c = PiController(SafetySupervisor(SCFG), motor, clock=bad_clock)
    assert c.tick().reason.value == "fault" and motor.stop_count == 1


def test_fault_clears_when_healthy(clock, motor):
    sup = SafetySupervisor(SCFG)
    c = PiController(sup, motor, clock=clock)
    c.update_scan(make_scan(clock.t))
    orig = sup.evaluate
    sup.evaluate = lambda *a: (_ for _ in ()).throw(RuntimeError("once"))
    c.submit_manual(0.3, 0.0)
    assert c.tick().reason.value == "fault"
    sup.evaluate = orig
    assert c.tick().reason is Reason.OK
    assert c.status()["fault"] is None


def test_close_stops_motor_and_ticks_stay_stopped(ctl, motor):
    ctl.submit_manual(0.3, 0.0)
    ctl.tick()
    ctl.close()
    assert motor.stop_count >= 1 and motor.closed
    n = len(motor.outputs)
    ctl.submit_manual(0.3, 0.0)
    d = ctl.tick()
    assert d.stopped and d.throttle == 0.0
    assert all(o == (0.0, 0.0) for o in motor.outputs[n:])


# --- status -------------------------------------------------------------------------

def test_status_fields(ctl, clock):
    for i in range(1, 11):
        clock.t += 0.1
        ctl.update_scan(make_scan(clock.t, seq=i))
    ctl.submit_manual(0.2, 0.0)
    ctl.tick()
    st = ctl.status()
    assert st["estop"] is False
    assert st["active_source"] == "manual"
    assert st["reason"] == "ok"
    assert st["throttle"] == 0.2 and st["steering"] == 0.0
    assert st["scan_age_s"] == pytest.approx(0.0)
    assert 8.0 <= st["scans_per_s"] <= 12.0
    assert st["speed_limit"] == 1.0


def test_status_without_scan(clock, motor):
    c = PiController(SafetySupervisor(SCFG), motor, clock=clock)
    st = c.status()
    assert st["scan_age_s"] is None and st["scans_per_s"] == 0.0 and st["reason"] is None


def test_pi_status_message(ctl):
    ctl.submit_manual(0.2, 0.0)
    ctl.tick()
    msg = ctl.pi_status(camera_ok=False, cpu_temp_c=None)
    assert isinstance(msg, PiStatus)
    assert msg.lidar_ok and not msg.estop and msg.safety_reason == "ok"
    assert msg.throttle == 0.2 and math.isfinite(msg.scans_per_s)
    from locatebot.link.messages import decode, encode
    assert decode(encode(msg)) == msg


def test_control_config_validation():
    with pytest.raises(ValueError):
        ControlConfig(manual_hold_s=-1.0)
    with pytest.raises(ValueError):
        ControlConfig(speed_limit=0.0)
