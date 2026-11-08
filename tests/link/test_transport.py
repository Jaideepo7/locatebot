"""Tests for locatebot.link.transport (ZeroMQ PUB/SUB over 127.0.0.1 and inproc, no hardware).

Slow joiner: a SUB socket only receives messages published after its subscription reached
the PUB. Tests call ``join`` (publish a probe until it arrives) before asserting anything.
Burst semantics (newest-only, e-stop delivery) use ``inproc``: once ``send`` returns the
message is already in the receiver's pipe, so a single drain sees the whole burst and the
assertions are exact, not timing-dependent.
"""
from __future__ import annotations

import itertools
import time

import numpy as np
import pytest
import zmq

from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.link.messages import (
    ORIN_PUB_PORT,
    PI_PUB_PORT,
    DriveCmd,
    EStopCmd,
    Frame,
    MissionCancel,
    MissionRequest,
    MissionStatus,
    OrinStatus,
    PiStatus,
    Range,
    encode,
)
from locatebot.link.transport import (
    ORIN_HOST_ENV,
    PI_HOST_ENV,
    DriveReceiver,
    DriveUpdate,
    Publisher,
    Subscriber,
    bind_endpoint,
    orin_pub_endpoint,
    pi_pub_endpoint,
)

_ids = itertools.count()


def inproc() -> str:
    return f"inproc://test-transport-{next(_ids)}"


TCP_ANY = "tcp://127.0.0.1:*"  # ZeroMQ picks a free port; Publisher.endpoint resolves it


def pi_status(**kw) -> PiStatus:
    base = dict(stamp_s=10.0, lidar_ok=True, scans_per_s=9.8, camera_ok=True, estop=False,
                safety_reason="ok", throttle=0.2, steering=-0.1, cpu_temp_c=55.5)
    base.update(kw)
    return PiStatus(**base)


def orin_status(**kw) -> OrinStatus:
    base = dict(stamp_s=5.0, vision_alive=True, vision_latency_s=2.1, mission_state="explore",
                cpu_temp_c=None, gpu_temp_c=61.0, boot_id="0123456789abcdef0123456789abcdef")
    base.update(kw)
    return OrinStatus(**base)


def drive(seq: int, throttle: float = 0.3) -> DriveCmd:
    return DriveCmd(throttle=throttle, steering=-0.2, seq=seq, sent_s=100.0 + seq)


def estop(seq: int, engage: bool = True) -> EStopCmd:
    return EStopCmd(engage=engage, seq=seq, sent_s=200.0 + seq)


def join(send_probe, received, deadline_s: float = 5.0) -> None:
    """Publish a probe until the subscriber sees one (slow-joiner handshake)."""
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        send_probe()
        if received(0.02):
            return
    raise AssertionError("subscriber never joined")


def drain(sub: Subscriber) -> None:
    while sub.poll(0.0):
        pass


@pytest.fixture
def pair(request):
    """Publisher + Subscriber on a fresh endpoint; closed after the test."""
    made = []

    def make(endpoint: str, topics, **sub_kw):
        pub = Publisher(endpoint)
        sub = Subscriber(pub.endpoint, topics, **sub_kw)
        made.extend([sub, pub])
        return pub, sub

    yield make
    for s in made:
        s.close()


# --- endpoints ---------------------------------------------------------------

def test_endpoint_helpers_use_readme_ports_and_never_default_an_address(monkeypatch):
    assert bind_endpoint(PI_PUB_PORT) == "tcp://*:5555"
    assert bind_endpoint(ORIN_PUB_PORT, "eth0") == "tcp://eth0:5556"
    monkeypatch.delenv(PI_HOST_ENV, raising=False)
    monkeypatch.delenv(ORIN_HOST_ENV, raising=False)
    with pytest.raises(ValueError):
        pi_pub_endpoint()
    with pytest.raises(ValueError):
        orin_pub_endpoint()
    assert pi_pub_endpoint("10.9.8.7") == "tcp://10.9.8.7:5555"
    monkeypatch.setenv(PI_HOST_ENV, "pi.test")
    monkeypatch.setenv(ORIN_HOST_ENV, "orin.test")
    assert pi_pub_endpoint() == "tcp://pi.test:5555"
    assert orin_pub_endpoint() == "tcp://orin.test:5556"
    assert pi_pub_endpoint("override") == "tcp://override:5555"


def test_publisher_resolves_wildcard_port():
    with Publisher(TCP_ANY) as pub:
        assert pub.endpoint.startswith("tcp://127.0.0.1:")
        assert not pub.endpoint.endswith(":*")


# --- round trip --------------------------------------------------------------

ALL_MESSAGES = [
    simulate_scan(Room.box(6, 4), Pose2D(2, 2, 0.3), n_beams=450, stamp_s=12.5, seq=7, dropout=0.1),
    Frame(b"\xff\xd8" + bytes(range(256)) * 40, capture_s=12.4, seq=3),
    pi_status(cpu_temp_c=None),
    drive(5),
    estop(6, engage=False),
    orin_status(),
    Range(stamp_s=12.6, sensor="front_ultrasonic", range_m=None, fov_rad=0.26),
    MissionRequest(mission_id="m-1", target="blue toolbox", issued_s=1.7e9, expires_s=1.7e9 + 600),
    MissionCancel(mission_id="m-1", reason="USER_CANCELLED"),
    MissionStatus(mission_id="m-1", state="SEARCHING", message="Looking for blue toolbox", reason=""),
]


@pytest.mark.parametrize("transport", ["tcp", "inproc"])
def test_every_message_type_round_trips(pair, transport):
    topics = ["scan", "frame", "pi_status", "drive", "estop", "orin_status", "range",
              "mission_request", "mission_cancel", "mission_status"]
    pub, sub = pair(TCP_ANY if transport == "tcp" else inproc(), topics)
    join(lambda: pub.send(pi_status(stamp_s=-1.0)), lambda t: sub.recv(t) is not None)
    drain(sub)

    for m in ALL_MESSAGES:
        assert pub.send(m)
    got = []
    end = time.monotonic() + 5.0
    while len(got) < len(ALL_MESSAGES) and time.monotonic() < end:
        m = sub.recv(0.1)
        if m is not None:
            got.append(m)
    assert len(got) == len(ALL_MESSAGES)
    for want, have in zip(ALL_MESSAGES, got):
        assert type(have) is type(want)
        if hasattr(want, "ranges_m"):
            assert (want.stamp_s, want.seq) == (have.stamp_s, have.seq)
            np.testing.assert_allclose(have.ranges_m, want.ranges_m.astype(np.float32), equal_nan=True)
            np.testing.assert_array_equal(have.quality, want.quality)
        else:
            assert have == want
    assert sub.malformed == 0


def test_topic_filter_only_delivers_subscribed_topics(pair):
    pub, sub = pair(inproc(), ["drive"])
    join(lambda: pub.send(drive(0)), lambda t: sub.recv(t) is not None)
    drain(sub)
    pub.send(orin_status())
    pub.send(estop(1))
    pub.send(drive(2))
    assert sub.poll(1.0) == [drive(2)]


def test_recv_times_out_with_none(pair):
    _, sub = pair(inproc(), ["drive"])
    t0 = time.monotonic()
    assert sub.recv(0.05) is None
    assert time.monotonic() - t0 < 1.0
    assert sub.poll(0.0) == []


# --- newest-only -------------------------------------------------------------

def test_newest_only_keeps_only_the_last_of_a_burst_per_topic(pair):
    pub, sub = pair(inproc(), ["frame", "pi_status"], newest_only={"frame"})
    join(lambda: pub.send(pi_status(stamp_s=-1.0)), lambda t: bool(sub.poll(t)))
    drain(sub)

    for i in range(20):
        pub.send(Frame(b"\xff\xd8%d" % i, capture_s=float(i), seq=i))
        pub.send(pi_status(stamp_s=float(i)))
    got = sub.poll(1.0)
    frames = [m for m in got if isinstance(m, Frame)]
    statuses = [m for m in got if isinstance(m, PiStatus)]
    assert [f.seq for f in frames] == [19]
    assert [s.stamp_s for s in statuses] == [float(i) for i in range(20)]  # not conflated
    assert sub.superseded == 19
    assert got[-2] == frames[0]  # kept at its own (last) arrival position, before the last status


def test_newest_only_over_tcp_ends_on_the_newest_and_never_goes_backwards(pair):
    pub, sub = pair(TCP_ANY, ["frame"], newest_only={"frame"})
    join(lambda: pub.send(Frame(b"x", 0.0, -1)), lambda t: bool(sub.poll(t)))
    drain(sub)
    # Stay below SNDHWM/RCVHWM (100): a full PUB queue drops the *newest* message.
    for i in range(50):
        pub.send(Frame(b"\xff\xd8" * 100, capture_s=float(i), seq=i))
    seen = []
    end = time.monotonic() + 5.0
    while (not seen or seen[-1] != 49) and time.monotonic() < end:
        got = sub.poll(0.1)
        assert len(got) <= 1
        seen += [f.seq for f in got]
    assert seen[-1] == 49
    assert seen == sorted(seen)


# --- malformed ---------------------------------------------------------------

def test_malformed_messages_are_counted_and_dropped():
    ep = inproc()
    raw = zmq.Context.instance().socket(zmq.PUB)
    raw.bind(ep)
    try:
        with Subscriber(ep, ["drive"]) as sub:
            good = encode(drive(1))
            join(lambda: raw.send_multipart(good), lambda t: sub.recv(t) is not None)
            drain(sub)
            assert sub.malformed == 0
            raw.send_multipart([b"drive", b"not json", b""])
            raw.send_multipart([b"drive"])
            raw.send_multipart([b"drive", b'{"v":1,"type":"drive","throttle":5,"steering":0,'
                                          b'"seq":2,"sent_s":0}', b""])
            raw.send_multipart([b"drivex", b"{}", b""])  # prefix-matches the subscription
            raw.send_multipart(encode(drive(3)))
            got = sub.poll(1.0)
            assert got == [drive(3)]
            assert sub.malformed == 4
    finally:
        raw.close(linger=0)


# --- sending never blocks ----------------------------------------------------

def test_send_never_blocks_without_a_subscriber():
    with Publisher(TCP_ANY) as pub:
        t0 = time.monotonic()
        for i in range(5000):
            pub.send(drive(i))
        assert time.monotonic() - t0 < 2.0


def test_send_never_blocks_when_the_subscriber_stops_reading():
    with Publisher(TCP_ANY, sndhwm=2) as pub, Subscriber(pub.endpoint, ["frame"], rcvhwm=2) as sub:
        join(lambda: pub.send(Frame(b"x", 0.0, -1)), lambda t: sub.recv(t) is not None)
        big = Frame(b"\xff" * 200_000, capture_s=1.0, seq=1)
        t0 = time.monotonic()
        for _ in range(500):  # ~100 MB: far beyond HWMs and kernel buffers
            pub.send(big)
        assert time.monotonic() - t0 < 5.0


# --- DriveReceiver -----------------------------------------------------------

@pytest.fixture
def orin_to_pi():
    pub = Publisher(inproc())
    rx = DriveReceiver(pub.endpoint)

    def joined():
        # drive and estop use separate sockets: both must have joined
        seq = itertools.count(-1000)
        join(lambda: pub.send(drive(next(seq))), lambda t: rx.poll(t).drive is not None)
        join(lambda: pub.send(estop(0)), lambda t: bool(rx.poll(t).estops))
        while rx.poll(0.0) != DriveUpdate(None, ()):
            pass
        rx.reset()

    yield pub, rx, joined
    rx.close()
    pub.close()


def test_drive_receiver_returns_newest_drive_and_every_estop(orin_to_pi):
    pub, rx, joined = orin_to_pi
    joined()
    for i in range(1, 11):
        pub.send(drive(i))
        if i in (3, 7):
            pub.send(estop(i, engage=(i == 3)))
    up = rx.poll(1.0)
    assert up.drive == drive(10)
    assert up.estops == (estop(3, True), estop(7, False))  # release passed through as data
    assert rx.last_seq == 10
    assert rx.superseded == 9


def test_drive_receiver_waits_for_estop_alone(orin_to_pi):
    pub, rx, joined = orin_to_pi
    joined()
    pub.send(estop(1))
    up = rx.poll(1.0)
    assert up.drive is None and up.estops == (estop(1),)


def test_drive_receiver_drops_replayed_and_old_seq(orin_to_pi):
    pub, rx, joined = orin_to_pi
    joined()
    pub.send(drive(5))
    assert rx.poll(1.0).drive == drive(5)
    pub.send(drive(5))
    pub.send(drive(4, throttle=-0.5))
    assert rx.poll(0.2).drive is None
    assert rx.dropped_old == 2
    pub.send(drive(3))
    pub.send(drive(8))
    pub.send(drive(6))  # reordered inside one poll: the highest new seq wins
    assert rx.poll(1.0).drive == drive(8)
    assert rx.last_seq == 8


def test_drive_receiver_reset_accepts_lower_seq_after_orin_restart(orin_to_pi):
    pub, rx, joined = orin_to_pi
    joined()
    pub.send(drive(50))
    assert rx.poll(1.0).drive == drive(50)
    pub.send(drive(0))
    assert rx.poll(0.2).drive is None
    rx.reset()
    assert rx.last_seq is None
    pub.send(drive(0))
    assert rx.poll(1.0).drive == drive(0)


def test_drive_receiver_counts_malformed(orin_to_pi):
    pub, rx, joined = orin_to_pi
    joined()
    pub._sock.send_multipart([b"drive", b"garbage", b""])
    pub._sock.send_multipart([b"estop", b"{}", b""])
    pub.send(drive(1))
    assert rx.poll(1.0).drive == drive(1)
    assert rx.malformed == 2


def test_drive_receiver_timeout_returns_empty_update(orin_to_pi):
    _, rx, _ = orin_to_pi
    t0 = time.monotonic()
    assert rx.poll(0.05) == DriveUpdate(None, ())
    assert time.monotonic() - t0 < 1.0


def test_drive_receiver_over_tcp():
    with Publisher(TCP_ANY) as pub, DriveReceiver(pub.endpoint) as rx:
        seq = itertools.count(1)
        join(lambda: pub.send(drive(next(seq))), lambda t: rx.poll(t).drive is not None)
        join(lambda: pub.send(estop(0)), lambda t: bool(rx.poll(t).estops))
        pub.send(drive(10_000))
        end = time.monotonic() + 5.0
        last = None
        while last != drive(10_000) and time.monotonic() < end:
            last = rx.poll(0.1).drive or last
        assert last == drive(10_000)


# --- close -------------------------------------------------------------------

def test_close_is_clean_idempotent_and_quick():
    pub = Publisher(TCP_ANY, sndhwm=1000)
    sub = Subscriber(pub.endpoint, ["frame"])
    rx = DriveReceiver(pub.endpoint)
    join(lambda: pub.send(Frame(b"x", 0.0, -1)), lambda t: sub.recv(t) is not None)
    for i in range(200):
        pub.send(Frame(b"\xff" * 50_000, capture_s=1.0, seq=i))  # unread backlog
    t0 = time.monotonic()
    for s in (sub, rx, pub):
        s.close()
        s.close()
    assert time.monotonic() - t0 < 2.0
    assert pub.closed and sub.closed and rx.closed
    with pytest.raises(RuntimeError):
        pub.send(drive(1))
    with pytest.raises(RuntimeError):
        sub.poll(0.0)
    with pytest.raises(RuntimeError):
        rx.poll(0.0)


# --- tools/link_echo.py ------------------------------------------------------

def test_link_echo_prints_decoded_messages():
    import importlib.util
    from pathlib import Path

    spec = importlib.util.spec_from_file_location(
        "link_echo", Path(__file__).resolve().parents[2] / "tools" / "link_echo.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert "scan seq=7" in mod.describe(ALL_MESSAGES[0])
    with Publisher(TCP_ANY) as pub:
        lines = []
        assert mod.main([pub.endpoint, "--topic", "drive", "--seconds", "0.3"], out=lines.append) == 0
    assert lines[-1].startswith("received=")
