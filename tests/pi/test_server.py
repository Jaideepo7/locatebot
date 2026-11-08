"""Tests for locatebot.pi.server: real HTTP on 127.0.0.1, port 0, FakeMotor, synthetic scans."""
from __future__ import annotations

import json
import re
import threading
import urllib.error
import urllib.request

import pytest

from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.pi.control import ControlConfig, PiController
from locatebot.pi.motor import FakeMotor
from locatebot.pi.server import make_server
from locatebot.safety.supervisor import SafetyConfig, SafetySupervisor


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def setup():
    clock = Clock()
    motor = FakeMotor()
    ctl = PiController(SafetySupervisor(SafetyConfig(max_speed=0.6)), motor,
                       ControlConfig(speed_limit=1.0), clock=clock)
    ctl.update_scan(simulate_scan(Room.box(12, 6), Pose2D(1, 3, 0), n_beams=720, noise_std_m=0.0,
                                  stamp_s=clock.t))
    srv = make_server(ctl, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True)
    th.start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    yield base, ctl, motor
    srv.shutdown()
    srv.server_close()
    th.join(timeout=2)


def get(url):
    with urllib.request.urlopen(url, timeout=2) as r:
        return r.status, r.headers.get("Content-Type", ""), r.read()


def post(url, body):
    data = body if isinstance(body, bytes) else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=2) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def test_index_is_self_contained(setup):
    base, _, _ = setup
    status, ctype, body = get(base + "/")
    html = body.decode()
    assert status == 200 and ctype.startswith("text/html")
    assert "E-STOP" in html
    assert "http://" not in html and "https://" not in html
    assert not re.search(r"<script[^>]+src=", html) and not re.search(r"<link[^>]+href=", html)
    assert "/drive" in html and "/status" in html and "visibilitychange" in html


def test_status_json(setup):
    base, _, _ = setup
    status, ctype, body = get(base + "/status")
    st = json.loads(body)
    assert status == 200 and ctype.startswith("application/json")
    assert st["estop"] is False and "reason" in st and "scans_per_s" in st


def test_drive_sets_manual(setup):
    base, ctl, motor = setup
    status, st = post(base + "/drive", {"throttle": 0.3, "steering": -0.2})
    assert status == 200
    ctl.tick()
    assert motor.last == (0.3, -0.2)
    assert ctl.status()["active_source"] == "manual"


def test_drive_clamps(setup):
    base, ctl, motor = setup
    assert post(base + "/drive", {"throttle": 0.5, "steering": 7})[0] == 200
    ctl.tick()
    assert motor.last == (0.5, 1.0)


@pytest.mark.parametrize("body", [
    b"not json", b"[1, 2]", b'{"throttle": 0.1}', b'{"throttle": "0.1", "steering": 0}',
    b'{"throttle": true, "steering": 0}', b'{"throttle": NaN, "steering": 0}',
    b'{"throttle": Infinity, "steering": 0}', b'{"throttle": 1e999, "steering": 0}',
    b'{"throttle": null, "steering": 0}',
])
def test_drive_rejects_invalid(setup, body):
    base, ctl, motor = setup
    post(base + "/drive", {"throttle": 0.3, "steering": 0.0})
    status, resp = post(base + "/drive", body)
    assert status == 400 and "error" in resp
    ctl.tick()
    assert motor.last == (0.0, 0.0)  # a rejected command never leaves the old one driving


def test_drive_rejects_oversized_body(setup):
    base, _, _ = setup
    status, _ = post(base + "/drive", b'{"throttle": 0, "steering": 0, "pad": "' + b"x" * 5000 + b'"}')
    assert status in (400, 413)


def test_estop_and_release(setup):
    base, ctl, motor = setup
    assert post(base + "/estop", {})[0] == 200
    assert ctl.status()["estop"] and ctl.status()["estop_source"] == "dashboard"
    post(base + "/drive", {"throttle": 0.3, "steering": 0.0})
    assert ctl.tick().reason.value == "estop"
    # release requires an explicit confirm
    assert post(base + "/release", {})[0] == 400
    assert ctl.status()["estop"]
    status, st = post(base + "/release", {"confirm": True})
    assert status == 200 and st["estop"] is False
    assert ctl.tick().reason.value == "no_command"


def test_estop_accepts_empty_body(setup):
    base, ctl, _ = setup
    assert post(base + "/estop", b"")[0] == 200
    assert ctl.status()["estop"]


def test_speed_limit(setup):
    base, ctl, motor = setup
    assert post(base + "/speed", {"limit": 0.5})[0] == 200
    post(base + "/drive", {"throttle": 1.0, "steering": 0.0})
    ctl.tick()
    assert motor.last[0] == pytest.approx(0.5)
    assert post(base + "/speed", {"limit": 2})[0] == 400
    assert post(base + "/speed", {"limit": "fast"})[0] == 400


def test_unknown_paths(setup):
    base, _, _ = setup
    with pytest.raises(urllib.error.HTTPError) as e:
        get(base + "/nope")
    assert e.value.code == 404
    assert post(base + "/nope", {})[0] == 404


def test_default_host_is_loopback():
    import inspect

    assert inspect.signature(make_server).parameters["host"].default == "127.0.0.1"


@pytest.mark.parametrize("path,body", [("/drive", {"throttle": 0.1, "steering": 0}),
                                       ("/release", {"confirm": True}), ("/speed", {"limit": 0.5})])
def test_non_json_content_type_rejected(setup, path, body):
    # blocks cross-origin "simple" POSTs (text/plain) from other pages in the browser
    base, ctl, _ = setup
    ctl.engage_estop("test")
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "text/plain"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req, timeout=2)
    assert e.value.code == 415
    assert ctl.status()["estop"]
