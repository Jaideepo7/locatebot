import json
import pathlib

from locatebot.backend.robot_link import RobotLink

TOKEN = "t" * 40


class Conn:
    def __init__(self):
        self.sent, self.closed = [], None

    def send_text(self, text):
        self.sent.append(json.loads(text))

    def close(self, code):
        self.closed = code


class Listener:
    def __init__(self):
        self.events = []

    def on_robot_connected(self):
        self.events.append("up")

    def on_robot_disconnected(self):
        self.events.append("down")


def link():
    lst = Listener()
    return RobotLink("jetson-01", TOKEN, lst), lst


def test_auth_requires_matching_device_and_bearer_token():  # L1
    lk, _ = link()
    assert lk.authenticate("jetson-01", f"Bearer {TOKEN}")
    assert not lk.authenticate("jetson-02", f"Bearer {TOKEN}")
    assert not lk.authenticate("jetson-01", f"Bearer {TOKEN}x")
    assert not lk.authenticate("jetson-01", TOKEN)
    assert not lk.authenticate("jetson-01", None)
    assert not lk.authenticate(None, f"Bearer {TOKEN}")


def test_constant_time_compare():  # L3
    src = pathlib.Path(__file__).resolve().parents[2].joinpath("locatebot/backend/robot_link.py").read_text()
    assert "hmac.compare_digest" in src


def test_send_requires_connection():
    lk, lst = link()
    assert not lk.connected
    assert not lk.send({"type": "MISSION_CANCEL", "mission_id": "x"})
    c = Conn()
    lk.attach(c)
    assert lk.connected and lst.events == ["up"]
    msg = {"type": "MISSION_CANCEL", "mission_id": "3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70"}
    assert lk.send(msg) and c.sent == [msg]


def test_newest_connection_wins():  # L4
    lk, lst = link()
    a, b = Conn(), Conn()
    lk.attach(a)
    lk.attach(b)
    assert a.closed == 1012 or a.closed is not None
    lk.detach(a)  # the old socket's handler exiting must not mark the robot offline
    assert lk.connected and lst.events == ["up", "up"]
    lk.detach(b)
    assert not lk.connected and lst.events == ["up", "up", "down"]


def test_send_error_marks_failure():
    class Broken(Conn):
        def send_text(self, text):
            raise ConnectionError

    lk, _ = link()
    lk.attach(Broken())
    assert not lk.send({"type": "MISSION_CANCEL", "mission_id": "3f2b8c1e-9a4d-4c6e-8f1a-2b3c4d5e6f70"})
