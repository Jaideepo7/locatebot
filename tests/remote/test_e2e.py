"""End-to-end scenarios E1-E12 with no robot and no network.

Real code on the Pi side: Photon webhook handler -> MissionService -> FakeRobotLink (the scripted
stand-in for the Orin), and replies through OutboxSender to a recording provider.
Only the robot, Photon's servers and the clock are replaced. Ported from chanabyte's
tests/backend/test_e2e.py on feat/photon-dispatch (which used JetsonAgent + ScriptedSink over an
in-memory WebSocket).
"""
import base64
import collections
import json
import pathlib
import secrets

from locatebot.remote.messaging import SendResult
from locatebot.remote.photon import PhotonWebhookHandler, WebhookVerifier, sign_payload
from locatebot.remote.missions import MissionService
from locatebot.remote.notifier import OutboxSender
from locatebot.remote.protocol import MissionState as S
from locatebot.remote.robot import FakeRobotLink
from locatebot.remote.store import Store

FIXTURE = json.loads((pathlib.Path(__file__).resolve().parents[1] / "fixtures" / "photon_message_received.json").read_text())
PROJECT = FIXTURE["data"]["projectId"]
SECRET = "whsec_" + base64.b64encode(b"e2e-test-secret-e2e-test-secret!").decode()
CUSTOMERS = {"a": ("pho_spc_01jaaaaaaaaaaaaaaaaaaaaaaa", "pho_usr_01jaaaaaaaaaaaaaaaaaaaaaaa"),
             "b": ("pho_spc_01jbbbbbbbbbbbbbbbbbbbbbbb", "pho_usr_01jbbbbbbbbbbbbbbbbbbbbbbb")}
SPACE_TO_CUSTOMER = {v[0]: k for k, v in CUSTOMERS.items()}


class Clock:
    def __init__(self):
        self.t = 1_794_000_000.0

    def __call__(self):
        return self.t


class Telegram:
    """Recording messaging provider: what each customer would see in Telegram."""
    name = "photon"

    def __init__(self):
        self.chats = collections.defaultdict(list)
        self.keys = []
        self.fail_next = 0

    def send_text(self, conversation, text, idempotency_key):
        if self.fail_next:
            self.fail_next -= 1
            return SendResult(False, retryable=True, detail="HTTP 503")
        self.keys.append(idempotency_key)
        self.chats[SPACE_TO_CUSTOMER[conversation.conversation_id]].append(text)
        return SendResult(True)


class World:
    def __init__(self, script="happy", ttl_s=600, robot_online=True):
        self.clock = Clock()
        self.telegram = Telegram()
        self.store = Store(":memory:")
        self.link = FakeRobotLink(script, clock=self.clock, online=robot_online)
        self.service = MissionService(self.store, self.link, robot_id="orin-01", clock=self.clock, ttl_s=ttl_s,
                                      ack_timeout_s=10, link_grace_s=15)
        self.link.listener = self.service
        self.outbox = OutboxSender(self.store, self.telegram, clock=self.clock, base_backoff_s=1)
        self.webhook = PhotonWebhookHandler(WebhookVerifier(SECRET, clock=self.clock), PROJECT,
                                            self.service.handle_inbound)

    @property
    def robot_received(self):
        return self.link.received

    @property
    def silent(self):
        return self.link.silent

    @silent.setter
    def silent(self, value):
        self.link.silent = value

    def connect(self):
        self.link.connect()
        self.pump()

    def drop(self):
        self.link.drop()

    def pump(self):
        self.link.poll()
        self.outbox.run_once()

    def run(self, seconds, step=0.5):
        for _ in range(int(seconds / step)):
            self.clock.t += step
            self.link.poll()
            self.service.tick()
            self.pump()

    # --- the customer -----------------------------------------------------------------------------
    def say(self, text, customer="a", event_id=None, repeat=1, sign=True):
        ev = json.loads(json.dumps(FIXTURE))
        space, user = CUSTOMERS[customer]
        ev["data"]["object"]["space"]["id"] = space
        ev["data"]["object"]["sentBy"]["id"] = user
        ev["data"]["object"]["content"] = {"type": "text", "format": "plain", "text": text}
        ev["id"] = event_id or "pho_evt_" + secrets.token_hex(13)
        body = json.dumps(ev).encode()
        statuses = []
        for _ in range(repeat):
            ts = int(self.clock())
            headers = {"webhook-id": ev["id"], "webhook-timestamp": str(ts),
                       "webhook-signature": sign_payload(SECRET, ev["id"], ts, body) if sign else "v1,AAAA"}
            statuses.append(self.webhook.handle(headers, body))
            self.pump()
        return statuses

    def chat(self, customer="a"):
        return self.telegram.chats[customer]


def test_e1_happy_path():
    w = World("happy")
    assert w.say("/start") == [204]
    w.say("/find blue toolbox")
    w.run(10)
    assert w.chat()[1:] == [
        "Request received. Sending your target to the robot.",
        "The robot has started searching for: blue toolbox.",
        "Target detected. Preparing to approach it.",
        "The robot is approaching the target.",
        "Search mission completed.",
    ]
    assert w.robot_received[0]["target_description"] == "blue toolbox"


def test_e2_not_found():
    w = World("not_found")
    w.say("/find blue toolbox")
    w.run(10)
    assert w.chat() == ["Request received. Sending your target to the robot.",
                        "The robot has started searching for: blue toolbox.",
                        "I couldn't spot blue toolbox. The search has ended."]


def test_e3_robot_offline():
    w = World(robot_online=False)
    w.say("/find mug")
    assert w.chat() == ["The robot is currently unavailable. Please try again later."]
    assert w.store.active_mission("orin-01") is None


def test_e4_duplicate_deliveries():
    w = World("happy")
    assert w.say("/find blue toolbox", repeat=3) == [204, 204, 204]
    w.run(10)
    assert len([f for f in w.robot_received if f["type"] == "MISSION_REQUEST"]) == 1
    assert w.chat().count("Request received. Sending your target to the robot.") == 1
    assert w.chat()[-1] == "Search mission completed."


def test_e5_cancel():
    w = World("stall")
    w.say("/find blue toolbox")
    w.run(2)
    w.say("/cancel")
    assert w.chat()[-1] == "Cancelling... I'll confirm once the robot has stopped."
    w.run(2)
    assert w.chat()[-1] == "Mission cancelled."
    assert w.link.active_mission_id is None


def test_e6_disconnect_robot_stops_itself():
    w = World("stall")
    w.say("/find blue toolbox")
    w.run(2)
    w.drop()
    assert w.link.active_mission_id is None  # ScriptedSink applied the safe-stop policy at once
    w.pump()
    assert w.chat()[-1] == "Lost contact with the robot. It will stop on its own for safety."
    w.run(20)
    assert w.chat()[-1] == "Mission ended: the robot is offline."


def test_e7_reconnect_within_grace_reports_real_state():
    w = World("stall")
    w.say("/find blue toolbox")
    w.run(2)
    w.drop()
    w.run(5)
    w.connect()  # the agent re-reports: the robot stopped when the link dropped
    assert w.chat()[-2:] == ["Reconnected to the robot.",
                             "The robot stopped the search after losing contact with the server."]
    m = w.store.latest_for("photon", CUSTOMERS["a"][0])
    assert (m.state, m.fail_reason) == (S.CANCELLED, "LINK_LOST")


def test_e8_expired():
    w = World("stall", ttl_s=5)
    w.say("/find blue toolbox")
    w.run(8)
    assert "The search for blue toolbox timed out." in w.chat()
    assert w.link.active_mission_id is None  # both sides expire it; whichever is first, the result is the same
    m = w.store.latest_for("photon", CUSTOMERS["a"][0])
    assert (m.state, m.fail_reason) == (S.FAILED, "EXPIRED")
    assert w.chat().count("The search for blue toolbox timed out.") == 1


def test_e9_busy():
    w = World("stall")
    w.say("/find a b")
    w.run(2)
    w.say("/find red cup")
    assert w.chat()[-1] == "I'm already searching for: a b. Send /cancel first if you want something else."


def test_e10_bad_input_never_reaches_robot():
    w = World("happy")
    w.say("/find")
    w.say("/find ;reboot")
    w.say("/find $(rm -rf /)")
    assert w.robot_received == []
    assert len(w.chat()) == 3


def test_e11_forged_webhook():
    w = World("happy")
    assert w.say("/find blue toolbox", sign=False) == [400]
    assert w.robot_received == [] and w.chat() == []


def test_e12_two_customers():
    w = World("stall")
    w.say("/find blue toolbox", customer="a")
    w.run(2)
    w.say("/find red cup", customer="b")
    w.say("/cancel", customer="b")
    w.say("/status", customer="b")
    assert w.chat("b") == ["The robot is busy with another request. Please try again in a few minutes.",
                           "You don't have an active mission.",
                           "You don't have any missions yet. Send /find <object> to start one."]
    assert all(f["type"] != "MISSION_CANCEL" for f in w.robot_received)
    assert not any("busy" in t for t in w.chat("a"))


def test_robot_rejects_when_its_perception_is_busy():
    w = World("reject")
    w.say("/find blue toolbox")
    assert w.chat()[-1] == "The robot couldn't start this search (it is busy)."


def test_silent_robot_times_out():
    w = World("happy")
    w.silent = True
    w.say("/find blue toolbox")
    w.run(12)
    assert w.chat()[-1] == "The robot didn't respond. Please try again."


def test_telegram_outage_is_retried_without_duplicates():
    w = World("happy")
    w.telegram.fail_next = 3
    w.say("/find blue toolbox")
    w.run(20)
    assert w.chat()[-1] == "Search mission completed."
    assert len(w.telegram.keys) == len(set(w.telegram.keys))
    assert w.chat().count("Request received. Sending your target to the robot.") == 1
