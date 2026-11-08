"""Store-guide agent: catalog search, state machine, tool allowlist, check-in, verify, enrichment.

All offline: a scripted chat LLM, a mock robot and a fake locator. Proves the code's rules, not
the real model's choices.
"""
import json

import pytest

from locatebot.agent.llm import ChatReply, LLMError, LLMTimeout, LLMTruncated, ToolCall
from locatebot.store.agent import STAFF_FALLBACK, State, StoreAgent, StoreConfig, tools_for
from locatebot.store.catalog import MemoryCatalog, Product
from locatebot.store.enrich import ATTRIBUTE_KEYS, enrich_product, parse_attributes
from locatebot.store.robot import Box, FakeLocator, MockRobot, position_text
from locatebot.store.verify import NemotronVerifier, Verdict, parse_verdict

PRODUCTS = [
    Product("p1", "Paper towels", "AISLE_2", "middle", 14, aliases=("kitchen roll",),
            grounding_query="a white cylindrical pack of paper towels with a blue stripe"),
    Product("p2", "Peanut butter", "AISLE_1", "top", 0),
    Product("p3", "Potato chips", "AISLE_3", "bottom", 5),
    Product("p4", "Tortilla chips", "AISLE_3", "middle", 2),
    Product("p5", "AA batteries", "AISLE_3", "top", 6),
]
CODE = "4821"


class ScriptedChat:
    """Fake chat LLM: pops replies; records (messages, tool names, max_tokens) per call."""

    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    def chat(self, messages, *, tools=None, timeout_s, max_tokens=None):
        self.calls.append((json.loads(json.dumps(messages)), [t["name"] for t in tools or []], max_tokens))
        item = self.script.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item


def say(text):
    return ChatReply(text, [], "stop", {"role": "assistant", "content": text})


def call(name, **args):
    tc = ToolCall(f"c-{name}", name, args)
    msg = {"role": "assistant", "content": "", "tool_calls": [
        {"id": tc.id, "type": "function", "function": {"name": name, "arguments": json.dumps(args)}}]}
    return ChatReply("", [tc], "tool_calls", msg)


class FakeVerifier:
    def __init__(self, verdict=Verdict(True, "high", "blue stripe visible")):
        self.verdict = verdict
        self.calls = []

    def verify(self, frame, box, product):
        self.calls.append((frame, box, product.id))
        if isinstance(self.verdict, BaseException):
            raise self.verdict
        return self.verdict


def make(script, *, robot=None, locator=None, verifier=None):
    llm = ScriptedChat(script)
    agent = StoreAgent(llm, MemoryCatalog(PRODUCTS), robot or MockRobot(frame=b"JPEG"),
                       locator or FakeLocator([Box(0.1, 0.4, 0.3, 0.6, 0.9)]),
                       verifier or FakeVerifier(), StoreConfig(store_code=CODE))
    return agent, llm


def tool_results(llm_call):
    return [json.loads(m["content"]) for m in llm_call[0] if m["role"] == "tool"]


# ------------------------------------------------------------------ catalog

def test_catalog_search():
    c = MemoryCatalog(PRODUCTS)
    assert [p.id for p in c.search("paper towels")] == ["p1"]
    assert [p.id for p in c.search("Paper Towel")] == ["p1"]
    assert [p.id for p in c.search("papr towels")] == ["p1"]
    assert [p.id for p in c.search("kitchen roll")] == ["p1"]
    assert sorted(p.id for p in c.search("chips")) == ["p3", "p4"]
    assert c.search("motor oil") == []
    assert c.search("") == []


def test_catalog_from_json(tmp_path):
    f = tmp_path / "store.json"
    f.write_text(json.dumps({"products": [{"id": "x", "name": "Tea", "aisle": "AISLE_1", "shelf": "top", "qty": 3,
                                           "attributes": {"major_color": "green"}}]}))
    (p,) = MemoryCatalog.from_json(f).search("tea")
    assert p.qty == 3 and p.attributes == {"major_color": "green"} and p.aliases == ()


# ------------------------------------------------------------------ robot helpers

def test_box_and_position():
    assert position_text(Box(0.0, 0.4, 0.3, 0.6)) == "middle shelf, left side"
    assert position_text(Box(0.4, 0.0, 0.6, 0.2)) == "top shelf, center"
    assert position_text(Box(0.7, 0.8, 0.9, 1.0)) == "bottom shelf, right side"
    with pytest.raises(ValueError):
        Box(0.5, 0.1, 0.4, 0.2)
    with pytest.raises(ValueError):
        Box(-0.1, 0.1, 0.4, 0.2)


# ------------------------------------------------------------------ tool allowlist per state

def test_tools_per_state():
    names = lambda s: {t["name"] for t in tools_for(s, ("ENTRANCE", "AISLE_1"))}
    assert names(State.IDLE) == {"search_product"}
    assert names(State.OFFERED) == {"search_product", "record_decision"}
    assert names(State.WANTS_GUIDE) == {"search_product"}
    assert names(State.GUIDING) == {"navigate_to", "find_in_view", "verify_crop", "finish_guidance"}
    nav = next(t for t in tools_for(State.GUIDING, ("ENTRANCE", "AISLE_1")) if t["name"] == "navigate_to")
    assert nav["input_schema"]["properties"]["waypoint"]["enum"] == ["ENTRANCE", "AISLE_1"]


# ------------------------------------------------------------------ conversation flows

def test_in_stock_offers_guide_and_first_reply_has_demo_notice():
    agent, llm = make([call("search_product", query="paper towels"), say("Aisle 2. Want a guide?")])
    reply = agent.handle_text("u1", "do you have paper towels?")
    assert "mock inventory" in reply and reply.endswith("Aisle 2. Want a guide?")
    assert agent.session("u1").state is State.OFFERED and agent.session("u1").product.id == "p1"
    (res,) = tool_results(llm.calls[1])
    assert res["matches"][0]["in_stock"] is True and res["matches"][0]["aisle"] == "AISLE_2"
    assert llm.calls[0][1] == ["search_product"]
    assert llm.calls[1][1] == ["search_product", "record_decision"]  # tools follow the new state
    # second reply: no notice again
    llm.script = [say("ok")]
    assert agent.handle_text("u1", "cool") == "ok"


def test_out_of_stock_stays_idle_and_cannot_navigate():
    robot = MockRobot(frame=b"J")
    agent, llm = make([call("search_product", query="peanut butter"), call("navigate_to", waypoint="AISLE_1"),
                       say("Sorry, out of stock.")], robot=robot)
    agent.handle_text("u1", "peanut butter?")
    assert agent.session("u1").state is State.IDLE and robot.visits == []
    res = tool_results(llm.calls[2])
    assert res[0]["matches"][0]["in_stock"] is False
    assert "not available" in res[1]["error"]


def test_ambiguous_search_keeps_state():
    agent, llm = make([call("search_product", query="chips"), say("Potato or tortilla?")])
    agent.handle_text("u1", "chips")
    assert agent.session("u1").state is State.IDLE and agent.session("u1").product is None
    (res,) = tool_results(llm.calls[1])
    assert res["ambiguous"] is True and len(res["matches"]) == 2


def test_guide_requires_code_checked_in_code_not_llm():
    robot = MockRobot(frame=b"J")
    agent, llm = make([call("search_product", query="paper towels"), say("Want a guide?"),
                       call("record_decision", wants_guide=True), say("Send the entrance code."),
                       say("That is not the code."),
                       call("navigate_to", waypoint="AISLE_2"), say("Follow me!")], robot=robot)
    agent.handle_text("u1", "paper towels")
    agent.handle_text("u1", "yes please")
    assert agent.session("u1").state is State.WANTS_GUIDE
    agent.handle_text("u1", "take me to batteries now")  # no code -> no navigate tool exposed
    assert llm.calls[4][1] == ["search_product"] and robot.visits == []
    agent.handle_text("u1", f"  {CODE} ")
    assert agent.session("u1").state is State.GUIDING
    assert robot.visits == ["AISLE_2"] and agent.session("u1").arrived
    assert "checked in" in json.dumps(llm.calls[5][0]).lower()


def guided(agent, user="u1"):
    s = agent.session(user)
    s.state, s.product = State.WANTS_GUIDE, PRODUCTS[0]
    s.greeted = True


def test_navigate_rejects_other_waypoints():
    robot = MockRobot(frame=b"J")
    agent, llm = make([call("navigate_to", waypoint="PARKING_LOT"), call("navigate_to", waypoint="AISLE_1"),
                       say("I can only take you to aisle 2.")], robot=robot)
    guided(agent)
    agent.handle_text("u1", CODE)
    res = tool_results(llm.calls[2])
    assert "error" in res[0] and "error" in res[1] and robot.visits == []


def test_find_and_verify_full_flow():
    verifier = FakeVerifier()
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), call("find_in_view"), call("verify_crop", box=0),
                       say("Found it: middle shelf, left side.")], verifier=verifier)
    guided(agent)
    assert agent.handle_text("u1", CODE) == "Found it: middle shelf, left side."
    res = tool_results(llm.calls[3])
    assert res[0] == {"status": "arrived"}
    assert res[1]["found"] == 1 and res[1]["boxes"][0]["position"] == "middle shelf, left side"
    assert res[2]["match"] is True and res[2]["confidence"] == "high"
    assert verifier.calls[0][0] == b"JPEG" and verifier.calls[0][2] == "p1"


def test_find_before_arrival_is_refused():
    agent, llm = make([call("find_in_view"), call("verify_crop", box=0), say("hm")])
    guided(agent)
    agent.handle_text("u1", CODE)
    res = tool_results(llm.calls[2])
    assert "error" in res[0] and "error" in res[1]


def test_empty_shelf_reports_honestly():
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), call("find_in_view"), say("The shelf looks empty.")],
                      locator=FakeLocator([]))
    guided(agent)
    agent.handle_text("u1", CODE)
    assert tool_results(llm.calls[2])[1]["found"] == 0


def test_verifier_rejects_wrong_product():
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), call("find_in_view"), call("verify_crop", box=0),
                       say("That's not it.")], verifier=FakeVerifier(Verdict(False, "high", "different brand")))
    guided(agent)
    agent.handle_text("u1", CODE)
    assert tool_results(llm.calls[3])[2]["match"] is False


def test_finish_guidance_sends_robot_home_and_releases_it():
    robot = MockRobot(frame=b"J")
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), say("Here."),
                       call("finish_guidance"), say("Bye!")], robot=robot)
    guided(agent)
    agent.handle_text("u1", CODE)
    agent.handle_text("u1", "thanks, done")
    assert robot.visits == ["AISLE_2", "ENTRANCE"]
    assert agent.session("u1").state is State.IDLE and agent.robot_owner is None


def test_one_guided_session_at_a_time():
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), say("Follow me")])
    guided(agent, "u1")
    guided(agent, "u2")
    agent.handle_text("u1", CODE)
    reply = agent.handle_text("u2", CODE)
    assert "another shopper" in reply and agent.session("u2").state is State.WANTS_GUIDE
    assert len(llm.calls) == 2  # u2 never reached the LLM


def test_robot_failure_is_reported_not_hidden():
    robot = MockRobot(frame=b"J", statuses=["blocked"])
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), say("I'm blocked.")], robot=robot)
    guided(agent)
    agent.handle_text("u1", CODE)
    assert tool_results(llm.calls[1])[0] == {"status": "blocked"} and not agent.session("u1").arrived


# ------------------------------------------------------------------ failures and limits

def test_truncation_retries_once_with_bigger_limit():
    agent, llm = make([LLMTruncated(), say("hi")])
    assert agent.handle_text("u1", "hello").endswith("hi")
    assert llm.calls[0][2] == 2048 and llm.calls[1][2] == 4096


def test_api_failure_twice_gives_staff_fallback():
    agent, llm = make([LLMTimeout(), LLMError("HTTP 503", status=503)])
    assert agent.handle_text("u1", "hello").endswith(STAFF_FALLBACK)


def test_non_retryable_error_no_retry():
    agent, llm = make([LLMError("HTTP 400", status=400, retryable=False)])
    assert agent.handle_text("u1", "hi").endswith(STAFF_FALLBACK) and len(llm.calls) == 1


def test_max_steps_gives_fallback():
    agent, llm = make([call("search_product", query="tea")] * 8)
    assert agent.handle_text("u1", "tea").endswith(STAFF_FALLBACK) and len(llm.calls) == 8


def test_empty_reply_never_none():
    agent, _ = make([say("")])
    assert agent.handle_text("u1", "??").endswith("Sorry, I didn't catch that. Could you rephrase?")


def test_bad_tool_args_rejected():
    agent, llm = make([call("search_product", query=""), call("search_product", q="x"), say("?")])
    agent.handle_text("u1", "x")
    assert all("error" in r for r in tool_results(llm.calls[2]))


def test_history_is_kept_short_and_text_only():
    agent, llm = make([say(f"r{i}") for i in range(12)])
    for i in range(12):
        agent.handle_text("u1", f"m{i}")
    msgs = llm.calls[-1][0]
    assert msgs[0]["role"] == "system" and msgs[-1] == {"role": "user", "content": "m11"}
    assert len(msgs) == 1 + 2 * StoreConfig(store_code="x").history_turns + 1
    assert all(m["role"] in ("system", "user", "assistant") for m in msgs)


# ------------------------------------------------------------------ verifier and enrichment parsing

def test_parse_verdict():
    assert parse_verdict('{"match": true, "confidence": "high", "reason": "logo"}') == Verdict(True, "high", "logo")
    assert parse_verdict('```json\n{"match": false, "confidence": "low", "reason": "blurry"}\n```').match is False
    for bad in ("yes", '{"match": "yes", "confidence": "high", "reason": "x"}',
                '{"match": true, "confidence": "very", "reason": "x"}', "[]"):
        with pytest.raises(ValueError):
            parse_verdict(bad)


def test_nemotron_verifier_sends_image_and_product():
    llm = ScriptedChat([say('{"match": true, "confidence": "medium", "reason": "stripe"}')])
    v = NemotronVerifier(llm, cropper=lambda jpeg, box: b"CROP")
    out = v.verify(b"FRAME", Box(0.1, 0.1, 0.5, 0.5), PRODUCTS[0])
    assert out == Verdict(True, "medium", "stripe")
    content = llm.calls[0][0][-1]["content"]
    assert any(p.get("type") == "image_url" for p in content)
    text = " ".join(p.get("text", "") for p in content)
    assert "Paper towels" in text and "blue stripe" in text and "cropped" in text


def test_verifier_without_cropper_describes_region():
    llm = ScriptedChat([say('{"match": false, "confidence": "low", "reason": "x"}')])
    NemotronVerifier(llm, cropper=lambda jpeg, box: None).verify(b"F", Box(0.0, 0.4, 0.3, 0.6), PRODUCTS[0])
    text = " ".join(p.get("text", "") for p in llm.calls[0][0][-1]["content"])
    assert "middle shelf, left side" in text


GOOD_ATTRS = {"category": "paper towels", "major_color": "white with a blue stripe", "shape": "cylindrical pack",
              "text_on_label": "BRAND X", "distinguishing": "blue stripe",
              "grounding_query": "a white cylindrical pack of paper towels with a blue stripe"}


def test_parse_attributes_strict():
    assert parse_attributes(json.dumps(GOOD_ATTRS)) == GOOD_ATTRS
    assert parse_attributes("```json\n" + json.dumps(GOOD_ATTRS) + "\n```") == GOOD_ATTRS
    assert set(ATTRIBUTE_KEYS) == set(GOOD_ATTRS)
    for bad in [{k: v for k, v in GOOD_ATTRS.items() if k != "shape"}, {**GOOD_ATTRS, "extra": "x"},
                {**GOOD_ATTRS, "grounding_query": ""}, {**GOOD_ATTRS, "category": 5},
                {**GOOD_ATTRS, "grounding_query": "x" * 200}]:
        with pytest.raises(ValueError):
            parse_attributes(json.dumps(bad))
    with pytest.raises(ValueError):
        parse_attributes("not json")


def test_enrich_retries_once_on_bad_json():
    llm = ScriptedChat([say("Sure! here you go"), say(json.dumps(GOOD_ATTRS))])
    assert enrich_product(llm, "Paper towels", image=b"\xff\xd8x") == GOOD_ATTRS
    assert any(p.get("type") == "image_url" for p in llm.calls[0][0][-1]["content"])
    llm = ScriptedChat([say("nope"), say("still nope")])
    with pytest.raises(ValueError):
        enrich_product(llm, "Paper towels", notes="white pack")
    with pytest.raises(ValueError):
        enrich_product(ScriptedChat([]), "Paper towels")  # needs a photo or notes


# ------------------------------------------------------------------ found in the live run (2026-11-08)

def test_no_arg_tool_ignores_stray_args_and_box_accepts_digit_string():
    # Nemotron sent find_in_view({"box": "0"}) seven times in a row
    agent, llm = make([call("navigate_to", waypoint="AISLE_2"), call("find_in_view", box="0"),
                       call("verify_crop", box="0"), say("Found it.")])
    guided(agent)
    agent.handle_text("u1", CODE)
    res = tool_results(llm.calls[3])
    assert res[1]["found"] == 1 and res[2]["match"] is True


def test_bad_args_error_names_expected_args():
    agent, llm = make([call("search_product", q="x"), say("?")])
    agent.handle_text("u1", "x")
    assert "query" in tool_results(llm.calls[1])[0]["error"]


def test_guiding_prompt_says_finish_before_new_topics():
    agent, llm = make([say("ok")])
    guided(agent)
    agent.session("u1").state = State.GUIDING
    agent.handle_text("u1", "peanut butter?")
    system = llm.calls[0][0][0]["content"]
    assert "finish_guidance" in system and "another product" in system


def test_ungrounded_aisle_claim_is_corrected():
    # live: "chips?" -> "They're in Aisle 4 on the bottom shelf" with no search_product call
    agent, llm = make([say("They're in Aisle 4 on the bottom shelf."), call("search_product", query="chips"),
                       say("Potato or tortilla chips? Both in Aisle 3.")])
    assert agent.handle_text("u1", "chips?").endswith("Both in Aisle 3.")
    assert "search_product" in json.dumps(llm.calls[1][0][-1])


def test_ungrounded_stock_claim_twice_gives_safe_reply():
    agent, llm = make([say("Yes, it's in stock!"), say("Yes, it's in stock!")])
    reply = agent.handle_text("u1", "got milk?")
    assert "in stock" not in reply and "product" in reply and len(llm.calls) == 2


def test_current_product_aisle_is_grounded():
    agent, llm = make([say("Send the entrance code and I'll take you to Aisle 2.")])
    guided(agent)
    assert agent.handle_text("u1", "how do I start?").endswith("Aisle 2.") and len(llm.calls) == 1


def test_research_same_product_keeps_wants_guide():
    agent, llm = make([call("search_product", query="paper towels"), say("Still Aisle 2. Send the code.")])
    guided(agent)
    agent.handle_text("u1", "paper towels again?")
    assert agent.session("u1").state is State.WANTS_GUIDE
