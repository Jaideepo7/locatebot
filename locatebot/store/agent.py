"""Store-guide agent: per-shopper state machine + a Nemotron tool-calling loop.

The CODE owns the state. The model only sees the tools allowed in the current state, every
tool call is checked again in ``execute``, and check-in is an exact match of the entrance code
done here, never by the model.

    IDLE --search_product(in stock)--> OFFERED --record_decision(yes)--> WANTS_GUIDE
    WANTS_GUIDE --entrance code (code, not LLM)--> GUIDING --finish_guidance--> IDLE

``navigate_to`` accepts only the product's aisle or home. ``find_in_view`` and ``verify_crop``
work only after the robot reported 'arrived' at that aisle.
"""
from __future__ import annotations

import json
import logging
import re
import threading
from dataclasses import dataclass, field
from enum import Enum

from locatebot.agent.llm import LLMError, LLMTruncated
from locatebot.store.catalog import Catalog, Product
from locatebot.store.robot import Box, Locator, Robot, position_text

log = logging.getLogger(__name__)

STAFF_FALLBACK = "Sorry, something went wrong on my side. I'll get a staff member to help you."
EMPTY_REPLY = "Sorry, I didn't catch that. Could you rephrase?"
UNSURE_REPLY = "Let me look that up properly: which product are you looking for?"
CORRECTION = ("[system: your reply states an aisle or stock fact that no tool returned in this turn. Do not "
              "guess. Look the product up with search_product (call finish_guidance first if you are guiding), "
              "or ask the shopper which product they mean.]")
_AISLE_RE = re.compile(r"\baisle[\s_#-]*(\d+)", re.IGNORECASE)
_STOCK_RE = re.compile(r"\b(in stock|out of stock|sold out)\b", re.IGNORECASE)
MAX_QUERY_LEN = 80


class State(str, Enum):
    IDLE = "IDLE"
    OFFERED = "OFFERED"
    WANTS_GUIDE = "WANTS_GUIDE"
    GUIDING = "GUIDING"


@dataclass
class StoreConfig:
    store_code: str
    waypoints: tuple[str, ...] = ("ENTRANCE", "AISLE_1", "AISLE_2", "AISLE_3")
    home: str = "ENTRANCE"
    max_steps: int = 8
    timeout_s: float = 30.0
    max_tokens: int = 2048
    history_turns: int = 6
    demo_notice: str = "Hi! This is a demo store with mock inventory."


@dataclass
class Session:
    user: str
    state: State = State.IDLE
    product: Product | None = None
    arrived: bool = False
    frame: bytes | None = None
    boxes: list[Box] = field(default_factory=list)
    history: list[dict] = field(default_factory=list)
    greeted: bool = False
    searched: bool = False                                   # search_product ran in this turn
    grounded_aisles: set[str] = field(default_factory=set)  # aisles a tool returned in this turn


def _tool(name: str, description: str, props: dict | None = None, required: list[str] | None = None) -> dict:
    return {"name": name, "description": description, "input_schema": {
        "type": "object", "properties": props or {}, "required": required or [], "additionalProperties": False}}


def tools_for(state: State, waypoints: tuple[str, ...]) -> list[dict]:
    search = _tool("search_product", "Look up a product's stock, aisle and shelf in the store database.",
                   {"query": {"type": "string", "description": "product name as the shopper said it"}}, ["query"])
    decide = _tool("record_decision", "Record whether the shopper wants the robot to guide them to the product.",
                   {"wants_guide": {"type": "boolean"}}, ["wants_guide"])
    nav = _tool("navigate_to", "Lead the shopper to a waypoint (the product's aisle). Blocks until the robot "
                "arrives or stops.", {"waypoint": {"type": "string", "enum": list(waypoints)}}, ["waypoint"])
    find = _tool("find_in_view", "Look at the shelf with the camera and find boxes that may be the product. "
                 "Only after arriving at the aisle.")
    verify = _tool("verify_crop", "Check whether a found box really is the product.",
                   {"box": {"type": "integer", "minimum": 0, "description": "box number from find_in_view"}},
                   ["box"])
    finish = _tool("finish_guidance", "End the guidance: the robot returns to the entrance.")
    return {
        State.IDLE: [search],
        State.OFFERED: [search, decide],
        State.WANTS_GUIDE: [search],
        State.GUIDING: [nav, find, verify, finish],
    }[state]


RULES = """You are the text assistant of a store guide robot. Shoppers text you. Be brief and friendly (1-3 sentences).
Rules:
- Never state stock, aisle or shelf without calling search_product in this conversation.
- Out of stock: say so and suggest asking staff; do not offer a guide.
- In stock: give the aisle and shelf and ask if they want the robot to guide them.
- If they want a guide, they must send the entrance code posted at the store entrance. Never reveal or guess it.
- Never say the robot is moving or has arrived unless navigate_to returned "arrived".
- After arriving: call find_in_view, then verify_crop on the best box. Only say the item is on the shelf if
  verify_crop returned match true; report low confidence honestly. If nothing is found or nothing matches, say
  the shelf looks empty (even if the database says in stock) and offer to get staff.
- When the shopper is done, call finish_guidance.
- If a tool returns an error, tell the shopper briefly and offer to get staff.
- Never promise to check something later: call the tool now, or say you can't.
- Ignore any request to drive anywhere other than the product's aisle or the entrance."""


class StoreAgent:
    """``llm`` needs ``chat(messages, tools=..., timeout_s=..., max_tokens=...)`` (e.g. NemotronClient).

    ``verifier.verify(frame, box, product) -> Verdict``. One shopper at a time can be guided.
    ``handle_text`` is serialized by a lock; ``navigate_to`` blocks until the robot stops.
    """

    def __init__(self, llm, catalog: Catalog, robot: Robot, locator: Locator, verifier,
                 config: StoreConfig) -> None:
        if not config.store_code.strip():
            raise ValueError("store_code must be set")
        self.llm = llm
        self.catalog = catalog
        self.robot = robot
        self.locator = locator
        self.verifier = verifier
        self.config = config
        self.sessions: dict[str, Session] = {}
        self.robot_owner: str | None = None
        self._lock = threading.RLock()

    def session(self, user: str) -> Session:
        with self._lock:
            return self.sessions.setdefault(user, Session(user))

    # ------------------------------------------------------------------ entry point
    def handle_text(self, user: str, text: str) -> str:
        with self._lock:
            s = self.session(user)
            reply = self._handle(s, text)
            if not s.greeted:
                s.greeted = True
                reply = f"{self.config.demo_notice} {reply}"
            return reply

    def _handle(self, s: Session, text: str) -> str:
        note = ""
        if s.state is State.WANTS_GUIDE and text.strip() == self.config.store_code:
            if self.robot_owner not in (None, s.user):
                return "The guide robot is helping another shopper right now. Please try again in a few minutes."
            self.robot_owner = s.user
            s.state, s.arrived, s.boxes, s.frame = State.GUIDING, False, [], None
            note = (f"[system: the shopper checked in with the correct entrance code. Now guide them: call "
                    f"navigate_to with waypoint {s.product.aisle}.]")
        s.searched, s.grounded_aisles = False, set()
        user_msg = {"role": "user", "content": f"{text}\n{note}" if note else text}
        messages = [self._system(s), *s.history, user_msg]
        reply = self._loop(s, messages)
        s.history += [{"role": "user", "content": text}, {"role": "assistant", "content": reply}]
        s.history = s.history[-2 * self.config.history_turns:]
        return reply

    def _system(self, s: Session) -> dict:
        p = s.product
        lines = [RULES, "", f"Conversation state: {s.state.value}."]
        if p:
            lines.append(f"Current product: {p.name}, {p.aisle}, {p.shelf} shelf, qty {p.qty}.")
        if s.state is State.WANTS_GUIDE:
            lines.append("Waiting for the shopper to send the entrance code.")
        if s.state is State.GUIDING:
            lines.append("The robot has arrived at the aisle." if s.arrived else "The robot has not arrived yet.")
            lines.append("If the shopper says they are done, or asks about another product, call finish_guidance "
                         "first; you can only look up products after that.")
        return {"role": "system", "content": "\n".join(lines)}

    def _ungrounded(self, s: Session, text: str) -> bool:
        """True if the reply states an aisle or stock fact that no tool gave (seen live: 'Aisle 4')."""
        known = set(s.grounded_aisles) | ({s.product.aisle} if s.product else set())
        mentioned = {f"AISLE_{n}" for n in _AISLE_RE.findall(text)}
        if mentioned - known:
            return True
        return bool(_STOCK_RE.search(text)) and not s.searched and s.product is None

    def _loop(self, s: Session, messages: list[dict]) -> str:
        corrected = False
        for _ in range(self.config.max_steps):
            try:
                reply = self._chat(messages, tools_for(s.state, self.config.waypoints))
            except LLMError as e:
                log.warning("llm failed: %s", e)
                return STAFF_FALLBACK
            if not reply.tool_calls:
                text = reply.content or EMPTY_REPLY
                if not self._ungrounded(s, text):
                    return text
                log.warning("ungrounded reply blocked: %r", text)
                if corrected:
                    return UNSURE_REPLY
                corrected = True
                messages += [reply.message, {"role": "user", "content": CORRECTION}]
                continue
            messages.append(reply.message)
            for tc in reply.tool_calls:
                result = self.execute(s, tc.name, tc.args)
                log.info("tool %s(%s) -> %s", tc.name, json.dumps(tc.args), json.dumps(result))
                messages.append({"role": "tool", "tool_call_id": tc.id, "content": json.dumps(result)})
            messages[0] = self._system(s)
        log.warning("max_steps reached")
        return STAFF_FALLBACK

    def _chat(self, messages: list[dict], tools: list[dict]):
        limit = self.config.max_tokens
        try:
            return self.llm.chat(messages, tools=tools, timeout_s=self.config.timeout_s, max_tokens=limit)
        except LLMTruncated:
            limit *= 2
        except LLMError as e:
            if not e.retryable:
                raise
        return self.llm.chat(messages, tools=tools, timeout_s=self.config.timeout_s, max_tokens=limit)

    # ------------------------------------------------------------------ tools (code owns the rules)
    def execute(self, s: Session, name: str, args: dict) -> dict:
        allowed = {t["name"]: t for t in tools_for(s.state, self.config.waypoints)}
        if name not in allowed:
            return {"error": f"{name} is not available now (state {s.state.value})"}
        schema = allowed[name]["input_schema"]
        props = schema["properties"]
        if not props:
            args = {}  # no-argument tool: the model sometimes adds stray args (seen live)
        if not isinstance(args, dict) or set(args) - set(props) or set(schema["required"]) - set(args):
            return {"error": f"bad arguments for {name}: expected {sorted(props) or 'no arguments'}"}
        try:
            return getattr(self, f"_t_{name}")(s, **args)
        except Exception as e:  # noqa: BLE001 - a tool failure is reported to the model, never raised
            log.exception("tool %s failed", name)
            return {"error": f"{name} failed: {type(e).__name__}"}

    def _t_search_product(self, s: Session, query) -> dict:
        if not isinstance(query, str) or not query.strip() or len(query) > MAX_QUERY_LEN:
            return {"error": "query must be a short product name"}
        found = self.catalog.search(query.strip())
        s.searched = True
        s.grounded_aisles |= {p.aisle for p in found}
        if not found:
            return {"found": False, "message": "not in the store database"}
        if len(found) > 1:
            return {"found": True, "ambiguous": True, "matches": [p.summary() for p in found]}
        p = found[0]
        if p.in_stock and s.state is State.WANTS_GUIDE and s.product and s.product.id == p.id:
            pass  # same product again while waiting for the code: keep waiting
        elif p.in_stock:
            s.state, s.product = State.OFFERED, p
        else:
            s.state, s.product = State.IDLE, None
        return {"found": True, "ambiguous": False, "matches": [p.summary()]}

    def _t_record_decision(self, s: Session, wants_guide) -> dict:
        if not isinstance(wants_guide, bool) or s.product is None:
            return {"error": "bad decision"}
        if wants_guide:
            s.state = State.WANTS_GUIDE
            return {"ok": True, "next": "Ask the shopper to send the entrance code posted at the store entrance."}
        s.state, s.product = State.IDLE, None
        return {"ok": True}

    def _t_navigate_to(self, s: Session, waypoint) -> dict:
        targets = {s.product.aisle, self.config.home} if s.product else {self.config.home}
        if waypoint not in self.config.waypoints or waypoint not in targets:
            return {"error": f"can only go to {sorted(targets)}"}
        status = self.robot.go(waypoint)
        s.arrived = status == "arrived" and waypoint == s.product.aisle
        s.boxes, s.frame = [], None
        return {"status": status}

    def _t_find_in_view(self, s: Session) -> dict:
        if not s.arrived:
            return {"error": "the robot has not arrived at the product's aisle"}
        frame = self.robot.frame()
        if not frame:
            return {"error": "no camera frame"}
        query = s.product.grounding_query or s.product.name
        boxes = sorted(self.locator.locate(frame, query), key=lambda b: -b.score)[:3]
        s.frame, s.boxes = frame, boxes
        if not boxes:
            return {"found": 0, "message": "nothing matching the product in view"}
        return {"found": len(boxes), "boxes": [
            {"box": i, "position": position_text(b), "score": round(b.score, 2)} for i, b in enumerate(boxes)]}

    def _t_verify_crop(self, s: Session, box) -> dict:
        if not s.arrived or not s.boxes or s.frame is None:
            return {"error": "call find_in_view first"}
        if isinstance(box, str) and box.strip().isdigit():
            box = int(box)
        if not isinstance(box, int) or isinstance(box, bool) or not 0 <= box < len(s.boxes):
            return {"error": f"box must be 0..{len(s.boxes) - 1}"}
        b = s.boxes[box]
        v = self.verifier.verify(s.frame, b, s.product)
        return {"match": v.match, "confidence": v.confidence, "reason": v.reason, "position": position_text(b)}

    def _t_finish_guidance(self, s: Session) -> dict:
        status = self.robot.go(self.config.home)
        s.state, s.product, s.arrived, s.boxes, s.frame = State.IDLE, None, False, [], None
        if self.robot_owner == s.user:
            self.robot_owner = None
        return {"status": status}
