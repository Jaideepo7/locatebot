"""Product enrichment (run once per product, offline): reference photo/notes -> strict attributes JSON."""
from __future__ import annotations

from locatebot.agent.llm import image_part
from locatebot.store._json import extract_object

ATTRIBUTE_KEYS = ("category", "major_color", "shape", "text_on_label", "distinguishing", "grounding_query")
MAX_LEN = {"grounding_query": 120}
DEFAULT_MAX_LEN = 160
MAY_BE_EMPTY = {"text_on_label"}

PROMPT = """You describe store products so a camera-based detector can find them on a shelf.
Product name: {name}
{notes}
Reply with ONLY a JSON object with exactly these string keys:
"category", "major_color", "shape", "text_on_label" (empty string if no readable text),
"distinguishing" (what tells it apart from similar products),
"grounding_query" (one short visual description for a detector, under 100 characters, e.g.
"a white cylindrical pack of paper towels with a blue stripe")."""


def parse_attributes(text: str) -> dict:
    d = extract_object(text)
    missing = [k for k in ATTRIBUTE_KEYS if k not in d]
    extra = [k for k in d if k not in ATTRIBUTE_KEYS]
    if missing or extra:
        raise ValueError(f"wrong keys: missing {missing}, unexpected {extra}")
    out = {}
    for k in ATTRIBUTE_KEYS:
        v = d[k]
        if not isinstance(v, str):
            raise ValueError(f"{k} must be a string")
        v = " ".join(v.split())
        if not v and k not in MAY_BE_EMPTY:
            raise ValueError(f"{k} is empty")
        if len(v) > MAX_LEN.get(k, DEFAULT_MAX_LEN):
            raise ValueError(f"{k} is too long")
        out[k] = v
    return out


def enrich_product(llm, name: str, *, image: bytes | None = None, mime: str = "image/jpeg",
                   notes: str = "", timeout_s: float = 60.0, attempts: int = 2) -> dict:
    """Ask the model for attributes; retry once on invalid JSON. Photo must show no people."""
    if image is None and not notes.strip():
        raise ValueError("need a reference photo or hand-written notes")
    text = PROMPT.format(name=name, notes=f"Notes from staff: {notes.strip()}" if notes.strip() else "")
    content: list[dict] = [{"type": "text", "text": text}]
    if image is not None:
        content.append(image_part(image, mime))
    last: Exception | None = None
    for _ in range(attempts):
        reply = llm.chat([{"role": "user", "content": content}], timeout_s=timeout_s, max_tokens=1024)
        try:
            return parse_attributes(reply.content)
        except ValueError as e:
            last = e
    raise ValueError(f"model did not return valid attributes: {last}")
