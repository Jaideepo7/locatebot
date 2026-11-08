"""Crop verification: ask Nemotron VL whether a LocateAnything box really is the product."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from locatebot.agent.llm import image_part
from locatebot.store._json import extract_object
from locatebot.store.catalog import Product
from locatebot.store.robot import Box, crop_jpeg, position_text

CONFIDENCE = ("high", "medium", "low")


@dataclass(frozen=True)
class Verdict:
    match: bool
    confidence: str
    reason: str


def parse_verdict(text: str) -> Verdict:
    d = extract_object(text)
    match, conf, reason = d.get("match"), d.get("confidence"), d.get("reason", "")
    if not isinstance(match, bool):
        raise ValueError("'match' must be true or false")
    if conf not in CONFIDENCE:
        raise ValueError(f"'confidence' must be one of {CONFIDENCE}")
    if not isinstance(reason, str):
        raise ValueError("'reason' must be a string")
    return Verdict(match, conf, reason.strip()[:200])


def describe(product: Product) -> str:
    lines = [f"Product: {product.name}"]
    if product.grounding_query:
        lines.append(f"Looks like: {product.grounding_query}")
    for k, v in product.attributes.items():
        if v and k != "grounding_query":
            lines.append(f"{k.replace('_', ' ')}: {v}")
    return "\n".join(lines)


INSTRUCTIONS = (
    'Reply with ONLY a JSON object: {"match": true or false, "confidence": "high" or "medium" or "low", '
    '"reason": "<one short sentence>"}. Say match false if the item is a different product, a different '
    "brand or variant, or you cannot tell."
)


class NemotronVerifier:
    """``llm`` needs ``chat(messages, tools=None, timeout_s=..., max_tokens=...)`` (e.g. NemotronClient).

    ``cropper(jpeg, box) -> bytes | None``; with no crop the whole frame is sent and the box is
    described in words. Raises ``LLMError`` on API failure, ``ValueError`` on an unusable reply.
    """

    def __init__(self, llm, *, timeout_s: float = 30.0,
                 cropper: Callable[[bytes, Box], bytes | None] = crop_jpeg) -> None:
        self.llm = llm
        self.timeout_s = timeout_s
        self.cropper = cropper

    def verify(self, frame: bytes, box: Box, product: Product) -> Verdict:
        crop = self.cropper(frame, box)
        if crop is not None:
            intro = "This is a cropped photo of one item on a store shelf. Is it the product described below?"
            image = crop
        else:
            intro = (f"This is a photo of a store shelf. Look only at the item in the {position_text(box)} "
                     f"(x {box.x1:.0%}-{box.x2:.0%}, y {box.y1:.0%}-{box.y2:.0%} of the image). "
                     "Is it the product described below?")
            image = frame
        content = [{"type": "text", "text": f"{intro}\n\n{describe(product)}\n\n{INSTRUCTIONS}"}, image_part(image)]
        reply = self.llm.chat([{"role": "user", "content": content}], timeout_s=self.timeout_s, max_tokens=512)
        return parse_verdict(reply.content)
