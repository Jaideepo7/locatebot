"""Pull one JSON object out of a model reply (tolerates ```json fences and chatter around it)."""
from __future__ import annotations

import json


def extract_object(text: str) -> dict:
    s = (text or "").strip()
    start, end = s.find("{"), s.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("no JSON object in reply")
    try:
        data = json.loads(s[start:end + 1])
    except ValueError:
        raise ValueError("reply is not valid JSON") from None
    if not isinstance(data, dict):
        raise ValueError("reply is not a JSON object")
    return data
