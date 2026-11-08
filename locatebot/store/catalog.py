"""Product catalog. ``MemoryCatalog`` is the mock store; SpacetimeDB can implement ``Catalog`` later."""
from __future__ import annotations

import difflib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True)
class Product:
    id: str
    name: str
    aisle: str            # waypoint name of the aisle, e.g. "AISLE_2"
    shelf: str            # "top" / "middle" / "bottom"
    qty: int
    aliases: tuple[str, ...] = ()
    attributes: dict = field(default_factory=dict, compare=False, hash=False)
    grounding_query: str = ""

    @property
    def in_stock(self) -> bool:
        return self.qty > 0

    def summary(self) -> dict:
        return {"name": self.name, "in_stock": self.in_stock, "qty": self.qty,
                "aisle": self.aisle, "shelf": self.shelf}


class Catalog(Protocol):
    def search(self, query: str, limit: int = 3) -> list[Product]: ...


def _words(text: str) -> list[str]:
    out = []
    for w in re.findall(r"[a-z0-9]+", text.lower()):
        if len(w) > 3 and w.endswith("s") and not w.endswith("ss"):
            w = w[:-1]  # crude singular: "towels" -> "towel"
        out.append(w)
    return out


class MemoryCatalog:
    """In-memory catalog with forgiving search: exact name/alias, all words present, or fuzzy."""

    FUZZY_CUTOFF = 0.75

    def __init__(self, products: list[Product]) -> None:
        self.products = list(products)

    @classmethod
    def from_json(cls, path: str | Path) -> "MemoryCatalog":
        data = json.loads(Path(path).read_text())
        items = data["products"] if isinstance(data, dict) else data
        return cls([Product(id=str(p["id"]), name=p["name"], aisle=p["aisle"], shelf=p.get("shelf", ""),
                            qty=int(p.get("qty", 0)), aliases=tuple(p.get("aliases", ())),
                            attributes=dict(p.get("attributes") or {}),
                            grounding_query=p.get("grounding_query") or "") for p in items])

    def search(self, query: str, limit: int = 3) -> list[Product]:
        q = " ".join(_words(query))
        if not q:
            return []
        scored: list[tuple[float, int, Product]] = []
        for i, p in enumerate(self.products):
            best = 0.0
            for name in (p.name, *p.aliases):
                n = " ".join(_words(name))
                if n == q:
                    s = 1.0
                elif set(q.split()) <= set(n.split()):
                    s = 0.9
                else:
                    s = difflib.SequenceMatcher(None, q, n).ratio()
                    s = s if s >= self.FUZZY_CUTOFF else 0.0
                best = max(best, s)
            if best > 0:
                scored.append((best, i, p))
        scored.sort(key=lambda t: (-t[0], t[1]))
        if scored and scored[0][0] == 1.0:
            scored = [t for t in scored if t[0] == 1.0]
        return [p for _, _, p in scored[:limit]]
