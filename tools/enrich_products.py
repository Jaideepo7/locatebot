#!/usr/bin/env python3
"""Enrich products once, offline: reference photo and/or notes -> attributes + grounding_query.

Photos: <photos>/<product id>.jpg or .png. Objects only: NO people or faces (sent to NVIDIA's
hosted API, which logs use). Writes a new catalog JSON; SpacetimeDB upload comes later.

  python tools/enrich_products.py --catalog data/mock_store.json --photos photos/ --out data/mock_store.enriched.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from locatebot.agent.llm import LLMError, NemotronClient  # noqa: E402
from locatebot.store.enrich import enrich_product  # noqa: E402
from tools.store_agent import load_env_file  # noqa: E402

MIME = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--catalog", type=Path, default=ROOT / "data" / "mock_store.json")
    ap.add_argument("--photos", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--only", nargs="*", help="product ids to enrich (default: all with a photo or notes)")
    ap.add_argument("--env-file", type=Path, default=ROOT / ".env")
    args = ap.parse_args(argv)
    load_env_file(args.env_file)
    data = json.loads(args.catalog.read_text())
    llm = NemotronClient()
    failed = 0
    for p in data["products"]:
        if args.only and p["id"] not in args.only:
            continue
        photo = next((args.photos / f"{p['id']}{ext}" for ext in MIME
                      if args.photos and (args.photos / f"{p['id']}{ext}").is_file()), None)
        notes = p.get("notes", "")
        if photo is None and not notes:
            continue
        try:
            attrs = enrich_product(llm, p["name"], image=photo.read_bytes() if photo else None,
                                   mime=MIME[photo.suffix.lower()] if photo else "image/jpeg", notes=notes)
        except (LLMError, ValueError) as e:
            failed += 1
            print(f"FAIL {p['id']}: {e}")
            continue
        p["attributes"] = {k: v for k, v in attrs.items() if k != "grounding_query"}
        p["grounding_query"] = attrs["grounding_query"]
        print(f"ok   {p['id']}: {attrs['grounding_query']}")
    args.out.write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {args.out}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
