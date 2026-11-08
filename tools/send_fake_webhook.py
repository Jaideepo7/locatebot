#!/usr/bin/env python3
"""Send a signed, Photon-shaped Telegram message to the local remote-control service (tools/remote_run.py) (no Photon or Telegram needed).

Signs exactly like Photon (Standard Webhooks) with PHOTON_WEBHOOK_SECRET; uses PHOTON_PROJECT_ID.
Standard library only.

Examples:
  python tools/send_fake_webhook.py --text "/find blue toolbox"
  python tools/send_fake_webhook.py --text "/find blue toolbox" --repeat 3        # duplicate deliveries
  python tools/send_fake_webhook.py --text "/find red cup" --customer b          # a second customer
  python tools/send_fake_webhook.py --text "/find blue toolbox" --bad-signature  # forged request
"""
from __future__ import annotations

import argparse
import json
import os
import secrets
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from locatebot.remote.photon import sign_payload  # noqa: E402

FIXTURE = ROOT / "tests" / "fixtures" / "photon_message_received.json"
CUSTOMERS = {"a": ("pho_spc_01jaaaaaaaaaaaaaaaaaaaaaaa", "pho_usr_01jaaaaaaaaaaaaaaaaaaaaaaa", "customer_a"),
             "b": ("pho_spc_01jbbbbbbbbbbbbbbbbbbbbbbb", "pho_usr_01jbbbbbbbbbbbbbbbbbbbbbbb", "customer_b")}
_CROCKFORD = "0123456789abcdefghjkmnpqrstvwxyz"


def new_event_id() -> str:
    return "pho_evt_" + secrets.choice("01234567") + "".join(secrets.choice(_CROCKFORD) for _ in range(25))


def build_event(text: str | None, project_id: str, customer: str = "a", event_id: str | None = None) -> dict:
    ev = json.loads(FIXTURE.read_text())
    space, user, handle = CUSTOMERS[customer]
    obj = ev["data"]["object"]
    ev["id"] = event_id or new_event_id()
    ev["data"]["projectId"] = obj["projectId"] = project_id
    obj["space"]["id"], obj["sentBy"]["id"], obj["sentBy"]["handle"] = space, user, handle
    obj["content"] = {"type": "text", "format": "plain", "text": text} if text is not None else \
        {"type": "attachment", "attachmentId": "pho_att_01jaaaaaaaaaaaaaaaaaaaaaaa"}
    return ev


def build_request(ev: dict, secret: str, now: float | None = None, bad_signature: bool = False):
    body = json.dumps(ev).encode()
    ts = int(time.time() if now is None else now)
    sig = "v1," + "A" * 44 if bad_signature else sign_payload(secret, ev["id"], ts, body)
    headers = {"Content-Type": "application/json", "webhook-id": ev["id"], "webhook-timestamp": str(ts),
               "webhook-signature": sig}
    return headers, body


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--url", default="http://127.0.0.1:8070/api/photon/webhook")
    ap.add_argument("--text", help="message text; omit to send a photo-like non-text message")
    ap.add_argument("--customer", choices=sorted(CUSTOMERS), default="a")
    ap.add_argument("--repeat", type=int, default=1, help="deliver the same event N times (Photon retry)")
    ap.add_argument("--event-id", help="reuse a specific event id")
    ap.add_argument("--bad-signature", action="store_true")
    args = ap.parse_args(argv)
    secret, project = os.environ.get("PHOTON_WEBHOOK_SECRET", ""), os.environ.get("PHOTON_PROJECT_ID", "")
    if not secret or not project:
        print("set PHOTON_WEBHOOK_SECRET and PHOTON_PROJECT_ID (same values as tools/remote_run.py)", file=sys.stderr)
        return 2
    ev = build_event(args.text, project, args.customer, args.event_id)
    for i in range(args.repeat):
        headers, body = build_request(ev, secret, bad_signature=args.bad_signature)
        req = urllib.request.Request(args.url, data=body, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                status = r.status
        except urllib.error.HTTPError as e:
            status = e.code
        print(f"delivery {i + 1}: event {ev['id']} -> HTTP {status}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
