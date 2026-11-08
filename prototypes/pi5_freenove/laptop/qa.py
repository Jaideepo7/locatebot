#!/usr/bin/env python3
"""qa.py - the LLM's side of the LocateBot app inbox (questions typed into the phone app).

  python3 qa.py watch                    block until a question is waiting, print it as JSON, exit
                                         (run in the background: the exit is the notification)
  python3 qa.py list [pending|working|answered|all]
  python3 qa.py claim ID                 mark it "Claude is on it"
  python3 qa.py say ID "text"            add a live progress line under the question
  python3 qa.py snap [NAME]              save a photo from the car camera locally, print its path
  python3 qa.py answer ID "text" [PHOTO ...]   PHOTO = local .jpg (uploaded first) or a name already on the car
"""
import json, os, sys, time, urllib.request

CAR = os.environ.get("CAR_URL", "http://raspberrypi.local:8000")
OUT = os.path.join("finds", "qa")


def req(path, body=None, ctype="application/json", timeout=20):
    data = None
    if body is not None:
        data = body if isinstance(body, (bytes, bytearray)) else json.dumps(body).encode()
    r = urllib.request.Request(CAR + path, data=data, headers={"Content-Type": ctype} if data else {})
    raw = urllib.request.urlopen(r, timeout=timeout).read()
    return json.loads(raw) if raw[:1] in (b"{", b"[") else raw


def upload(path, name=None):
    name = name or "qa_" + os.path.basename(path)
    with open(path, "rb") as fh:
        return req("/upload?name=" + urllib.request.quote(name), fh.read(), "image/jpeg")["name"]


def main(a):
    cmd = a[0] if a else "list"
    if cmd == "watch":
        down_since = None
        while True:
            try:
                items = req("/inbox?status=pending", timeout=5)
                down_since = None
                if items:
                    print(json.dumps(items, indent=1)); return
            except Exception as e:
                if down_since is None:
                    down_since = time.time()
                    print(f"car unreachable ({e.__class__.__name__}), still watching...", file=sys.stderr, flush=True)
            time.sleep(1.0)
    if cmd == "list":
        print(json.dumps(req("/inbox?status=" + (a[1] if len(a) > 1 else "all")), indent=1)); return
    if cmd == "claim":
        print(req("/claim", {"id": int(a[1])})); return
    if cmd == "say":
        print(req("/progress", {"id": int(a[1]), "text": a[2]})); return
    if cmd == "snap":
        os.makedirs(OUT, exist_ok=True)
        path = os.path.join(OUT, (a[1] if len(a) > 1 else time.strftime("snap_%H%M%S")) + ".jpg")
        with open(path, "wb") as fh:
            fh.write(req("/snap", timeout=30))
        print(os.path.abspath(path)); return
    if cmd == "answer":
        names = []
        for p in a[3:]:
            names.append(upload(p) if os.path.isfile(p) else os.path.basename(p))
        print(req("/answer", {"id": int(a[1]), "text": a[2], "photos": names})); return
    sys.exit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
