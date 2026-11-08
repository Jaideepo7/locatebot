#!/usr/bin/env python3
"""locate_server.py - keep LocateAnything loaded on the Orin GPU and answer over HTTP.

  POST /locate?item=orange%20box[,red%20bucket]&mode=hybrid     body = JPEG bytes
       -> {"w","h","results":[{"item","ms","boxes":[[x1,y1,x2,y2],...],"raw_n"}], "total_ms"}
  GET  /health -> {"ok", "load_s", "calls", "avg_ms", "busy"}

The model loads once (~57 s from the SD card) and stays in GPU memory, so each call costs only
the inference (~2.1 s per item at 640 px on the Orin Nano). One item per prompt (multi-item
prompts were unreliable in the team's tests). Send ~640 px wide JPEGs: 960 px costs ~5 s.
"""
import json, os, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = os.path.expanduser("~/locatebot")
sys.path.insert(0, ROOT)
from locatebot.vision.engine import LocateEngine, find_library          # noqa: E402
from locatebot.vision.postprocess import build_prompt, clean_detections, jpeg_size  # noqa: E402

PORT = int(os.environ.get("LOCATE_PORT", "8091"))
t0 = time.time()
ENGINE = LocateEngine(os.path.join(ROOT, "models", "locate-anything-q4_k.gguf"), find_library(ROOT))
LOAD_S = round(time.time() - t0, 1)
STATS = {"calls": 0, "ms": 0.0, "busy": False}
LOCK = threading.Lock()                     # one inference at a time (GPU memory)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        if urlparse(self.path).path == "/health":
            avg = STATS["ms"] / STATS["calls"] if STATS["calls"] else None
            return self._json(200, {"ok": True, "load_s": LOAD_S, "calls": STATS["calls"],
                                    "avg_ms": None if avg is None else round(avg), "busy": STATS["busy"]})
        self._json(404, {"ok": False, "error": "GET /health or POST /locate"})

    def do_POST(self):
        u = urlparse(self.path)
        if u.path != "/locate":
            return self._json(404, {"ok": False, "error": "unknown path"})
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        items = [s.strip() for s in q.get("item", "").split(",") if s.strip()][:6]
        mode = q.get("mode", "hybrid")
        n = int(self.headers.get("Content-Length") or 0)
        if not items or not 0 < n <= 4_000_000:
            return self._json(400, {"ok": False, "error": "need ?item= and a JPEG body (<4 MB)"})
        jpeg = self.rfile.read(n)
        try:
            w, h = jpeg_size(jpeg)
        except Exception as e:
            return self._json(400, {"ok": False, "error": f"not a JPEG: {e}"})
        out, t_all = [], time.time()
        with LOCK:
            STATS["busy"] = True
            try:
                for item in items:
                    t = time.time()
                    dets = ENGINE.locate(jpeg, build_prompt(item), mode)
                    clean = clean_detections(dets, w, h)
                    ms = (time.time() - t) * 1000
                    STATS["calls"] += 1; STATS["ms"] += ms
                    out.append({"item": item, "ms": round(ms), "raw_n": len(dets),
                                "boxes": [[round(v, 1) for v in d.box] for d in clean]})
            except Exception as e:
                return self._json(500, {"ok": False, "error": repr(e)})
            finally:
                STATS["busy"] = False
        self._json(200, {"ok": True, "w": w, "h": h, "results": out,
                         "total_ms": round((time.time() - t_all) * 1000)})


if __name__ == "__main__":
    print(f"model loaded in {LOAD_S} s; serving on :{PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
