"""Local manual-drive page for the Pi, stdlib only (no CDN, no external assets).

Endpoints
- ``GET /``         self-contained HTML page (E-STOP, hold-to-drive, keys, joystick, speed slider, status)
- ``GET /status``   ``PiController.status()`` as JSON
- ``POST /drive``   ``{"throttle": t, "steering": s}`` numbers, clamped to [-1, 1]; non-finite rejected
- ``POST /estop``   engage the latched e-stop (any body, or none)
- ``POST /release`` ``{"confirm": true}``: local e-stop release; clears held commands
- ``POST /speed``   ``{"limit": x}`` with x in (0, 1]: manual throttle scale

``/drive``, ``/release`` and ``/speed`` require ``Content-Type: application/json``. Browsers
cannot send that cross-origin without a CORS preflight, which this server never approves, so
another web page open on the same laptop can't drive the car or release the e-stop.

The default bind host is ``127.0.0.1``. On the car, bind the Pi's own address on the
private dashboard network (``--host``). Anyone who can reach the port can drive the car:
there is no login, so keep it on the team's own router/hotspot.
"""
from __future__ import annotations

import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .control import PiController

MAX_BODY_BYTES = 1024


class BadRequest(Exception):
    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code


def _reject_constant(name: str):
    raise ValueError(f"{name} is not allowed")


def _number(body: dict, key: str) -> float:
    if key not in body:
        raise BadRequest(400, f"missing {key!r}")
    v = body[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise BadRequest(400, f"{key} must be a number")
    try:
        f = float(v)
    except OverflowError:
        raise BadRequest(400, f"{key} out of range") from None
    if not math.isfinite(f):
        raise BadRequest(400, f"{key} must be finite")
    return f


class PiHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, controller: PiController):
        self.controller = controller
        super().__init__(address, _Handler)


def make_server(controller: PiController, host: str = "127.0.0.1", port: int = 8080) -> PiHTTPServer:
    """Create (not start) the server; call ``serve_forever()`` in a thread, ``shutdown()`` to stop."""
    return PiHTTPServer((host, port), controller)


class _Handler(BaseHTTPRequestHandler):
    server: PiHTTPServer
    protocol_version = "HTTP/1.1"
    server_version = "locatebot-pi"

    def log_message(self, format, *args):  # noqa: A002 - quiet: 10 requests/s per client
        pass

    # --- helpers -------------------------------------------------------------------

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if ctype.startswith("text/html"):
            self.send_header("Content-Security-Policy",
                             "default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
                             "connect-src 'self'; img-src 'self' data:")
        if self.close_connection:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj) -> None:
        self._send(code, json.dumps(obj, allow_nan=False, default=str).encode(), "application/json")

    def _status(self) -> dict:
        st = self.server.controller.status()
        # JSON has no NaN/inf: report those as null
        return {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in st.items()}

    def _read_body(self) -> bytes:
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self.close_connection = True
            raise BadRequest(400, "bad Content-Length") from None
        if n < 0:
            self.close_connection = True
            raise BadRequest(400, "bad Content-Length")
        if n > MAX_BODY_BYTES:
            self.close_connection = True  # don't read it; drop the connection after replying
            raise BadRequest(413, "body too large")
        return self.rfile.read(n) if n else b""

    def _json_body(self, raw: bytes, require_json_type: bool = True) -> dict:
        if require_json_type:
            ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            if ctype != "application/json":
                raise BadRequest(415, "Content-Type must be application/json")
        try:
            body = json.loads(raw, parse_constant=_reject_constant)
        except (ValueError, UnicodeDecodeError) as e:
            raise BadRequest(400, f"invalid JSON: {e}") from None
        if not isinstance(body, dict):
            raise BadRequest(400, "body must be a JSON object")
        return body

    # --- routes --------------------------------------------------------------------

    def do_GET(self):  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        elif path == "/status":
            self._json(200, self._status())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):  # noqa: N802
        path = self.path.split("?", 1)[0]
        ctl = self.server.controller
        try:
            raw = self._read_body()
            if path == "/estop":
                ctl.engage_estop("dashboard")  # always safe: no content checks
            elif path == "/drive":
                try:
                    body = self._json_body(raw)
                    t, s = _number(body, "throttle"), _number(body, "steering")
                except BadRequest:
                    try:
                        ctl.submit_manual(float("nan"), 0.0)  # replace held command with a stop
                    except ValueError:
                        pass
                    raise
                ctl.submit_manual(max(-1.0, min(1.0, t)), max(-1.0, min(1.0, s)))
            elif path == "/release":
                body = self._json_body(raw)
                if body.get("confirm") is not True:
                    raise BadRequest(400, 'release needs {"confirm": true}')
                ctl.release_estop_local()
            elif path == "/speed":
                limit = _number(self._json_body(raw), "limit")
                try:
                    ctl.set_speed_limit(limit)
                except ValueError as e:
                    raise BadRequest(400, str(e)) from None
            else:
                raise BadRequest(404, "not found")
        except BadRequest as e:
            self._json(e.code, {"error": str(e)})
            return
        except ValueError as e:
            self._json(400, {"error": str(e)})
            return
        self._json(200, self._status())


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>locatebot drive</title>
<style>
  :root { color-scheme: dark; --bg:#111; --fg:#eee; --muted:#888; --panel:#1c1c1c; --red:#d00; --ok:#2a2; --warn:#c80; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg); font:16px system-ui, sans-serif;
         -webkit-user-select:none; user-select:none; touch-action:none; }
  #estop { position:fixed; top:0; left:0; right:0; z-index:10; height:84px; border:0; background:var(--red);
           color:#fff; font-size:40px; font-weight:800; letter-spacing:2px; cursor:pointer; }
  #estop:active { background:#900; }
  main { padding:96px 16px 16px; display:grid; gap:16px; grid-template-columns:1fr; max-width:900px; margin:0 auto; }
  @media (min-width:760px) { main { grid-template-columns:1fr 1fr; } }
  section { background:var(--panel); border-radius:10px; padding:14px; }
  h2 { margin:0 0 10px; font-size:14px; color:var(--muted); text-transform:uppercase; letter-spacing:1px; }
  .pad { display:grid; grid-template-columns:repeat(3, 1fr); gap:8px; }
  .pad button { height:72px; font-size:28px; border:0; border-radius:8px; background:#333; color:var(--fg); }
  .pad button.on { background:#06c; }
  #joy { position:relative; width:220px; height:220px; margin:8px auto 0; border-radius:50%; background:#262626;
         border:2px solid #444; touch-action:none; }
  #knob { position:absolute; width:70px; height:70px; left:75px; top:75px; border-radius:50%; background:#555; }
  #joy.on #knob { background:#06c; }
  table { width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }
  td { padding:3px 0; } td:last-child { text-align:right; }
  #banner { font-weight:700; padding:8px; border-radius:6px; text-align:center; margin-bottom:8px; }
  .ok { background:var(--ok); } .stop { background:var(--warn); } .estop { background:var(--red); } .lost { background:#555; }
  #release { width:100%; height:48px; border:0; border-radius:8px; background:#444; color:var(--fg); font-size:16px; margin-top:10px; }
  input[type=range] { width:100%; }
  .hint { color:var(--muted); font-size:13px; margin-top:8px; }
</style>
</head>
<body>
<button id="estop" title="Space">E-STOP</button>
<main>
  <section>
    <h2>Drive (hold)</h2>
    <div class="pad">
      <button data-t="1" data-s="1">&#8598;</button>
      <button data-t="1" data-s="0">&#8593;</button>
      <button data-t="1" data-s="-1">&#8599;</button>
      <button data-t="0" data-s="1">&#8592;</button>
      <button data-t="-1" data-s="0">&#8595;</button>
      <button data-t="0" data-s="-1">&#8594;</button>
    </div>
    <div id="joy"><div id="knob"></div></div>
    <div class="hint">Hold W/A/S/D or arrow keys, a button, or drag the joystick. Release to stop.
      Space = E-STOP. Reverse is blocked by the safety stop.</div>
  </section>
  <section>
    <h2>Status</h2>
    <div id="banner" class="lost">connecting...</div>
    <table id="st"></table>
    <h2 style="margin-top:14px">Speed limit <span id="lim"></span></h2>
    <input id="speed" type="range" min="5" max="100" step="5" value="30">
    <button id="release">Release E-stop (local)</button>
  </section>
</main>
<script>
"use strict";
const SEND_MS = 100, POLL_MS = 300;
const keys = new Set();
let btn = null, joy = null, driving = false;

function post(path, body, keepalive) {
  return fetch(path, { method: "POST", headers: { "Content-Type": "application/json" },
                       body: JSON.stringify(body || {}), keepalive: !!keepalive, cache: "no-store" });
}
const KEYMAP = { w:[1,0], arrowup:[1,0], s:[-1,0], arrowdown:[-1,0], a:[0,1], arrowleft:[0,1], d:[0,-1], arrowright:[0,-1] };
const clamp = v => Math.max(-1, Math.min(1, v));

function current() {
  if (joy) return joy;
  let t = 0, s = 0, any = false;
  for (const k of keys) { t += KEYMAP[k][0]; s += KEYMAP[k][1]; any = true; }
  if (btn) { t += btn[0]; s += btn[1]; any = true; }
  return any ? { t: clamp(t), s: clamp(s) } : null;
}
function sendZero(keepalive) { post("/drive", { throttle: 0, steering: 0 }, keepalive).catch(() => {}); }
function tick() {
  const c = current();
  if (c) { driving = true; post("/drive", { throttle: c.t, steering: c.s }).catch(() => {}); }
  else if (driving) { driving = false; sendZero(); }
}
function releaseAll() {
  keys.clear(); btn = null; joy = null; resetKnob();
  document.querySelectorAll(".pad button.on").forEach(b => b.classList.remove("on"));
  driving = false; sendZero(true);
}
function estop() { releaseAll(); post("/estop", {}, true).then(poll).catch(() => {}); }

document.getElementById("estop").addEventListener("pointerdown", e => { e.preventDefault(); estop(); });
document.getElementById("estop").addEventListener("click", estop);

window.addEventListener("keydown", e => {
  const k = e.key.toLowerCase();
  if (k === " ") { e.preventDefault(); estop(); return; }
  if (k in KEYMAP) { e.preventDefault(); if (!keys.has(k)) { keys.add(k); tick(); } }
});
window.addEventListener("keyup", e => {
  const k = e.key.toLowerCase();
  if (keys.delete(k) && !current()) { driving = false; sendZero(); }
});
window.addEventListener("blur", releaseAll);
window.addEventListener("pagehide", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });

document.querySelectorAll(".pad button").forEach(b => {
  const t = Number(b.dataset.t), s = Number(b.dataset.s);
  b.addEventListener("pointerdown", e => { e.preventDefault(); b.setPointerCapture(e.pointerId);
    btn = [t, s]; b.classList.add("on"); tick(); });
  const up = () => { if (btn) { btn = null; b.classList.remove("on"); driving = false; sendZero(); } };
  ["pointerup", "pointercancel", "lostpointercapture"].forEach(ev => b.addEventListener(ev, up));
  b.addEventListener("contextmenu", e => e.preventDefault());
});

const pad = document.getElementById("joy"), knob = document.getElementById("knob");
function resetKnob() { knob.style.left = "75px"; knob.style.top = "75px"; pad.classList.remove("on"); }
function joyMove(e) {
  const r = pad.getBoundingClientRect(), R = r.width / 2;
  let dx = (e.clientX - r.left - R) / R, dy = (e.clientY - r.top - R) / R;
  const m = Math.hypot(dx, dy); if (m > 1) { dx /= m; dy /= m; }
  knob.style.left = (75 + dx * 75) + "px"; knob.style.top = (75 + dy * 75) + "px";
  const dead = v => Math.abs(v) < 0.12 ? 0 : v;
  joy = { t: clamp(dead(-dy)), s: clamp(dead(-dx)) };
}
pad.addEventListener("pointerdown", e => { e.preventDefault(); pad.setPointerCapture(e.pointerId);
  pad.classList.add("on"); joyMove(e); tick(); });
pad.addEventListener("pointermove", e => { if (pad.classList.contains("on")) joyMove(e); });
["pointerup", "pointercancel", "lostpointercapture"].forEach(ev => pad.addEventListener(ev, () => {
  if (pad.classList.contains("on")) { joy = null; resetKnob(); driving = false; sendZero(); } }));

const speed = document.getElementById("speed"), lim = document.getElementById("lim");
speed.addEventListener("input", () => { lim.textContent = speed.value + "%"; });
speed.addEventListener("change", () => { post("/speed", { limit: Number(speed.value) / 100 }).catch(() => {}); });

document.getElementById("release").addEventListener("click", () => {
  if (confirm("Release the E-stop? The car stays stopped until a new drive command.")) {
    post("/release", { confirm: true }).then(poll).catch(() => {});
  }
});

const banner = document.getElementById("banner"), table = document.getElementById("st");
const fmt = v => v === null || v === undefined ? "-" : (typeof v === "number" ? v.toFixed(2) : String(v));
let speedSynced = false;
function show(st) {
  let cls = "ok", text = "OK: " + st.active_source;
  if (st.estop) { cls = "estop"; text = "E-STOP (" + (st.estop_source || "?") + ")"; }
  else if (st.reason && st.reason !== "ok" && st.reason !== "slowed") { cls = "stop"; text = "STOPPED: " + st.reason; }
  else if (st.reason === "slowed") { text = "SLOWED: " + st.active_source; }
  banner.className = cls; banner.textContent = text;
  const rows = [["source", st.active_source], ["reason", st.reason], ["throttle out", st.throttle],
    ["steering out", st.steering], ["scan age s", st.scan_age_s], ["scans/s", st.scans_per_s],
    ["fault", st.fault], ["mission seq", st.mission_last_seq]];
  table.replaceChildren(...rows.map(([k, v]) => { const tr = document.createElement("tr");
    const a = document.createElement("td"), b = document.createElement("td");
    a.textContent = k; b.textContent = fmt(v); tr.append(a, b); return tr; }));
  if (!speedSynced) { speed.value = Math.round(st.speed_limit * 100); lim.textContent = speed.value + "%"; speedSynced = true; }
}
function poll() {
  return fetch("/status", { cache: "no-store" }).then(r => r.json()).then(show)
    .catch(() => { banner.className = "lost"; banner.textContent = "LINK LOST (car stops on its own)"; });
}
setInterval(tick, SEND_MS);
setInterval(poll, POLL_MS);
poll();
</script>
</body>
</html>
"""
