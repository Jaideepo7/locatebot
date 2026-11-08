"""Orin application: threads around ``OrinRuntime`` plus a local operator page.

Threads
- **control** (``control_hz``, default 10): ``runtime.step(now)``; stores the requested drive.
- **drive sender** (``send_hz``, default 20): sends the newest requested drive to ``drive_sink``.
  If the control thread has not produced a command for ``max_command_age_s`` (a slow replan,
  a crash), it sends zero instead. Measured on the Orin CPU: control step p50 45 ms, p95 88 ms,
  max 221 ms; the Pi stops the car if commands stop for 0.3 s.
- scans/frames arrive on the caller's threads (``on_scan`` / ``on_frame``), never blocked by
  planning.

Clocks: scans and frames from the Pi carry Pi monotonic stamps. ``PeerClock`` estimates the
Pi-minus-Orin offset from scan arrivals so the runtime runs on the Pi's timeline (item ages,
pose lookups at frame capture time). With a local LiDAR or the simulator the offset is ~0.

Operator page (stdlib HTTP, everything inline, no CDN):
- ``GET /``           page: live map with pose, path-free; items; skill; LLM proposal approve/reject;
                      manual skill buttons; STOP.
- ``GET /status``     ``OrinApp.status()`` JSON
- ``GET /map.png``    the occupancy map as a PNG (unknown grey, free white, occupied black)
- ``POST /skill``     ``{"name": "go_to", "args": {...}}`` operator skill call (validated)
- ``POST /stop``      operator STOP (pauses LLM decisions until ``/resume``)
- ``POST /approve``, ``POST /reject``, ``POST /resume``, ``POST /mode`` ``{"mode": "approve"|"auto"}``
- ``POST /save_map``  ``{"name": "room1"}`` -> ``maps/<name>.lbmap/.pgm/.yaml``
- ``POST /goal``      ``{"text": "find the bolt"}`` goal for the LLM (1-200 chars, 400 if invalid)
- ``POST /goal/clear`` clear the goal
- ``POST /relocalize`` ``{"x": 1.0, "y": 2.0, "theta_deg": 90}`` (or ``{}``: global search) -
                      saved-map mode (``--map``) only: re-initialize localization

POST bodies must be ``application/json`` (blocks cross-site form posts). Default bind host is
127.0.0.1; anyone who can reach the port can command the robot.
"""
from __future__ import annotations

import json
import logging
import math
import re
import struct
import threading
import time
import zlib
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable

import numpy as np

from locatebot.agent.tools import InvalidSkillCall, parse_tool_call
from locatebot.lidar.grid import FREE, OCCUPIED
from locatebot.lidar.types import Scan
from locatebot.mission.types import SkillCall, SkillName

from .runtime import OrinRuntime
from .skills import DriveOutput

log = logging.getLogger(__name__)

DriveSink = Callable[[float, float], None]


class PeerClock:
    """Maps local monotonic time onto a peer's monotonic timeline.

    offset ~ peer_stamp - local_receive_time. Network/processing latency only makes that
    difference smaller, so the running maximum (slowly decayed so a peer restart or drift can
    pull it back) is the best estimate."""

    def __init__(self, local: Callable[[], float] = time.monotonic, decay_per_s: float = 0.002):
        self._local = local
        self._decay = decay_per_s
        self._offset: float | None = None
        self._t: float | None = None
        self._lock = threading.Lock()

    def observe(self, peer_stamp: float) -> None:
        now = self._local()
        sample = peer_stamp - now
        with self._lock:
            if self._offset is None:
                self._offset = sample
            else:
                decayed = self._offset - self._decay * max(0.0, now - (self._t or now))
                self._offset = max(sample, decayed)
            self._t = now

    @property
    def offset(self) -> float:
        with self._lock:
            return 0.0 if self._offset is None else self._offset

    def __call__(self) -> float:
        return self._local() + self.offset


@dataclass(frozen=True)
class AppConfig:
    control_hz: float = 10.0
    send_hz: float = 20.0
    max_command_age_s: float = 0.5
    maps_dir: str = "maps"


class OrinApp:
    def __init__(self, runtime: OrinRuntime, drive_sink: DriveSink, *, clock: Callable[[], float] | None = None,
                 config: AppConfig | None = None) -> None:
        self.runtime = runtime
        self.drive_sink = drive_sink
        self.config = config or AppConfig()
        self.clock = clock or runtime.clock
        self._lock = threading.Lock()
        self._cmd = DriveOutput()
        self._cmd_t = -math.inf
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._stats = {"steps": 0, "step_errors": 0, "sent": 0, "stale_zeros": 0, "last_step_ms": None,
                       "max_step_ms": 0.0}

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        for name, fn in (("orin-control", self._control_loop), ("orin-drive", self._send_loop)):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout)
        try:
            self.drive_sink(0.0, 0.0)
        except Exception:
            log.exception("final zero drive failed")

    # ------------------------------------------------------------------ inputs
    def on_scan(self, scan: Scan) -> None:
        self.runtime.on_scan(scan)

    def on_frame(self, jpeg: bytes, capture_s: float) -> None:
        self.runtime.on_frame(jpeg, capture_s)

    # ------------------------------------------------------------------ loops
    def step_once(self) -> DriveOutput:
        t0 = time.perf_counter()
        try:
            out = self.runtime.step(self.clock())
        except Exception:
            log.exception("control step failed")
            with self._lock:
                self._stats["step_errors"] += 1
                self._cmd, self._cmd_t = DriveOutput(), -math.inf
            return DriveOutput()
        ms = (time.perf_counter() - t0) * 1e3
        with self._lock:
            self._cmd, self._cmd_t = out, time.monotonic()
            self._stats["steps"] += 1
            self._stats["last_step_ms"] = round(ms, 1)
            self._stats["max_step_ms"] = round(max(self._stats["max_step_ms"], ms), 1)
        return out

    def current_command(self) -> DriveOutput:
        """Newest requested drive, or zero if the control thread has gone quiet."""
        with self._lock:
            if time.monotonic() - self._cmd_t > self.config.max_command_age_s:
                if self._cmd_t != -math.inf:
                    self._stats["stale_zeros"] += 1
                return DriveOutput()
            return self._cmd

    def _control_loop(self) -> None:
        period = 1.0 / self.config.control_hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.step_once()
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def _send_loop(self) -> None:
        period = 1.0 / self.config.send_hz
        while not self._stop.is_set():
            cmd = self.current_command()
            try:
                self.drive_sink(cmd.throttle, cmd.steering)
                with self._lock:
                    self._stats["sent"] += 1
            except Exception:
                log.exception("drive sink failed")
            self._stop.wait(period)

    # ------------------------------------------------------------------ operator actions
    def operator_skill(self, name: str, args: dict) -> SkillCall:
        call = parse_tool_call(name, args, source="operator", reason="operator")
        self.runtime.submit_operator(call)
        return call

    def save_map(self, name: str) -> dict:
        if not re.fullmatch(r"[A-Za-z0-9._-]{1,64}", name or ""):
            raise ValueError("map name: 1-64 letters, digits, dot, dash or underscore")
        paths = self.runtime.mapper.save(Path(self.config.maps_dir) / name, meta={"saved_by": "operator"})
        return {k: str(v) for k, v in paths.items()}

    def relocalize(self, body: dict) -> bool:
        """Re-initialize localization in a saved map around an operator pose (``{}`` = global).
        Returns False when the session is building a new map (nothing to localize in)."""
        from locatebot.lidar.types import Pose2D
        guess = None
        if body:
            try:
                x, y, th = float(body["x"]), float(body["y"]), math.radians(float(body.get("theta_deg", 0.0)))
            except (KeyError, TypeError, ValueError) as e:
                raise ValueError("relocalize needs numbers x, y and optional theta_deg") from e
            if not all(map(math.isfinite, (x, y, th))):
                raise ValueError("relocalize pose must be finite")
            guess = Pose2D(x, y, th)
        mapper = self.runtime.mapper
        if not getattr(mapper, "localizing", False):
            return False
        mapper.relocalize(guess)
        return True

    def status(self) -> dict:
        out = self.runtime.status()
        with self._lock:
            out["app"] = dict(self._stats)
            out["app"]["command"] = {"throttle": round(self._cmd.throttle, 3), "steering": round(self._cmd.steering, 3)}
        g = self.runtime.mapper.grid_snapshot()
        out["map_meta"] = {"width": g.width, "height": g.height, "resolution_m": g.resolution_m,
                           "origin_xy": list(g.origin_xy)}
        return out


# ---------------------------------------------------------------------- map image
def _png(gray: np.ndarray) -> bytes:
    """Encode an 8-bit grayscale image as PNG (stdlib zlib only)."""
    h, w = gray.shape
    raw = b"".join(b"\x00" + gray[r].tobytes() for r in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def map_png(runtime: OrinRuntime, overlay: str | None = None) -> bytes:
    """Map image, north up: row 0 is the top (max y), like the PGM export.

    ``overlay="wifi"`` (M3, saved-map mode): cells with Wi-Fi readings tinted red (-90 dBm) to
    green (-40 dBm); an RGB PNG. Without readings it is the plain grey map."""
    classes = runtime.mapper.grid_snapshot().classify()
    img = np.full(classes.shape, 160, np.uint8)
    img[classes == FREE] = 255
    img[classes == OCCUPIED] = 0
    wifi = runtime.mapper.layer_snapshot("wifi_rssi") if overlay == "wifi" and hasattr(
        runtime.mapper, "layer_snapshot") else None
    if wifi is None or wifi.shape != img.shape or not np.isfinite(wifi).any():
        return _png(np.ascontiguousarray(img[::-1]))
    rgb = np.repeat(img[:, :, None], 3, axis=2).astype(np.float32)
    m = np.isfinite(wifi)
    t = np.clip((wifi[m] + 90.0) / 50.0, 0.0, 1.0)
    tint = np.column_stack((255 * (1 - t), 255 * t, np.full_like(t, 40)))
    rgb[m] = 0.4 * rgb[m] + 0.6 * tint
    return _png_rgb(np.ascontiguousarray(rgb.astype(np.uint8)[::-1]))


def _png_rgb(rgb: np.ndarray) -> bytes:
    """Encode an 8-bit RGB image (H, W, 3) as PNG (stdlib zlib only)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def scan_json(runtime: OrinRuntime, max_points: int = 1000) -> dict:
    """Newest scan for the live radar view: valid points only, robot frame, angles in degrees.

    Includes the pose so the page can show heading, and a per-second scan rate."""
    scan = runtime.mapper.latest_scan
    if scan is None:
        return {"seq": None, "stamp_s": None, "a": [], "r": [], "q": []}
    m = scan.valid_mask()
    a = np.degrees(scan.angles_rad[m])
    r = scan.ranges_m[m]
    q = scan.quality[m]
    if len(a) > max_points:
        idx = np.linspace(0, len(a) - 1, max_points).astype(int)
        a, r, q = a[idx], r[idx], q[idx]
    p = runtime.mapper.pose
    return {"seq": scan.seq, "stamp_s": scan.stamp_s, "n_beams": len(scan), "n_valid": int(m.sum()),
            "a": np.round(a, 2).tolist(), "r": np.round(r, 3).tolist(), "q": q.astype(int).tolist(),
            "pose": {"x": round(p.x, 3), "y": round(p.y, 3), "theta_deg": round(math.degrees(p.theta), 1)},
            "scans": runtime.mapper.stats()["scans"]}


# ---------------------------------------------------------------------- HTTP
MAX_BODY = 4096


class OrinHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, app: OrinApp):
        self.app = app
        super().__init__(address, _Handler)


def make_server(app: OrinApp, host: str = "127.0.0.1", port: int = 8090) -> OrinHTTPServer:
    return OrinHTTPServer((host, port), app)


class _Handler(BaseHTTPRequestHandler):
    server: OrinHTTPServer

    def log_message(self, format, *args):  # noqa: A002 - the page polls; keep the log quiet
        pass

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Security-Policy",
                         "default-src 'none'; img-src 'self'; script-src 'unsafe-inline'; "
                         "style-src 'unsafe-inline'; connect-src 'self'")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj) -> None:
        self._send(code, json.dumps(obj, default=str).encode(), "application/json")

    def do_GET(self):  # noqa: N802
        app = self.server.app
        if self.path == "/":
            self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        elif self.path == "/status":
            self._json(200, app.status())
        elif self.path.startswith("/map.png"):
            self._send(200, map_png(app.runtime, "wifi" if "wifi=1" in self.path else None), "image/png")
        elif self.path.startswith("/scan.json"):
            self._json(200, scan_json(app.runtime))
        elif self.path == "/radar":
            self._send(200, RADAR_PAGE.encode(), "text/html; charset=utf-8")
        else:
            self._json(404, {"error": "not found"})

    def _body(self) -> dict:
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            raise ValueError("Content-Type must be application/json")
        n = int(self.headers.get("Content-Length") or 0)
        if n > MAX_BODY:
            raise ValueError("body too large")
        raw = self.rfile.read(n) if n else b"{}"
        body = json.loads(raw or b"{}", parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
        if not isinstance(body, dict):
            raise ValueError("body must be a JSON object")
        return body

    def do_POST(self):  # noqa: N802
        app = self.server.app
        rt = app.runtime
        try:
            body = self._body()
            if self.path == "/stop":
                app.operator_skill("stop", {})
                return self._json(200, {"ok": True})
            if self.path == "/skill":
                name = body.get("name")
                args = body.get("args") or {}
                if not isinstance(name, str) or not isinstance(args, dict):
                    raise ValueError("need name (string) and args (object)")
                call = app.operator_skill(name, args)
                return self._json(200, {"ok": True, "call": {"name": call.name.value, "args": dict(call.args)}})
            if rt.brain is None and self.path in ("/approve", "/reject", "/resume", "/mode"):
                return self._json(409, {"error": "no LLM brain configured"})
            if self.path == "/approve":
                call = rt.brain.approve()
                return self._json(200, {"ok": call is not None})
            if self.path == "/reject":
                return self._json(200, {"ok": rt.brain.reject()})
            if self.path == "/resume":
                rt.brain.resume()
                return self._json(200, {"ok": True})
            if self.path == "/mode":
                rt.brain.set_mode(body.get("mode"))
                return self._json(200, {"ok": True, "mode": rt.brain.mode})
            if self.path == "/save_map":
                return self._json(200, {"ok": True, "files": app.save_map(body.get("name", ""))})
            if self.path == "/goal":
                goal = rt.set_goal(body.get("text"), source="operator")
                return self._json(200, {"ok": True, "goal": goal})
            if self.path == "/goal/clear":
                rt.clear_goal()
                return self._json(200, {"ok": True})
            if self.path == "/relocalize":
                if not app.relocalize(body):
                    return self._json(409, {"error": "not localizing in a saved map (start with --map)"})
                return self._json(200, {"ok": True})
            return self._json(404, {"error": "not found"})
        except (ValueError, InvalidSkillCall, json.JSONDecodeError) as e:
            return self._json(400, {"error": str(e)})


_SKILL_BUTTONS = "".join(f'<option value="{s.value}">{s.value}</option>' for s in SkillName)

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>locatebot mission</title>
<style>
:root{--bg:#111;--panel:#1c1c1c;--fg:#eee;--dim:#999;--red:#d00;--ok:#2a7;--warn:#c80}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
header{display:flex;gap:12px;align-items:center;padding:10px 16px;background:#000}
header h1{font-size:17px;margin:0;flex:1}
#stop{background:var(--red);color:#fff;border:0;font-size:20px;font-weight:700;padding:12px 28px;border-radius:6px}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(0,2fr);gap:12px;padding:12px 16px}
@media (max-width:800px){main{grid-template-columns:1fr}}
section{background:var(--panel);border-radius:8px;padding:12px}
h2{font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:var(--dim);margin:0 0 8px}
canvas{width:100%;background:#555;border-radius:4px;image-rendering:pixelated}
table{width:100%;border-collapse:collapse}td{padding:3px 4px;border-bottom:1px solid #2a2a2a}
.k{color:var(--dim)}button{background:#333;color:var(--fg);border:1px solid #444;border-radius:5px;padding:7px 12px;font-size:14px}
button.go{background:var(--ok);border:0}input,select{background:#222;color:var(--fg);border:1px solid #444;border-radius:4px;padding:6px;font-size:14px}
#proposal{border:2px solid var(--warn);border-radius:6px;padding:8px;margin:6px 0;display:none}
#events div{font-size:13px;color:#ccc;padding:2px 0;border-bottom:1px solid #262626}
.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:6px 0}
</style></head><body>
<header><h1>locatebot mission</h1><a href="/radar" style="color:#9cf">live radar</a><span id="conn" class="k">connecting...</span><button id="stop">STOP</button></header>
<main>
<section><h2>Map</h2><canvas id="map" width="400" height="400"></canvas>
<div class="row k" id="mapinfo"></div><label class="k"><input type="checkbox" id="wifi"> Wi-Fi overlay</label>
<div class="row"><input id="mapname" placeholder="map name" size="12"><button id="save">Save map</button></div></section>
<section>
<h2>Goal</h2><div class="row"><span id="goal" class="k">none</span></div>
<div class="row"><input id="goaltext" placeholder="e.g. find the bolt" size="24" maxlength="200"><button class="go" id="setgoal">Set goal</button><button id="cleargoal">Clear</button></div>
<h2 style="margin-top:12px">Skill</h2><table id="skill"></table>
<div id="proposal"><b>LLM proposes:</b> <span id="prop"></span>
<div class="row"><button class="go" id="approve">Approve</button><button id="reject">Reject</button></div></div>
<div class="row"><span class="k">LLM:</span><span id="brain">-</span>
<button id="mode">toggle approve/auto</button><button id="resume">resume LLM</button></div>
<h2 style="margin-top:12px">Send a skill</h2>
<div class="row"><select id="sname">SKILLS</select><input id="sargs" placeholder='args JSON, e.g. {"items":["marker"]}' size="26"><button id="send">Send</button></div>
<h2 style="margin-top:12px">Items</h2><table id="items"></table>
<h2 style="margin-top:12px">Events</h2><div id="events"></div>
</section></main>
<script>
"use strict";
const $=id=>document.getElementById(id);
async function post(path,body){const r=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body||{})});
 const j=await r.json().catch(()=>({}));if(!r.ok)alert(j.error||r.status);return j}
$("stop").onclick=()=>post("/stop");$("approve").onclick=()=>post("/approve");$("reject").onclick=()=>post("/reject");
$("resume").onclick=()=>post("/resume");
$("setgoal").onclick=()=>post("/goal",{text:$("goaltext").value});$("cleargoal").onclick=()=>post("/goal/clear");
$("mode").onclick=async()=>{const m=(window.lastStatus&&window.lastStatus.brain&&window.lastStatus.brain.mode)==="auto"?"approve":"auto";post("/mode",{mode:m})};
$("send").onclick=()=>{let a={};const t=$("sargs").value.trim();if(t){try{a=JSON.parse(t)}catch(e){alert("args must be JSON");return}}post("/skill",{name:$("sname").value,args:a})};
$("save").onclick=async()=>{const j=await post("/save_map",{name:$("mapname").value.trim()});if(j.ok)alert("saved "+j.files.lbmap)};
const img=new Image();let imgReady=false;img.onload=()=>{imgReady=true;draw()};
function esc(s){return String(s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]))}
function rows(el,pairs){el.innerHTML=pairs.map(([k,v])=>`<tr><td class="k">${esc(k)}</td><td>${esc(v)}</td></tr>`).join("")}
function mapMode(m){if(m.mode!=="localizing")return "mapping (new map)";
 const st=m.lost?"LOST":m.ambiguous?"ambiguous: give a start pose":!m.initialized?"localizing...":"localized ("+m.source+")";
 return st+(m.last_score==null?"":" score "+m.last_score)+(m.changes?", "+m.changes+" new things":"")}
function draw(){const s=window.lastStatus;if(!s||!imgReady)return;const c=$("map"),g=c.getContext("2d"),m=s.map_meta;
 c.width=m.width;c.height=m.height;g.imageSmoothingEnabled=false;g.drawImage(img,0,0);
 const px=(x,y)=>[(x-m.origin_xy[0])/m.resolution_m,m.height-(y-m.origin_xy[1])/m.resolution_m];
 for(const it of s.items){const[u,v]=px(it.x,it.y);g.fillStyle=it.confirmed?"#2c6":"#ca0";g.beginPath();g.arc(u,v,5,0,7);g.fill();g.fillStyle="#fff";g.font="11px sans-serif";g.fillText(it.name,u+6,v-6)}
 const p=s.pose,[u,v]=px(p.x,p.y),th=p.theta_deg*Math.PI/180;g.strokeStyle="#e33";g.fillStyle="#e33";g.lineWidth=2;
 g.beginPath();g.arc(u,v,4,0,7);g.fill();g.beginPath();g.moveTo(u,v);g.lineTo(u+12*Math.cos(th),v-12*Math.sin(th));g.stroke()}
async function tick(){try{const s=await (await fetch("/status")).json();window.lastStatus=s;$("conn").textContent="live";
 $("goal").textContent=s.goal?s.goal.text+" (by "+s.goal.source+")":"none";
 const sk=s.skill;rows($("skill"),[["state",sk.state],["skill",sk.call?sk.call.name+" "+JSON.stringify(sk.call.args)+" ("+sk.call.source+")":"-"],
  ["detail",sk.detail||"-"],["explored",s.explored_frac==null?"-":Math.round(s.explored_frac*100)+" %"],["safety",s.safety_reason],
  ["pose",`(${s.pose.x}, ${s.pose.y}) ${s.pose.theta_deg}°`],["drive",`thr ${s.app.command.throttle}  steer ${s.app.command.steering}`],
  ["step ms",`${s.app.last_step_ms} (max ${s.app.max_step_ms})`],["map",mapMode(s.map)]]);
 const b=s.brain;$("brain").textContent=b?`${b.mode}${b.paused?" (paused)":""}${b.in_fallback?" FALLBACK":""} failures ${b.failures}${b.last_error?" - "+b.last_error:""}`:"none (operator only)";
 const pend=b&&b.pending;$("proposal").style.display=pend?"block":"none";if(pend)$("prop").textContent=typeof pend==="string"?pend:JSON.stringify(pend);
 rows($("items"),s.items.map(i=>[i.name,`${i.confirmed?"confirmed":"unconfirmed"} (${i.sightings}) at (${i.x}, ${i.y})`]));
 $("events").innerHTML=s.events.slice().reverse().map(e=>`<div>${esc(e.text)}</div>`).join("");
 $("mapinfo").textContent=`${s.map.integrated}/${s.map.scans} scans integrated`;
 img.src="/map.png?"+($("wifi").checked?"wifi=1&":"")+Date.now();draw()}catch(e){$("conn").textContent="disconnected"}}
setInterval(tick,700);tick();
</script></body></html>
""".replace("SKILLS", _SKILL_BUTTONS)


RADAR_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>locatebot radar</title>
<style>
:root{--bg:#05080a;--fg:#d8f5e3;--dim:#6f8f7e;--grid:#173326;--grid2:#24503a;--pt:#5dff9e;--near:#ff4d4d;--mid:#ffc14d}
*{box-sizing:border-box}html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:14px ui-monospace,monospace}
#wrap{display:flex;flex-direction:column;height:100%}
header{display:flex;gap:16px;align-items:center;flex-wrap:wrap;padding:8px 14px;border-bottom:1px solid var(--grid)}
header b{font-size:15px}.k{color:var(--dim)}button{background:#0d1712;color:var(--fg);border:1px solid var(--grid2);border-radius:4px;padding:4px 10px;font:inherit}
button.on{background:#1d4430}#cv{flex:1;width:100%;min-height:0;display:block}
#sec{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:4px 12px;padding:8px 14px;border-top:1px solid var(--grid)}
@media (max-width:700px){#sec{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body><div id="wrap">
<header><b>LiDAR radar (live)</b><span id="st" class="k">connecting...</span>
<span>range <button data-r="1">1 m</button><button data-r="2">2 m</button><button data-r="4">4 m</button><button data-r="8">8 m</button><button data-r="0" class="on">auto</button></span>
<span>trail <button id="trail" class="on">on</button></span><a href="/" style="color:var(--dim)">mission page</a></header>
<canvas id="cv"></canvas>
<div id="sec"></div></div>
<script>
"use strict";
const cv=document.getElementById("cv"),g=cv.getContext("2d");let fixedR=0,trail=true,last=null,frames=[],rate=0,lastSeq=null,seqT=[];
document.querySelectorAll("button[data-r]").forEach(b=>b.onclick=()=>{fixedR=+b.dataset.r;document.querySelectorAll("button[data-r]").forEach(x=>x.classList.toggle("on",x===b))});
document.getElementById("trail").onclick=e=>{trail=!trail;e.target.textContent=trail?"on":"off";e.target.classList.toggle("on",trail)};
const NAMES=["front","front-left","left","back-left","back","back-right","right","front-right"];
function resize(){const r=cv.getBoundingClientRect(),d=window.devicePixelRatio||1;cv.width=r.width*d;cv.height=r.height*d}
window.addEventListener("resize",resize);resize();
function colour(r){return r<0.4?getComputedStyle(document.documentElement).getPropertyValue("--near"):r<1.0?getComputedStyle(document.documentElement).getPropertyValue("--mid"):getComputedStyle(document.documentElement).getPropertyValue("--pt")}
function draw(){const s=last;const W=cv.width,H=cv.height,cx=W/2,cy=H/2,d=window.devicePixelRatio||1;g.fillStyle="#05080a";g.fillRect(0,0,W,H);
 if(!s||!s.r.length){g.fillStyle="#6f8f7e";g.font=14*d+"px monospace";g.fillText(s?"no valid points":"waiting for scans...",20*d,30*d);return}
 let R=fixedR;if(!R){const sorted=[...s.r].sort((a,b)=>a-b);R=Math.max(0.5,Math.ceil(sorted[Math.floor(sorted.length*0.98)]*2)/2)}
 const ppm=Math.min(W,H)/2/R*0.94,step=R<=2?0.25:R<=4?0.5:1;
 g.lineWidth=1*d;for(let k=step;k<=R+1e-9;k+=step){g.strokeStyle=Math.abs(k-Math.round(k))<1e-6?"#24503a":"#173326";g.beginPath();g.arc(cx,cy,k*ppm,0,7);g.stroke();
  g.fillStyle="#6f8f7e";g.font=11*d+"px monospace";g.fillText(k.toFixed(step<1?2:0)+" m",cx+4*d,cy-k*ppm-3*d)}
 g.strokeStyle="#173326";for(let i=0;i<12;i++){const t=i*Math.PI/6;g.beginPath();g.moveTo(cx,cy);g.lineTo(cx-Math.sin(t)*R*ppm,cy-Math.cos(t)*R*ppm);g.stroke()}
 const P=(a,r)=>{const t=a*Math.PI/180;return[cx-Math.sin(t)*r*ppm,cy-Math.cos(t)*r*ppm]};
 if(trail){frames.forEach((f,i)=>{g.fillStyle="rgba(93,255,158,"+(0.08+0.05*i)+")";for(let j=0;j<f.r.length;j++){const[x,y]=P(f.a[j],f.r[j]);g.fillRect(x-d,y-d,2*d,2*d)}})}
 for(let j=0;j<s.r.length;j++){const[x,y]=P(s.a[j],s.r[j]);g.fillStyle=colour(s.r[j]);g.fillRect(x-1.5*d,y-1.5*d,3*d,3*d)}
 g.fillStyle="#ff4d4d";g.beginPath();g.arc(cx,cy,5*d,0,7);g.fill();g.strokeStyle="#ff4d4d";g.lineWidth=3*d;g.beginPath();g.moveTo(cx,cy);g.lineTo(cx,cy-28*d);g.stroke();
 g.fillStyle="#d8f5e3";g.font=12*d+"px monospace";g.fillText("FRONT",cx-20*d,cy-34*d);g.fillText("LEFT",14*d,cy-6*d);g.fillText("RIGHT",W-54*d,cy-6*d)}
function sectors(s){const best=new Array(8).fill(Infinity);for(let j=0;j<s.r.length;j++){let a=((s.a[j]+22.5)%360+360)%360;const k=Math.floor(a/45)%8;if(s.r[j]<best[k])best[k]=s.r[j]}
 document.getElementById("sec").innerHTML=best.map((v,i)=>`<div><span class="k">${NAMES[i]}</span> ${isFinite(v)?v.toFixed(2)+" m":"none"}</div>`).join("")}
async function loop(){for(;;){const t0=performance.now();try{const s=await (await fetch("/scan.json",{cache:"no-store"})).json();
  if(s.seq!==lastSeq){lastSeq=s.seq;seqT.push(performance.now());seqT=seqT.filter(t=>performance.now()-t<2000);if(last&&last.r.length){frames.push(last);if(frames.length>4)frames.shift()}last=s;sectors(s)}
  document.getElementById("st").textContent=`scan #${s.seq} | ${s.n_valid}/${s.n_beams} valid points | ${(seqT.length/2).toFixed(1)} scans/s | heading ${s.pose?s.pose.theta_deg:"-"}\u00b0`}
 catch(e){document.getElementById("st").textContent="disconnected, retrying..."}
 draw();await new Promise(r=>setTimeout(r,Math.max(20,100-(performance.now()-t0))))}}
loop();
</script></body></html>
"""
