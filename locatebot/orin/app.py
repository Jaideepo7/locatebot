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


def map_png(runtime: OrinRuntime) -> bytes:
    """Map image, north up: row 0 is the top (max y), like the PGM export."""
    classes = runtime.mapper.grid_snapshot().classify()
    img = np.full(classes.shape, 160, np.uint8)
    img[classes == FREE] = 255
    img[classes == OCCUPIED] = 0
    return _png(np.ascontiguousarray(img[::-1]))


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
            self._send(200, map_png(app.runtime), "image/png")
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
<header><h1>locatebot mission</h1><span id="conn" class="k">connecting...</span><button id="stop">STOP</button></header>
<main>
<section><h2>Map</h2><canvas id="map" width="400" height="400"></canvas>
<div class="row k" id="mapinfo"></div>
<div class="row"><input id="mapname" placeholder="map name" size="12"><button id="save">Save map</button></div></section>
<section>
<h2>Skill</h2><table id="skill"></table>
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
$("mode").onclick=async()=>{const m=(window.lastStatus&&window.lastStatus.brain&&window.lastStatus.brain.mode)==="auto"?"approve":"auto";post("/mode",{mode:m})};
$("send").onclick=()=>{let a={};const t=$("sargs").value.trim();if(t){try{a=JSON.parse(t)}catch(e){alert("args must be JSON");return}}post("/skill",{name:$("sname").value,args:a})};
$("save").onclick=async()=>{const j=await post("/save_map",{name:$("mapname").value.trim()});if(j.ok)alert("saved "+j.files.lbmap)};
const img=new Image();let imgReady=false;img.onload=()=>{imgReady=true;draw()};
function esc(s){return String(s).replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]))}
function rows(el,pairs){el.innerHTML=pairs.map(([k,v])=>`<tr><td class="k">${esc(k)}</td><td>${esc(v)}</td></tr>`).join("")}
function draw(){const s=window.lastStatus;if(!s||!imgReady)return;const c=$("map"),g=c.getContext("2d"),m=s.map_meta;
 c.width=m.width;c.height=m.height;g.imageSmoothingEnabled=false;g.drawImage(img,0,0);
 const px=(x,y)=>[(x-m.origin_xy[0])/m.resolution_m,m.height-(y-m.origin_xy[1])/m.resolution_m];
 for(const it of s.items){const[u,v]=px(it.x,it.y);g.fillStyle=it.confirmed?"#2c6":"#ca0";g.beginPath();g.arc(u,v,5,0,7);g.fill();g.fillStyle="#fff";g.font="11px sans-serif";g.fillText(it.name,u+6,v-6)}
 const p=s.pose,[u,v]=px(p.x,p.y),th=p.theta_deg*Math.PI/180;g.strokeStyle="#e33";g.fillStyle="#e33";g.lineWidth=2;
 g.beginPath();g.arc(u,v,4,0,7);g.fill();g.beginPath();g.moveTo(u,v);g.lineTo(u+12*Math.cos(th),v-12*Math.sin(th));g.stroke()}
async function tick(){try{const s=await (await fetch("/status")).json();window.lastStatus=s;$("conn").textContent="live";
 const sk=s.skill;rows($("skill"),[["state",sk.state],["skill",sk.call?sk.call.name+" "+JSON.stringify(sk.call.args)+" ("+sk.call.source+")":"-"],
  ["detail",sk.detail||"-"],["explored",s.explored_frac==null?"-":Math.round(s.explored_frac*100)+" %"],["safety",s.safety_reason],
  ["pose",`(${s.pose.x}, ${s.pose.y}) ${s.pose.theta_deg}°`],["drive",`thr ${s.app.command.throttle}  steer ${s.app.command.steering}`],
  ["step ms",`${s.app.last_step_ms} (max ${s.app.max_step_ms})`]]);
 const b=s.brain;$("brain").textContent=b?`${b.mode}${b.paused?" (paused)":""}${b.in_fallback?" FALLBACK":""} failures ${b.failures}${b.last_error?" - "+b.last_error:""}`:"none (operator only)";
 const pend=b&&b.pending;$("proposal").style.display=pend?"block":"none";if(pend)$("prop").textContent=typeof pend==="string"?pend:JSON.stringify(pend);
 rows($("items"),s.items.map(i=>[i.name,`${i.confirmed?"confirmed":"unconfirmed"} (${i.sightings}) at (${i.x}, ${i.y})`]));
 $("events").innerHTML=s.events.slice().reverse().map(e=>`<div>${esc(e.text)}</div>`).join("");
 $("mapinfo").textContent=`${s.map.integrated}/${s.map.scans} scans integrated`;
 img.src="/map.png?"+Date.now();draw()}catch(e){$("conn").textContent="disconnected"}}
setInterval(tick,700);tick();
</script></body></html>
""".replace("SKILLS", _SKILL_BUTTONS)
