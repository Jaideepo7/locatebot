"""OrinApp threads, PeerClock, map PNG, operator HTTP API and the orin_run.py --sim smoke test."""
import json
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import zlib
from pathlib import Path

import numpy as np
import pytest

from locatebot.lidar.mapping import demo_room
from locatebot.lidar.sim import Room, simulate_scan
from locatebot.lidar.types import Pose2D
from locatebot.mission.types import SkillName, VehicleConfig
from locatebot.orin.app import AppConfig, OrinApp, PeerClock, _png, make_server, map_png
from locatebot.orin.live_map import LiveMapper
from locatebot.orin.runtime import OrinRuntime
from locatebot.orin.skills import SkillExecutor

ROOT = Path(__file__).resolve().parents[2]


def runtime_with_map(n=5):
    rt = OrinRuntime(LiveMapper(size_m=12.0), SkillExecutor(VehicleConfig()))
    rng = np.random.default_rng(0)
    for i in range(n):
        rt.on_scan(simulate_scan(demo_room(), Pose2D(4.0, 3.0, 0.0), rng=rng, stamp_s=1.0 + 0.1 * i, seq=i))
    return rt


def test_peer_clock_tracks_offset_and_ignores_latency():
    local = [100.0]
    pc = PeerClock(local=lambda: local[0], decay_per_s=0.0)
    assert pc() == 100.0  # no samples: local time
    pc.observe(1000.0)          # peer stamp, received with zero latency
    local[0] = 100.5
    pc.observe(1000.5 - 0.2)    # 0.2 s late: must not drag the offset down
    assert pc.offset == pytest.approx(900.0)
    assert pc() == pytest.approx(1000.5)


def test_command_goes_stale_to_zero():
    sent = []
    app = OrinApp(runtime_with_map(), lambda t, s: sent.append((t, s)), config=AppConfig(max_command_age_s=0.05))
    from locatebot.orin.skills import DriveOutput
    app._cmd, app._cmd_t = DriveOutput(0.3, 0.5), time.monotonic()
    assert app.current_command() == DriveOutput(0.3, 0.5)
    time.sleep(0.08)
    assert app.current_command() == DriveOutput()
    assert app.status()["app"]["stale_zeros"] == 1


def test_step_errors_send_zero(monkeypatch):
    app = OrinApp(runtime_with_map(), lambda t, s: None)

    def boom(now):
        raise RuntimeError("bad")

    monkeypatch.setattr(app.runtime, "step", boom)
    out = app.step_once()
    assert (out.throttle, out.steering) == (0.0, 0.0) and app.status()["app"]["step_errors"] == 1


def test_threads_send_and_stop_with_zero():
    sent = []
    app = OrinApp(runtime_with_map(), lambda t, s: sent.append((t, s)), config=AppConfig(control_hz=20, send_hz=50))
    app.start()
    time.sleep(0.3)
    app.stop()
    assert len(sent) >= 5 and sent[-1] == (0.0, 0.0)
    assert app.status()["app"]["steps"] >= 3


def test_png_is_valid():
    img = np.arange(12, dtype=np.uint8).reshape(3, 4)
    png = _png(img)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    w, h = struct.unpack(">II", png[16:24])
    assert (w, h) == (4, 3)
    i = png.index(b"IDAT")
    (n,) = struct.unpack(">I", png[i - 4:i])
    raw = zlib.decompress(png[i + 4:i + 4 + n])
    assert raw == b"".join(b"\x00" + bytes(img[r]) for r in range(3))
    assert map_png(runtime_with_map())[:8] == b"\x89PNG\r\n\x1a\n"


@pytest.fixture
def server(tmp_path):
    app = OrinApp(runtime_with_map(), lambda t, s: None, config=AppConfig(maps_dir=str(tmp_path)))
    srv = make_server(app, "127.0.0.1", 0)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield app, f"http://127.0.0.1:{srv.server_address[1]}", tmp_path
    srv.shutdown()
    srv.server_close()


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, r.headers.get("Content-Type"), r.read()


def post(url, body, ctype="application/json"):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": ctype})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_page_status_and_map(server):
    app, base, _ = server
    code, ctype, page = get(base + "/")
    assert code == 200 and b"STOP" in page
    assert b"http://" not in page and b"https://" not in page  # no external assets
    code, _, raw = get(base + "/status")
    st = json.loads(raw)
    assert {"pose", "skill", "items", "events", "map", "app", "map_meta"} <= set(st)
    code, ctype, png = get(base + "/map.png")
    assert ctype == "image/png" and png[:4] == b"\x89PNG"


def test_operator_skill_validation_and_queue(server):
    app, base, _ = server
    assert post(base + "/skill", {"name": "go_to", "args": {}})[0] == 400
    assert post(base + "/skill", {"name": "fly", "args": {}})[0] == 400
    code, body = post(base + "/skill", {"name": "search", "args": {"items": ["marker"]}})
    assert code == 200 and body["call"]["name"] == "search"
    app.step_once()
    assert app.runtime.executor.status.call.name == SkillName.SEARCH
    assert app.runtime.executor.status.call.source == "operator"
    assert post(base + "/stop", {})[0] == 200
    app.step_once()
    assert app.runtime.executor.status.call.name == SkillName.STOP


def test_rejects_non_json_and_brainless_llm_actions(server):
    app, base, _ = server
    assert post(base + "/stop", {}, ctype="text/plain")[0] == 400
    assert post(base + "/approve", {})[0] == 409


def test_save_map(server):
    app, base, tmp = server
    assert post(base + "/save_map", {"name": "../evil"})[0] == 400
    code, body = post(base + "/save_map", {"name": "room1"})
    assert code == 200 and (tmp / "room1.lbmap").exists()


def test_orin_run_sim_smoke():
    p = subprocess.run([sys.executable, str(ROOT / "tools" / "orin_run.py"), "--sim", "--port", "0", "--seconds", "3"],
                       capture_output=True, text=True, timeout=60, cwd=ROOT)
    assert p.returncode == 0, p.stderr[-2000:]
    assert "operator page: http://127.0.0.1:" in p.stdout and "stopped:" in p.stdout


def test_radar_page_and_scan_json(server):
    app, base, _ = server
    code, ctype, page = get(base + "/radar")
    assert code == 200 and b"<canvas" in page and b"/scan.json" in page
    assert b"http://" not in page and b"https://" not in page
    code, _, raw = get(base + "/scan.json")
    s = json.loads(raw)
    assert s["seq"] == 4 and len(s["a"]) == len(s["r"]) == len(s["q"]) == s["n_valid"] > 100
    assert all(r > 0 for r in s["r"]) and -180.0 <= min(s["a"]) and max(s["a"]) <= 180.0
    assert set(s["pose"]) == {"x", "y", "theta_deg"}


def test_scan_json_without_scans_and_downsampling():
    from locatebot.orin.app import scan_json
    empty = OrinRuntime(LiveMapper(size_m=8.0), SkillExecutor(VehicleConfig()))
    assert scan_json(empty)["a"] == []
    s = scan_json(runtime_with_map(), max_points=50)
    assert len(s["a"]) == 50 and s["n_valid"] > 50


def _orin_run_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location("orin_run", ROOT / "tools" / "orin_run.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_nemotron_preset_uses_nvidia_api_and_key_env(monkeypatch):
    import argparse
    mod = _orin_run_module()
    llm = mod.make_llm(argparse.Namespace(llm="nemotron", llm_model="nvidia/some-nemotron", llm_url=None))
    assert llm.url == "https://integrate.api.nvidia.com/v1/chat/completions"
    assert llm.model == "nvidia/some-nemotron" and llm.api_key_env == "NVIDIA_API_KEY"
    monkeypatch.setenv("NVIDIA_API_KEY", "nvapi-test")
    headers, body = llm.build_request("sys", "user", [])
    assert body["model"] == "nvidia/some-nemotron" and body["tool_choice"] == "auto"
    assert any(v == "Bearer nvapi-test" for v in headers.values())
    with pytest.raises(SystemExit):
        mod.make_llm(argparse.Namespace(llm="nemotron", llm_model=None, llm_url=None))


def test_relocalize_endpoint_needs_saved_map_mode(server):
    app, base, _ = server
    assert post(base + "/relocalize", {})[0] == 409            # building a new map
    assert post(base + "/relocalize", {"x": "a", "y": 1})[0] == 400
    code, body = get(base + "/status")[0], json.loads(get(base + "/status")[2])
    assert body["map"]["mode"] == "mapping"


def test_relocalize_endpoint_in_saved_map_mode(tmp_path):
    from locatebot.lidar.mapping import Mapper
    rng = np.random.default_rng(3)
    room = Room.box(4.0, 3.0).with_box(1.0, 1.0, 1.4, 1.6)
    m = Mapper(size_m=8.0)
    for i in range(5):
        m.add_scan(simulate_scan(room, Pose2D(2.5, 1.5, 0.1), rng=rng, stamp_s=i * 0.1, seq=i))
    path = m.save(tmp_path / "box")["lbmap"]
    rt = OrinRuntime(LiveMapper.from_saved(path, guess=Pose2D(0, 0, 0)), SkillExecutor(VehicleConfig()))
    rt.on_scan(simulate_scan(room, Pose2D(2.5, 1.5, 0.1), rng=rng, stamp_s=1.0, seq=9))
    app = OrinApp(rt, lambda t, s: None)
    st = app.status()["map"]
    assert st["mode"] == "localizing" and st["initialized"] and not st["lost"]
    assert app.relocalize({"x": 0.1, "y": 0.0, "theta_deg": 5}) is True
    assert app.runtime.mapper.stats()["initialized"] is False
    with pytest.raises(ValueError):
        app.relocalize({"x": float("nan"), "y": 0})


def test_orin_run_sim_with_saved_map_writes_v2(tmp_path):
    from locatebot.lidar.mapping import Mapper
    mod = _orin_run_module()
    src = mod.SimSource(VehicleConfig())
    rng = np.random.default_rng(4)
    m = Mapper(size_m=12.0)
    for i in range(10):  # the map frame is the sim car's start pose, like a real first session
        m.add_scan(simulate_scan(src.room, src.car.pose, n_beams=508, rng=rng, stamp_s=i * 0.1, seq=i))
    path = m.save(tmp_path / "simroom")["lbmap"]
    p = subprocess.run([sys.executable, str(ROOT / "tools" / "orin_run.py"), "--sim", "--port", "0", "--seconds", "3",
                        "--map", str(path), "--start-pose", "0,0,0"],
                       capture_output=True, text=True, timeout=60, cwd=ROOT)
    assert p.returncode == 0, p.stderr[-2000:]
    assert "updated map:" in p.stdout, p.stdout + p.stderr[-2000:]
    assert (tmp_path / "simroom.v2.lbmap").exists()
    assert mod.map_stem(Path("maps/room.2026.11.08.lbmap")) == Path("maps/room.2026.11.08")
