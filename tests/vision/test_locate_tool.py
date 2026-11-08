"""Tests for tools/locate.py using the fake engine library (no model, no GPU)."""
from __future__ import annotations

import importlib.util
import json
import struct
from pathlib import Path

import pytest

from tests.vision.test_engine import fake_lib  # noqa: F401  (fixture)

TOOL = Path(__file__).resolve().parents[2] / "tools" / "locate.py"


def _load_tool():
    spec = importlib.util.spec_from_file_location("locate_tool", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _jpeg(path: Path, w: int, h: int) -> Path:
    sof = b"\xff\xc0" + struct.pack(">HBHHB", 11, 8, h, w, 1) + b"\x01\x11\x00"
    path.write_bytes(b"\xff\xd8" + sof + b"\xff\xd9")
    return path


def test_parse_job_splits_items():
    tool = _load_tool()
    assert tool.parse_job("a/b.jpg=tape measure, bolt ") == (Path("a/b.jpg"), ["tape measure", "bolt"])
    with pytest.raises(ValueError):
        tool.parse_job("photo.jpg")


def test_main_runs_every_item_and_writes_report(fake_lib, tmp_path):  # noqa: F811
    tool = _load_tool()
    model = tmp_path / "m.gguf"
    model.write_bytes(b"GGUF")
    img1 = _jpeg(tmp_path / "one.jpg", 640, 480)
    img2 = _jpeg(tmp_path / "two.jpg", 320, 240)
    out = tmp_path / "report.json"

    rc = tool.main([
        "--model", str(model), "--lib", str(fake_lib), "--mode", "fast", "--repeat", "2",
        "--out", str(out), f"{img1}=marker,bolt", f"{img2}=chair",
    ])

    assert rc == 0
    rep = json.loads(out.read_text())
    assert rep["mode"] == "fast"
    assert rep["load_s"] >= 0
    runs = rep["runs"]
    assert [(Path(r["image"]).name, r["item"]) for r in runs] == [
        ("one.jpg", "marker"), ("one.jpg", "bolt"), ("two.jpg", "chair")]
    r0 = runs[0]
    assert r0["width"] == 640 and r0["height"] == 480
    assert len(r0["times_s"]) == 2
    # fake labels the detection with the full prompt and puts the mode code in y1
    assert r0["raw"][0]["label"].endswith("marker.")
    assert r0["raw"][0]["box"][1] == 2.0
    assert r0["clean"] == r0["raw"]  # small box: nothing removed


def test_main_records_engine_errors_and_continues(fake_lib, tmp_path):  # noqa: F811
    tool = _load_tool()
    model = tmp_path / "m.gguf"
    model.write_bytes(b"GGUF")
    img = _jpeg(tmp_path / "x.jpg", 100, 100)
    out = tmp_path / "r.json"

    rc = tool.main(["--model", str(model), "--lib", str(fake_lib), "--out", str(out), f"{img}=FAIL,ok"])

    runs = json.loads(out.read_text())["runs"]
    assert rc == 1
    assert "fake failure" in runs[0]["error"]
    assert runs[1]["item"] == "ok" and "error" not in runs[1]
