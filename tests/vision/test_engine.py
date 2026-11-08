"""Tests for locatebot.vision.engine against a fake liblocate_anything.so.

The fake is compiled from fake_la_capi.c with the system C compiler. It proves the
ctypes binding (signatures, memory ownership, error paths), not the real model or the
Orin GPU build.
"""
from __future__ import annotations

import ctypes
import shutil
import subprocess
from pathlib import Path

import pytest

from locatebot.vision.engine import EngineError, LocateEngine, find_library
from locatebot.vision.postprocess import Detection

HERE = Path(__file__).parent


@pytest.fixture(scope="module")
def fake_lib(tmp_path_factory) -> Path:
    cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if cc is None:
        pytest.skip("no C compiler to build the fake engine library")
    out = tmp_path_factory.mktemp("fake") / "liblocate_anything.so"
    subprocess.run([cc, "-shared", "-fPIC", "-O0", "-o", str(out), str(HERE / "fake_la_capi.c")], check=True)
    return out


@pytest.fixture
def counters(fake_lib):
    lib = ctypes.CDLL(str(fake_lib))  # same handle the engine gets (dlopen is refcounted)
    names = ["fake_abi", "fake_live_ctx", "fake_live_strings", "fake_last_threads"]
    vals = {n: ctypes.c_int.in_dll(lib, n) for n in names}
    vals["fake_abi"].value = 1
    yield vals
    vals["fake_abi"].value = 1


@pytest.fixture
def model(tmp_path) -> Path:
    p = tmp_path / "model.gguf"
    p.write_bytes(b"GGUF")
    return p


def test_locate_passes_bytes_prompt_and_mode_and_parses_result(fake_lib, model, counters):
    with LocateEngine(model, lib_path=fake_lib) as eng:
        dets = eng.locate(b"\xff\xd8abc", "PROMPT", mode="fast")
    # fake encodes box = [len, mode, first_byte, 99]; fast == 2
    assert dets == [Detection("PROMPT", (5.0, 2.0, 255.0, 99.0))]


@pytest.mark.parametrize("mode,code", [("hybrid", 0), ("slow", 1), ("fast", 2)])
def test_mode_names_map_to_capi_codes(fake_lib, model, counters, mode, code):
    with LocateEngine(model, lib_path=fake_lib) as eng:
        assert eng.locate(b"x", "p", mode=mode)[0].box[1] == code


def test_unknown_mode_rejected(fake_lib, model, counters):
    with LocateEngine(model, lib_path=fake_lib) as eng:
        with pytest.raises(ValueError):
            eng.locate(b"x", "p", mode="turbo")


def test_result_strings_and_context_are_freed(fake_lib, model, counters):
    eng = LocateEngine(model, lib_path=fake_lib, n_threads=3)
    assert counters["fake_live_ctx"].value == 1
    assert counters["fake_last_threads"].value == 3
    for _ in range(5):
        eng.locate(b"x", "p")
    assert counters["fake_live_strings"].value == 0
    eng.close()
    eng.close()  # idempotent
    assert counters["fake_live_ctx"].value == 0


def test_engine_error_carries_last_error(fake_lib, model, counters):
    with LocateEngine(model, lib_path=fake_lib) as eng:
        with pytest.raises(EngineError, match="fake failure"):
            eng.locate(b"x", "please FAIL")
        # still usable after a failed call
        assert eng.locate(b"x", "ok")[0].label == "ok"


def test_load_failure_raises(fake_lib, tmp_path, counters):
    bad = tmp_path / "missing.gguf"
    bad.write_bytes(b"")
    with pytest.raises(EngineError, match="load"):
        LocateEngine(bad, lib_path=fake_lib)


def test_missing_model_file_raises_before_loading(fake_lib, tmp_path, counters):
    with pytest.raises(FileNotFoundError):
        LocateEngine(tmp_path / "nope.gguf", lib_path=fake_lib)
    assert counters["fake_live_ctx"].value == 0


def test_abi_mismatch_raises(fake_lib, model, counters):
    counters["fake_abi"].value = 2
    with pytest.raises(EngineError, match="ABI"):
        LocateEngine(model, lib_path=fake_lib)
    assert counters["fake_live_ctx"].value == 0


def test_locate_after_close_raises(fake_lib, model, counters):
    eng = LocateEngine(model, lib_path=fake_lib)
    eng.close()
    with pytest.raises(EngineError, match="closed"):
        eng.locate(b"x", "p")


def test_find_library_searches_build_tree(tmp_path):
    lib = tmp_path / "third_party/locate-anything.cpp/build/src/liblocate_anything.so"
    lib.parent.mkdir(parents=True)
    lib.write_bytes(b"")
    assert find_library(tmp_path) == lib


def test_find_library_missing_raises(tmp_path):
    with pytest.raises(FileNotFoundError, match="build_locate_anything"):
        find_library(tmp_path)
