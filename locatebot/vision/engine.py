"""ctypes binding to locate-anything.cpp's C API (``include/la_capi.h``, ABI v1).

The model is loaded once by ``LocateEngine(...)`` and stays in memory until ``close()``;
each ``locate()`` call runs one detection on JPEG/PNG bytes. Calls block until the model
finishes and cannot be cancelled, so the vision daemon must run them in a background
worker. Calls are serialised by a lock, and ``close()`` waits for a running call to finish
so the C context is never freed underneath it.
"""
from __future__ import annotations

import ctypes
import threading
from pathlib import Path

from .postprocess import Detection, parse_detections

ABI_VERSION = 1
MODES = {"hybrid": 0, "slow": 1, "fast": 2}
LIB_NAME = "liblocate_anything.so"


class EngineError(RuntimeError):
    pass


def find_library(repo_root: str | Path) -> Path:
    """Locate the shared library built by ``scripts/build_locate_anything.sh``."""
    build = Path(repo_root) / "third_party" / "locate-anything.cpp" / "build"
    hits = sorted(build.rglob(LIB_NAME + "*")) if build.is_dir() else []
    hits = [p for p in hits if p.is_file()]
    if not hits:
        raise FileNotFoundError(f"{LIB_NAME} not found under {build}; run scripts/build_locate_anything.sh")
    return hits[0]


def _bind(lib: ctypes.CDLL) -> None:
    c_ctx = ctypes.c_void_p
    lib.la_capi_abi_version.argtypes = []
    lib.la_capi_abi_version.restype = ctypes.c_int
    lib.la_capi_load.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.la_capi_load.restype = c_ctx
    lib.la_capi_free.argtypes = [c_ctx]
    lib.la_capi_free.restype = None
    # c_void_p (not c_char_p) so we keep the pointer and can free it ourselves.
    lib.la_capi_locate_buffer.argtypes = [c_ctx, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_int]
    lib.la_capi_locate_buffer.restype = ctypes.c_void_p
    lib.la_capi_free_string.argtypes = [ctypes.c_void_p]
    lib.la_capi_free_string.restype = None
    lib.la_capi_last_error.argtypes = [c_ctx]
    lib.la_capi_last_error.restype = ctypes.c_char_p


class LocateEngine:
    def __init__(self, model_path: str | Path, lib_path: str | Path, n_threads: int = 0):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"model not found: {model_path}; run scripts/get_model.sh")
        self._lock = threading.Lock()
        self._ctx = None
        self._lib = ctypes.CDLL(str(lib_path))
        _bind(self._lib)
        abi = self._lib.la_capi_abi_version()
        if abi != ABI_VERSION:
            raise EngineError(f"engine C ABI version {abi}, expected {ABI_VERSION}; rebuild the pinned engine")
        self._ctx = self._lib.la_capi_load(str(model_path).encode(), int(n_threads))
        if not self._ctx:
            raise EngineError(f"failed to load model {model_path}")

    def locate(self, image: bytes, prompt: str, mode: str = "hybrid") -> list[Detection]:
        """Run one detection. ``prompt`` is the full prompt (see ``postprocess.build_prompt``)."""
        if mode not in MODES:
            raise ValueError(f"mode must be one of {sorted(MODES)}, got {mode!r}")
        with self._lock:
            if not self._ctx:
                raise EngineError("engine is closed")
            ptr = self._lib.la_capi_locate_buffer(self._ctx, image, len(image), prompt.encode(), MODES[mode])
            if not ptr:
                err = (self._lib.la_capi_last_error(self._ctx) or b"").decode(errors="replace")
                raise EngineError(f"locate failed: {err or 'unknown error'}")
            try:
                raw = ctypes.string_at(ptr).decode()
            finally:
                self._lib.la_capi_free_string(ptr)
        return parse_detections(raw)

    def close(self) -> None:
        with self._lock:
            if self._ctx:
                self._lib.la_capi_free(self._ctx)
                self._ctx = None

    def __enter__(self) -> "LocateEngine":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self) -> None:
        if getattr(self, "_ctx", None):
            self.close()
