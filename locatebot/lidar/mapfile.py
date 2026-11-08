"""The per-session ``.lbmap`` map file and ROS ``PGM + YAML`` export.

Layout (little-endian)::

    magic   8 bytes  b"LBMAP1\\0\\0"
    header  struct "<HdddIII": version=1, resolution_m, origin_x, origin_y, width, height, meta_len
    meta    meta_len bytes of UTF-8 JSON
    cells   width*height float32 log-odds, row-major (row 0 = lowest y)
    crc     u32 CRC32 of everything before it

Grid update parameters (l_occ, l_free, clamps) are not stored; loaded grids use the defaults.

``LBMAP2`` (M2) adds named layers. Layout (little-endian)::

    magic   8 bytes  b"LBMAP2\0\0"
    header  struct "<HdddIIIH": version=2, resolution_m, origin_x, origin_y, width, height,
            meta_len, n_layers (>= 1)
    meta    meta_len bytes of UTF-8 JSON
    table   n_layers entries, each:
              struct "<H" name_len, then name_len bytes UTF-8 name
              struct "<Bdd" dtype code, scale, offset   (value = stored * scale + offset)
              struct "<I" params_len, then params_len bytes UTF-8 JSON object (update rule, units)
    blobs   per layer in table order: width*height cells of its dtype, row-major (row 0 = lowest y)
    crc     u32 CRC32 of everything before it

dtype codes: 1 float32, 2 float16, 3 uint8, 4 int16, 5 uint16. Layer 0 is always ``static``
(float32 log-odds, scale 1, offset 0): the room grid; ``load_map`` returns it, so v2 files load
wherever v1 files do. ``save_map`` writes v1 unless ``layers`` are given.
"""
from __future__ import annotations

import json
import os
import struct
import tempfile
import zlib
from dataclasses import dataclass, field

import numpy as np

from .grid import FREE, OCCUPIED, OccupancyGrid

MAGIC = b"LBMAP1\0\0"
VERSION = 1
_HDR = struct.Struct("<HdddIII")
_CRC = struct.Struct("<I")

MAGIC2 = b"LBMAP2\0\0"
VERSION2 = 2
_HDR2 = struct.Struct("<HdddIIIH")
_U16 = struct.Struct("<H")
_U32 = struct.Struct("<I")
_LAYER = struct.Struct("<Bdd")
_DTYPES = {1: np.dtype("<f4"), 2: np.dtype("<f2"), 3: np.dtype("u1"), 4: np.dtype("<i2"), 5: np.dtype("<u2")}
STATIC_LAYER = "static"


@dataclass
class MapLayer:
    """One named map layer: ``data`` (H, W) in its stored dtype; physical value =
    ``data * scale + offset``; ``params`` = JSON-able update rule / units / thresholds."""

    data: np.ndarray
    scale: float = 1.0
    offset: float = 0.0
    params: dict = field(default_factory=dict)

    def values(self) -> np.ndarray:
        return self.data.astype(np.float32) * np.float32(self.scale) + np.float32(self.offset)


class MapFileError(ValueError):
    """The file is not a valid .lbmap (bad magic, version, size, CRC or metadata)."""


def _atomic_write(path, data: bytes) -> None:
    path = os.fspath(path)
    d = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(dir=d, prefix="." + os.path.basename(path) + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        raise


def _save_v1(path, grid: OccupancyGrid, meta: dict | None = None) -> None:
    meta_b = json.dumps({} if meta is None else meta, ensure_ascii=False).encode("utf-8")
    h, w = grid.log_odds.shape
    head = MAGIC + _HDR.pack(VERSION, grid.resolution_m, grid.origin_xy[0], grid.origin_xy[1], w, h, len(meta_b))
    body = head + meta_b + np.ascontiguousarray(grid.log_odds, dtype="<f4").tobytes()
    _atomic_write(path, body + _CRC.pack(zlib.crc32(body)))


def _dtype_code(dt: np.dtype) -> int:
    dt = np.dtype(dt)
    for code, d in _DTYPES.items():
        if dt.kind == d.kind and dt.itemsize == d.itemsize:
            return code
    raise ValueError(f"unsupported layer dtype {dt}")


def save_map(path, grid: OccupancyGrid, meta: dict | None = None, layers: dict | None = None) -> None:
    """Write grid + meta to path atomically (temp file in the same dir, then os.replace).

    Without ``layers`` this writes ``LBMAP1`` exactly as before. With ``layers`` (name ->
    ``MapLayer`` or (H, W) array) it writes ``LBMAP2``: the grid as layer ``static`` first, then
    the extra layers in the given order.
    """
    if not layers:
        return _save_v1(path, grid, meta)
    h, w = grid.log_odds.shape
    meta_b = json.dumps({} if meta is None else meta, ensure_ascii=False).encode("utf-8")
    entries = [(STATIC_LAYER, MapLayer(grid.log_odds.astype(np.float32)))]
    for name, lay in layers.items():
        if name == STATIC_LAYER:
            raise ValueError("layer name 'static' is reserved for the grid")
        if not isinstance(lay, MapLayer):
            lay = MapLayer(np.asarray(lay))
        if lay.data.shape != (h, w):
            raise ValueError(f"layer {name!r} has shape {lay.data.shape}, grid is {(h, w)}")
        entries.append((str(name), lay))
    parts = [MAGIC2, _HDR2.pack(VERSION2, grid.resolution_m, grid.origin_xy[0], grid.origin_xy[1], w, h,
                                len(meta_b), len(entries)), meta_b]
    for name, lay in entries:
        nb = name.encode("utf-8")
        pb = json.dumps(lay.params or {}, ensure_ascii=False).encode("utf-8")
        if not nb or len(nb) > 0xFFFF:
            raise ValueError("layer names must be 1..65535 bytes")
        parts += [_U16.pack(len(nb)), nb, _LAYER.pack(_dtype_code(lay.data.dtype), float(lay.scale), float(lay.offset)),
                  _U32.pack(len(pb)), pb]
    for _, lay in entries:
        parts.append(np.ascontiguousarray(lay.data, dtype=_DTYPES[_dtype_code(lay.data.dtype)]).tobytes())
    body = b"".join(parts)
    _atomic_write(path, body + _CRC.pack(zlib.crc32(body)))


def load_map(path) -> tuple[OccupancyGrid, dict]:
    """Read a .lbmap (v1, or v2's static layer); raises MapFileError on any format problem."""
    grid, meta, _ = load_map_layers(path)
    return grid, meta


def load_map_layers(path) -> tuple[OccupancyGrid, dict, dict[str, MapLayer]]:
    """Read a .lbmap with its extra layers (``{}`` for v1 files and for v2 with only static)."""
    with open(path, "rb") as f:
        data = f.read()
    if data[: len(MAGIC2)] == MAGIC2:
        return _load_v2(data)
    grid, meta = _load_v1(data)
    return grid, meta, {}


def _load_v2(data: bytes) -> tuple[OccupancyGrid, dict, dict[str, MapLayer]]:
    fixed = len(MAGIC2) + _HDR2.size
    if len(data) < fixed + _CRC.size:
        raise MapFileError(f"file too short ({len(data)} bytes)")
    (crc,) = _CRC.unpack_from(data, len(data) - _CRC.size)
    if crc != zlib.crc32(data[: -_CRC.size]):
        raise MapFileError("CRC mismatch")
    version, res, ox, oy, w, h, meta_len, n_layers = _HDR2.unpack_from(data, len(MAGIC2))
    if version != VERSION2:
        raise MapFileError(f"unsupported version {version}")
    if w == 0 or h == 0 or not res > 0 or n_layers < 1:
        raise MapFileError("invalid grid dimensions, resolution or layer count")
    end = len(data) - _CRC.size
    off = fixed

    def take(n: int) -> bytes:
        nonlocal off
        if off + n > end:
            raise MapFileError("truncated layer table")
        b = data[off:off + n]
        off += n
        return b

    try:
        meta = json.loads(take(meta_len).decode("utf-8"))
        table = []
        for _ in range(n_layers):
            (nl,) = _U16.unpack(take(_U16.size))
            name = take(nl).decode("utf-8")
            code, scale, offset = _LAYER.unpack(take(_LAYER.size))
            (pl,) = _U32.unpack(take(_U32.size))
            params = json.loads(take(pl).decode("utf-8"))
            if code not in _DTYPES or not isinstance(params, dict):
                raise MapFileError(f"layer {name!r}: bad dtype code or params")
            table.append((name, code, scale, offset, params))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise MapFileError(f"bad metadata: {e}") from e
    if not isinstance(meta, dict):
        raise MapFileError("metadata is not a JSON object")
    if table[0][0] != STATIC_LAYER or table[0][1] != 1:
        raise MapFileError("layer 0 must be the float32 'static' layer")
    need = sum(w * h * _DTYPES[c].itemsize for _, c, _, _, _ in table)
    if end - off != need:
        raise MapFileError(f"size mismatch: layers need {need} bytes, got {end - off}")
    grid = OccupancyGrid(w, h, res, (ox, oy))
    layers: dict[str, MapLayer] = {}
    for k, (name, code, scale, offset, params) in enumerate(table):
        dt = _DTYPES[code]
        arr = np.frombuffer(data, dtype=dt, count=w * h, offset=off).reshape(h, w)
        off += w * h * dt.itemsize
        if k == 0:
            grid.log_odds[:] = arr  # static: the room grid (always float32 log-odds)
            continue
        if name == STATIC_LAYER or name in layers:
            raise MapFileError(f"duplicate layer {name!r}")
        layers[name] = MapLayer(arr.astype(dt.newbyteorder("=")), scale, offset, params)
    return grid, meta, layers


def _load_v1(data: bytes) -> tuple[OccupancyGrid, dict]:
    fixed = len(MAGIC) + _HDR.size
    if len(data) < fixed + _CRC.size:
        raise MapFileError(f"file too short ({len(data)} bytes)")
    if data[: len(MAGIC)] != MAGIC:
        raise MapFileError("bad magic")
    version, res, ox, oy, w, h, meta_len = _HDR.unpack_from(data, len(MAGIC))
    if version != VERSION:
        raise MapFileError(f"unsupported version {version}")
    expected = fixed + meta_len + w * h * 4 + _CRC.size
    if len(data) != expected:
        raise MapFileError(f"size mismatch: expected {expected} bytes, got {len(data)}")
    (crc,) = _CRC.unpack_from(data, len(data) - _CRC.size)
    if crc != zlib.crc32(data[: -_CRC.size]):
        raise MapFileError("CRC mismatch")
    if w == 0 or h == 0 or not res > 0:
        raise MapFileError("invalid grid dimensions or resolution")
    try:
        meta = json.loads(data[fixed : fixed + meta_len].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise MapFileError(f"bad metadata: {e}") from e
    if not isinstance(meta, dict):
        raise MapFileError("metadata is not a JSON object")
    grid = OccupancyGrid(w, h, res, (ox, oy))
    off = fixed + meta_len
    grid.log_odds[:] = np.frombuffer(data, dtype="<f4", count=w * h, offset=off).reshape(h, w)
    return grid, meta


def export_ros_map(grid: OccupancyGrid, pgm_path, yaml_path, occ_threshold=0.65, free_threshold=0.196) -> None:
    """ROS map_server style export: 8-bit P5 PGM (image row 0 = highest y) + YAML.

    Pixels: free 254, occupied 0, unknown 205.
    """
    cls = grid.classify(occ_threshold, free_threshold)
    img = np.full(cls.shape, 205, dtype=np.uint8)
    img[cls == FREE] = 254
    img[cls == OCCUPIED] = 0
    img = np.flipud(img)
    h, w = img.shape
    _atomic_write(pgm_path, f"P5\n{w} {h}\n255\n".encode("ascii") + img.tobytes())
    image = os.path.relpath(os.path.abspath(pgm_path), os.path.dirname(os.path.abspath(yaml_path)))
    yaml = (
        f"image: {image}\n"
        f"resolution: {grid.resolution_m!r}\n"
        f"origin: [{grid.origin_xy[0]!r}, {grid.origin_xy[1]!r}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: {float(occ_threshold)!r}\n"
        f"free_thresh: {float(free_threshold)!r}\n"
    )
    _atomic_write(yaml_path, yaml.encode("utf-8"))
