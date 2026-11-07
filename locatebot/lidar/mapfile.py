"""The per-session ``.lbmap`` map file and ROS ``PGM + YAML`` export.

Layout (little-endian)::

    magic   8 bytes  b"LBMAP1\\0\\0"
    header  struct "<HdddIII": version=1, resolution_m, origin_x, origin_y, width, height, meta_len
    meta    meta_len bytes of UTF-8 JSON
    cells   width*height float32 log-odds, row-major (row 0 = lowest y)
    crc     u32 CRC32 of everything before it

Grid update parameters (l_occ, l_free, clamps) are not stored; loaded grids use the defaults.
"""
from __future__ import annotations

import json
import os
import struct
import tempfile
import zlib

import numpy as np

from .grid import FREE, OCCUPIED, OccupancyGrid

MAGIC = b"LBMAP1\0\0"
VERSION = 1
_HDR = struct.Struct("<HdddIII")
_CRC = struct.Struct("<I")


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


def save_map(path, grid: OccupancyGrid, meta: dict | None = None) -> None:
    """Write grid + meta to path atomically (temp file in the same dir, then os.replace)."""
    meta_b = json.dumps({} if meta is None else meta, ensure_ascii=False).encode("utf-8")
    h, w = grid.log_odds.shape
    head = MAGIC + _HDR.pack(VERSION, grid.resolution_m, grid.origin_xy[0], grid.origin_xy[1], w, h, len(meta_b))
    body = head + meta_b + np.ascontiguousarray(grid.log_odds, dtype="<f4").tobytes()
    _atomic_write(path, body + _CRC.pack(zlib.crc32(body)))


def load_map(path) -> tuple[OccupancyGrid, dict]:
    """Read a .lbmap; raises MapFileError on any format problem."""
    with open(path, "rb") as f:
        data = f.read()
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
