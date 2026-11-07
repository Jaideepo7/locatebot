"""`.lbscan` scan logs: record and replay Scans so the team can develop without the car.

Format: magic b"LBSCAN1\\0", then records of header struct "<dqI" (stamp_s, seq, n),
n float32 angles, n float32 ranges (NaN kept), n uint8 quality, and a u32 CRC32 of
header+payload. A truncated last record (power loss) ends reading cleanly.
"""
from __future__ import annotations

import struct
import zlib
from typing import Iterator

import numpy as np

from .types import Scan

MAGIC = b"LBSCAN1\0"
_HEADER = struct.Struct("<dqI")
_CRC = struct.Struct("<I")


class ScanLogWriter:
    def __init__(self, path):
        self._f = open(path, "wb")
        self._f.write(MAGIC)
        self._f.flush()

    def write(self, scan: Scan) -> None:
        n = len(scan)
        rec = (_HEADER.pack(float(scan.stamp_s), int(scan.seq), n)
               + np.asarray(scan.angles_rad, dtype="<f4").tobytes()
               + np.asarray(scan.ranges_m, dtype="<f4").tobytes()
               + np.asarray(scan.quality, dtype=np.uint8).tobytes())
        self._f.write(rec + _CRC.pack(zlib.crc32(rec)))
        self._f.flush()

    def close(self) -> None:
        if not self._f.closed:
            self._f.close()

    def __enter__(self) -> "ScanLogWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_scan_log(path) -> Iterator[Scan]:
    """Yield Scans; stop at a truncated tail; ValueError on bad magic or CRC mismatch."""
    with open(path, "rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise ValueError(f"{path}: not an .lbscan file (bad magic)")
        while True:
            header = f.read(_HEADER.size)
            if len(header) < _HEADER.size:
                return  # clean EOF or truncated header
            stamp, seq, n = _HEADER.unpack(header)
            size = 9 * n
            body = f.read(size + _CRC.size)
            if len(body) < size + _CRC.size:
                return  # truncated record
            payload = body[:size]
            (crc,) = _CRC.unpack(body[size:])
            if zlib.crc32(header + payload) != crc:
                raise ValueError(f"{path}: CRC mismatch in record seq={seq}")
            angles = np.frombuffer(payload, dtype="<f4", count=n, offset=0)
            ranges = np.frombuffer(payload, dtype="<f4", count=n, offset=4 * n)
            quality = np.frombuffer(payload, dtype=np.uint8, count=n, offset=8 * n)
            yield Scan(stamp_s=stamp, seq=seq, angles_rad=angles, ranges_m=ranges, quality=quality)
