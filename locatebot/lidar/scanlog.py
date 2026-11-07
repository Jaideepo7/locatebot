"""`.lbscan` scan logs: record and replay Scans so the team can develop without the car.

Format (v2): magic b"LBSCAN2\\0", then records of header struct "<dqI" (stamp_s, seq, n),
a u32 CRC32 of those 20 header bytes, n float32 angles, n float32 ranges (NaN kept),
n uint8 quality, and a u32 CRC32 of header (20 bytes) + payload. n <= MAX_BEAMS.
A record cut off at end-of-file (power loss) ends reading cleanly; bad magic (including the
old LBSCAN1 format), a bad header CRC, n > MAX_BEAMS or a bad payload CRC raise ValueError.
"""
from __future__ import annotations

import struct
import zlib
from typing import Iterator

import numpy as np

from .types import Scan

MAGIC = b"LBSCAN2\0"
_OLD_MAGIC = b"LBSCAN1\0"
MAX_BEAMS = 20000
_HEADER = struct.Struct("<dqI")
_CRC = struct.Struct("<I")


class ScanLogWriter:
    def __init__(self, path):
        self._f = open(path, "wb")
        self._f.write(MAGIC)
        self._f.flush()

    def write(self, scan: Scan) -> None:
        n = len(scan)
        if n > MAX_BEAMS:
            raise ValueError(f"scan has {n} beams; .lbscan allows at most MAX_BEAMS={MAX_BEAMS}")
        header = _HEADER.pack(float(scan.stamp_s), int(scan.seq), n)
        rec = (header + _CRC.pack(zlib.crc32(header))
               + np.asarray(scan.angles_rad, dtype="<f4").tobytes()
               + np.asarray(scan.ranges_m, dtype="<f4").tobytes()
               + np.asarray(scan.quality, dtype=np.uint8).tobytes())
        payload = rec[_HEADER.size + _CRC.size:]
        self._f.write(rec + _CRC.pack(zlib.crc32(header + payload)))
        self._f.flush()

    def close(self) -> None:
        if not self._f.closed:
            self._f.close()

    def __enter__(self) -> "ScanLogWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_scan_log(path) -> Iterator[Scan]:
    """Yield Scans; stop at a record cut off at EOF; ValueError on bad magic, header CRC,
    n > MAX_BEAMS or payload CRC mismatch."""
    with open(path, "rb") as f:
        magic = f.read(len(MAGIC))
        if magic == _OLD_MAGIC:
            raise ValueError(f"{path}: old LBSCAN1 format (no header CRC) is no longer supported; re-record it")
        if magic != MAGIC:
            raise ValueError(f"{path}: not an .lbscan file (bad magic)")
        hsize = _HEADER.size + _CRC.size
        while True:
            head = f.read(hsize)
            if len(head) < hsize:
                return  # clean EOF or truncated header
            header = head[:_HEADER.size]
            (hcrc,) = _CRC.unpack(head[_HEADER.size:])
            if zlib.crc32(header) != hcrc:
                raise ValueError(f"{path}: header CRC mismatch at offset {f.tell() - hsize}")
            stamp, seq, n = _HEADER.unpack(header)
            if n > MAX_BEAMS:
                raise ValueError(f"{path}: record seq={seq} has n={n} > MAX_BEAMS={MAX_BEAMS}")
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
