"""RPLIDAR serial driver (standard SCAN mode) for the C1, with A1M8 as a fallback.

Written from the public SLAMTEC protocol documentation and tested only against synthetic
byte streams; it has NOT been checked against a real C1. The parser is therefore defensive:
malformed nodes (check bit clear, S == !S, angle >= 360 deg) are skipped one byte at a time
so the stream resyncs, and the first partial revolution after start_scan is discarded.

Safety (the scans feed an emergency stop): iter_scans raises TimeoutError when no complete
revolution arrives within scan_timeout_s even if bytes keep flowing; STOP is sent on every
exit from iter_scans and by close() while scanning; every command that expects a response
descriptor first sends STOP and flushes, so a sensor left streaming by a crash is recovered.

Sensor angles increase clockwise seen from above; scans are converted to the robot frame
(+x forward, CCW positive) with wrap_angle(-radians(angle_deg) + mount_offset_rad).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Iterator

import numpy as np

from .types import Scan

SYNC_A = 0xA5
SYNC_B = 0x5A
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

TYPE_SCAN = 0x81
TYPE_INFO = 0x04
TYPE_HEALTH = 0x06
MODE_SINGLE = 0
MODE_MULTI = 1

DESCRIPTOR_LEN = 7
NODE_LEN = 5
_ANGLE_Q6_MAX = 360 * 64
_READ_CHUNK = NODE_LEN * 64


@dataclass(frozen=True)
class Node:
    angle_deg: float
    distance_mm: float
    quality: int
    start: bool


def build_request(cmd: int) -> bytes:
    """Request packet for a command without payload."""
    return bytes((SYNC_A, cmd & 0xFF))


def parse_descriptor(buf: bytes) -> tuple[int, int, int]:
    """Parse a 7-byte response descriptor into (length, mode, data_type)."""
    if len(buf) < DESCRIPTOR_LEN:
        raise ValueError(f"descriptor too short: {len(buf)} bytes")
    if buf[0] != SYNC_A or buf[1] != SYNC_B:
        raise ValueError(f"bad descriptor sync: {bytes(buf[:2]).hex()}")
    word = int.from_bytes(bytes(buf[2:6]), "little")
    return word & 0x3FFFFFFF, word >> 30, buf[6]


def _node_ok(b0: int, b1: int, b2: int) -> bool:
    if (b0 & 1) == ((b0 >> 1) & 1):  # S and !S must differ
        return False
    if not (b1 & 1):  # check bit
        return False
    return ((b2 << 7) | (b1 >> 1)) < _ANGLE_Q6_MAX


def parse_scan_nodes(buf: bytes | bytearray) -> tuple[list[Node], int]:
    """Decode as many 5-byte scan nodes as possible.

    Returns (nodes, bytes_consumed). A bad node advances one byte so the parser resyncs;
    a trailing incomplete node is left unconsumed for the next call.
    """
    mv = memoryview(bytes(buf))
    n = len(mv)
    nodes: list[Node] = []
    i = 0
    while i + NODE_LEN <= n:
        b0, b1, b2, b3, b4 = mv[i], mv[i + 1], mv[i + 2], mv[i + 3], mv[i + 4]
        if not _node_ok(b0, b1, b2):
            i += 1
            continue
        angle_q6 = (b2 << 7) | (b1 >> 1)
        dist_q2 = b3 | (b4 << 8)
        nodes.append(Node(angle_q6 / 64.0, dist_q2 / 4.0, b0 >> 2, bool(b0 & 1)))
        i += NODE_LEN
    return nodes, i


class ScanAssembler:
    """Group nodes into revolutions. Synthetic-tested only.

    Each Scan is stamped with clock() taken when its start node was fed (i.e. when the chunk
    holding the revolution's first node was read), not when the revolution was closed.
    A revolution that grows past max_nodes without a start flag is discarded and the assembler
    waits for the next start flag, so the buffer stays bounded on a broken stream.
    """

    def __init__(self, mount_offset_rad: float = 0.0, min_points: int = 50,
                 clock: Callable[[], float] = time.monotonic, max_nodes: int = 10000):
        self.mount_offset_rad = float(mount_offset_rad)
        self.min_points = int(min_points)
        self.clock = clock
        self.max_nodes = int(max_nodes)
        self._seq = 0
        self._seen_start = False
        self._stamp = 0.0
        self._angle: list[float] = []
        self._dist: list[float] = []
        self._qual: list[int] = []

    def _clear(self) -> None:
        self._angle, self._dist, self._qual = [], [], []

    def _close(self) -> Scan | None:
        if len(self._angle) < self.min_points:
            return None
        deg = np.asarray(self._angle, dtype=np.float64)
        mm = np.asarray(self._dist, dtype=np.float64)
        ang = -np.radians(deg) + self.mount_offset_rad
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi
        rng = np.where(mm > 0.0, mm / 1000.0, np.nan)
        scan = Scan(stamp_s=self._stamp, seq=self._seq, angles_rad=ang, ranges_m=rng,
                    quality=np.asarray(self._qual, dtype=np.uint8))
        self._seq += 1
        return scan

    def feed(self, nodes: Iterable[Node]) -> list[Scan]:
        out: list[Scan] = []
        for nd in nodes:
            if nd.start:
                if self._seen_start:
                    s = self._close()
                    if s is not None:
                        out.append(s)
                self._seen_start = True
                self._stamp = float(self.clock())
                self._clear()
            if not self._seen_start:
                continue  # partial first revolution, or waiting after an overflow
            if len(self._angle) >= self.max_nodes:
                self._seen_start = False  # no start flag for too long: drop, wait for next one
                self._clear()
                continue
            self._angle.append(nd.angle_deg)
            self._dist.append(nd.distance_mm)
            self._qual.append(nd.quality)
        return out


class RPLidar:
    """Minimal RPLIDAR driver. `transport` needs read(n), write(b), reset_input_buffer(), close().

    clock and sleep are injectable so tests can drive the watchdog and STOP delays.
    """

    def __init__(self, port: str, baudrate: int = 460800, timeout_s: float = 1.0, transport=None,
                 mount_offset_rad: float = 0.0, clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.mount_offset_rad = mount_offset_rad
        self._clock = clock
        self._sleep = sleep
        if transport is None:
            import serial  # lazy: pyserial is only needed on real hardware

            transport = serial.Serial(port, baudrate=baudrate, timeout=timeout_s)
            try:
                transport.dtr = False  # A1M8 USB adapter: DTR low starts the motor; harmless on C1
            except Exception:  # pragma: no cover - adapter without DTR control
                pass
        self._t = transport
        self._scanning = False
        self._buf = bytearray()

    # -- low level
    def _send(self, cmd: int) -> None:
        self._t.write(build_request(cmd))

    def _read_exact(self, n: int) -> bytes:
        out = bytearray()
        while len(out) < n:
            chunk = self._t.read(n - len(out))
            if not chunk:
                raise TimeoutError(f"lidar sent {len(out)}/{n} bytes before timing out")
            out += chunk
        return bytes(out)

    def _request(self, cmd: int, expect_type: int, expect_mode: int) -> int:
        # The sensor may still be streaming (e.g. after a crash): stop it, let the last
        # in-flight nodes arrive, then flush them so the descriptor read is not misaligned.
        self._send(CMD_STOP)
        self._scanning = False
        self._sleep(0.005)
        self._t.reset_input_buffer()
        self._send(cmd)
        length, mode, dtype = parse_descriptor(self._read_exact(DESCRIPTOR_LEN))
        if dtype != expect_type or mode != expect_mode:
            raise RuntimeError(f"unexpected descriptor for cmd 0x{cmd:02x}: len={length} mode={mode} type=0x{dtype:02x}")
        return length

    # -- commands
    def get_health(self) -> tuple[int, int]:
        length = self._request(CMD_GET_HEALTH, TYPE_HEALTH, MODE_SINGLE)
        data = self._read_exact(length)
        if len(data) < 3:
            raise RuntimeError("health response too short")
        return data[0], int.from_bytes(data[1:3], "little")

    def get_info(self) -> dict:
        length = self._request(CMD_GET_INFO, TYPE_INFO, MODE_SINGLE)
        data = self._read_exact(length)
        if len(data) < 20:
            raise RuntimeError("info response too short")
        return {"model": data[0], "firmware": (data[2], data[1]), "hardware": data[3],
                "serial": data[4:20].hex().upper()}

    def start_scan(self) -> None:
        length = self._request(CMD_SCAN, TYPE_SCAN, MODE_MULTI)
        if length != NODE_LEN:
            raise RuntimeError(f"unexpected scan node length {length}")
        self._buf.clear()
        self._scanning = True

    def stop(self) -> None:
        self._send(CMD_STOP)
        self._scanning = False
        self._sleep(0.002)  # protocol asks for >= 1 ms before the next request

    def iter_scans(self, max_scans: int | None = None, scan_timeout_s: float = 2.0) -> Iterator[Scan]:
        """Yield full revolutions; starts scanning if needed.

        Raises TimeoutError if the port goes silent, or if no complete Scan has been produced
        for scan_timeout_s of wall-clock time while bytes keep arriving (garbage, or nodes that
        never set the start flag). Scanning is stopped (STOP sent) on every exit: max_scans
        reached, generator closed / loop broken, or any exception.
        """
        try:
            if not self._scanning:
                self.start_scan()
            asm = ScanAssembler(mount_offset_rad=self.mount_offset_rad, clock=self._clock)
            count = 0
            last_scan = self._clock()
            while max_scans is None or count < max_scans:
                chunk = self._t.read(_READ_CHUNK)
                if not chunk:
                    raise TimeoutError(f"no scan data from lidar within {self.timeout_s} s")
                self._buf += chunk
                nodes, used = parse_scan_nodes(self._buf)
                del self._buf[:used]
                for scan in asm.feed(nodes):
                    yield scan
                    count += 1
                    last_scan = self._clock()
                    if max_scans is not None and count >= max_scans:
                        return
                if self._clock() - last_scan > scan_timeout_s:
                    raise TimeoutError(f"lidar data arriving but no complete scan within {scan_timeout_s} s")
        finally:
            if self._scanning:
                self.stop()

    def close(self) -> None:
        """Send STOP if scanning, then close the transport."""
        try:
            if self._scanning:
                self.stop()
        finally:
            self._t.close()

    def __enter__(self) -> "RPLidar":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
