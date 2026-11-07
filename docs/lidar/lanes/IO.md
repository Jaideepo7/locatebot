# Lane lidar/io — RPLIDAR driver and scan logs

Files: `locatebot/lidar/rplidar.py`, `locatebot/lidar/scanlog.py`, `tools/record_scan.py`, `tests/lidar/test_rplidar.py`, `tests/lidar/test_scanlog.py`.

## RPLIDAR protocol (standard SCAN mode; C1 default baud 460800, A1M8 115200)
- Request: `0xA5, cmd` (+ size/payload/checksum for commands with payload; none needed here). Commands: STOP `0x25`, RESET `0x40`, SCAN `0x20`, GET_INFO `0x50`, GET_HEALTH `0x52`.
- Response descriptor (7 bytes): `0xA5 0x5A`, then a little-endian u32 whose low 30 bits are the response length and top 2 bits the send mode, then a data-type byte. SCAN answers length 5, mode 1 (multiple), type `0x81`.
- Scan node (5 bytes): byte0 = `quality<<2 | (!S)<<1 | S` (S = start of new revolution, so bits 0 and 1 must differ); byte1 = `angle_q6[6:0]<<1 | C` with check bit C == 1; byte2 = `angle_q6[14:7]`; bytes3-4 = little-endian `distance_q2`. angle_deg = angle_q6 / 64, distance_mm = distance_q2 / 4; distance 0 means no return.
- GET_HEALTH data (3 bytes): status u8 (0 good, 1 warning, 2 error), error code u16 LE. GET_INFO data (20 bytes): model u8, firmware minor u8, firmware major u8, hardware u8, serial 16 bytes.
- The sensor's angles increase **clockwise** seen from above. Convert to the robot frame with `wrap_angle(-radians(angle_deg) + mount_offset_rad)`.
- This protocol summary is from the public SLAMTEC protocol docs and has not been checked against a real C1. Keep the parser defensive and say so in the module docstring.

## API (exact)
```python
@dataclass(frozen=True)
class Node: angle_deg: float; distance_mm: float; quality: int; start: bool

def build_request(cmd: int) -> bytes
def parse_descriptor(buf: bytes) -> tuple[int, int, int]          # (length, mode, data_type); ValueError if bad sync
def parse_scan_nodes(buf: bytes | bytearray) -> tuple[list[Node], int]  # (nodes, bytes_consumed); skips 1 byte at a time on a bad node so it resyncs
class ScanAssembler:
    def __init__(self, mount_offset_rad: float = 0.0, min_points: int = 50, clock: Callable[[], float] = time.monotonic): ...
    def feed(self, nodes: Iterable[Node]) -> list[Scan]   # emits a Scan each time a start node closes a revolution with >= min_points nodes; seq increments; distance 0 -> NaN
class RPLidar:
    def __init__(self, port: str, baudrate: int = 460800, timeout_s: float = 1.0, transport=None, mount_offset_rad: float = 0.0): ...
    # transport: object with read(n) -> bytes, write(b), reset_input_buffer(), close(); default opens pyserial lazily
    def get_health(self) -> tuple[int, int]
    def get_info(self) -> dict
    def start_scan(self) -> None
    def stop(self) -> None
    def iter_scans(self, max_scans: int | None = None) -> Iterator[Scan]
    def close(self) -> None   # also context manager
```
The first, partial revolution after `start_scan` must be discarded (no start node seen yet).

## Scan log format `.lbscan`
File magic `b"LBSCAN1\0"`, then records: header `struct "<dqI"` (stamp_s, seq, n), then `n` float32 angles, `n` float32 ranges (NaN kept), `n` uint8 quality, then u32 CRC32 of header+payload.
```python
class ScanLogWriter:  # context manager
    def __init__(self, path): ...
    def write(self, scan: Scan) -> None   # flush per record
def read_scan_log(path) -> Iterator[Scan]   # stops cleanly at a truncated last record (power loss); raises ValueError on bad magic or CRC mismatch
```
`tools/record_scan.py`: `--port`, `--baud` (default 460800), `--out`, `--seconds`, `--mount-offset-deg`; prints health/info, scans/second and count at exit; Ctrl+C stops the motor and closes the file cleanly.

## Required tests
- Node parse: angle/distance/quality decoding from hand-built bytes; check-bit and S/!S violations are skipped and the parser resyncs mid-stream.
- Descriptor parse for the SCAN descriptor; bad sync raises.
- Assembler: partial first revolution discarded, revolution split on start flag, clockwise→CCW conversion and mount offset, distance 0 → NaN, min_points respected.
- RPLidar with a fake transport fed bytes generated from `sim.simulate_scan` output: `iter_scans` yields scans matching the simulated ranges within the q2/q6 quantisation; `stop()` writes `A5 25`.
- Scan log round trip (float32 tolerance, NaN preserved), truncated tail tolerated, CRC corruption detected.

## Rules for every lane
- Work only in your worktree and only in the files listed for your lane. `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `pyproject.toml` and this README belong to the integrator; if you need a change there, stop and report it.
- Python >= 3.10, numpy only (pyserial allowed lazily inside the RPLIDAR driver). No other dependencies, no network, no hardware access.
- Test first: write the tests, run them and see them fail, implement, then run the **whole** suite: `PYTHONPATH=. /opt/data/projects/locatebot-venv/bin/python -m pytest -q`.
- Use `locatebot.lidar.sim` for synthetic rooms/scans. Synthetic tests prove software behaviour, not real-sensor behaviour; say so in docstrings where it matters.
- Vectorise with numpy; no per-beam Python loops in hot paths unless the brief allows it.
- Commit only your files with a conventional message (`feat(lidar-<lane>): ...`). Do not merge, push, or touch other branches.
