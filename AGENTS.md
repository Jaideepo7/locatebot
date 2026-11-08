# Agent instructions for locatebot

<!-- omh:agent-instructions:begin -->
Read `docs/PROJECT.md` first. It is the source of truth for architecture, status and decisions. The numbered docs `docs/00-*` to `docs/08-*` predate the Pi + Orin design and the code; where they disagree, `docs/PROJECT.md` wins.

## What this repo is
An RC car (Raspberry Pi 4 + Jetson Orin Nano Super 8 GB + RPLIDAR C1) that maps a room with LiDAR, saves a `.lbmap` session map, then searches the map for a user-given item list with LocateAnything (q4_k GGUF via locate-anything.cpp). Python package: `locatebot/`. Subsystem docs: `docs/lidar/README.md`, `docs/vision/README.md`.

## Build and test
Run from the repo root.

| Command | What it does | Status |
|---|---|---|
| `python3 -m venv .venv && .venv/bin/pip install -e '.[hardware,dev,link]'` (or `uv pip install --python .venv/bin/python -e '.[hardware,dev,link]'`) | Install package + pyserial + pyzmq + pytest | verified (exit 0, 2026-11-08, x86 Linux, Python 3.14) |
| `.venv/bin/python -m pytest -q` | Full test suite, no hardware needed | verified (exit 0, 2026-11-08) |
| `.venv/bin/python tools/build_map.py --demo --out maps/demo --size 16` | Synthetic exploration → `maps/demo.lbmap/.pgm/.yaml` | verified (exit 0, 2026-11-08) |
| `.venv/bin/python tools/build_map.py --log <file>.lbscan --out maps/<name>` | Session map from a recorded scan log | verified via test suite only (synthetic log) |
| `.venv/bin/python tools/record_scan.py --port /dev/ttyUSB0 --out recordings/<name>.lbscan --seconds 60` | Record from a real RPLIDAR (C1 default 460800 baud; A1M8 needs `--baud 115200`) | unverified: never run against real hardware |
| `scripts/get_model.sh` | Download pinned q4_k model to `models/`, resumable, SHA-256 checked | verified (exit 0, 2026-11-08, x86) |
| `scripts/build_locate_anything.sh` | Build pinned locate-anything.cpp (CLI + `liblocate_anything.so`) into `third_party/`, applying `scripts/patches/` | verified CPU on x86 (2026-11-07) and CUDA on the Orin (2026-11-08) |

## Generated and downloaded files: never commit
| Path | Source | Regenerate |
|---|---|---|
| `models/locate-anything-q4_k.gguf` | Hugging Face `mudler/locate-anything.cpp-gguf` @ pinned revision in `scripts/get_model.sh` | `scripts/get_model.sh` (checks size + SHA-256) |
| `third_party/locate-anything.cpp/` | GitHub `mudler/locate-anything.cpp` @ pinned commit in `scripts/build_locate_anything.sh` | `scripts/build_locate_anything.sh` |
| `maps/*.lbmap/.pgm/.yaml`, `recordings/*.lbscan` | `tools/build_map.py`, `tools/record_scan.py` | Re-run the tool |

Model weights are NVIDIA-licensed (research/non-commercial). Never add them to git, LFS or any upload.

## Ownership
- Integrator-owned shared contracts: `locatebot/lidar/types.py`, `locatebot/lidar/sim.py`, `locatebot/mission/types.py`, `pyproject.toml`, `docs/PROJECT.md`. Don't change them inside a lane; propose the change instead.
- Lane modules import only shared contracts, never each other's modules, so branches merge without conflicts.
- Branch per lane, merge into an integration branch after an independent review, then fast-forward `main`.

## Invariants: do not break
- Robot frame: +x forward, +y left, angles counter-clockwise, radians, metres, monotonic seconds.
- `NaN` range means **unknown, never free**. `corridor_clearance` returns `inf` when nothing is seen; anything that stops the car must also check `forward_coverage`.
- Vision never blocks driving: model calls run in a background worker on the newest frame only.
- The Pi's LiDAR safety stop is the final gate on motion, above manual drive and mission commands.
- No internet at runtime: no CDN assets, no cloud services, no downloads after setup.
- The RPLIDAR driver must stop the motor on every exit path and time out on a stalled stream.

## Working rules for agents
- Test first: write the failing test, see it fail, implement, run the **whole** suite.
- No hardware in tests: use `locatebot.lidar.sim` and fake transports/engines.
- Synthetic tests prove software behaviour, not real-sensor behaviour or physical safety. Never claim otherwise.
- Commit only your lane's files with a conventional message (`feat(lidar-detect): …`, `fix(vision): …`).
- Don't push, merge or install new dependencies unless the task says so. Never put credentials, tokens or private hostnames in the repo.
- Report the commit SHA, the real test output, deviations from the spec, and known limitations.

## Pitfalls already hit
- **Branch names:** a branch `lidar` blocks creating `lidar/io` (git ref conflict). The integration branch is `lidar-integration`.
- **Spike filtering:** an early `clean_scan` removed single-beam close returns, deleting thin obstacles (a pole at 1 m vanished). Only remove points *farther* than both neighbours.
- **Coverage:** counting valid beams reported 100 % coverage with a blind wedge ahead. Coverage is measured over angular bins.
- **Corrupted scan logs:** without a header CRC, one flipped bit made a log read as empty. `.lbscan` is now `LBSCAN2` with a header CRC; old `LBSCAN1` files are rejected.
- **Map names:** `Path.with_suffix` cut dotted names (`2026.11.07` → `2026.11.lbmap`). `Mapper.save` appends suffixes instead.
- **Prompts:** `remote control` found nothing where `remote` found both remotes, and `a</c>b` prompts found only the first item. Test each item's wording; prefer one item per call.
- **Model speed:** about 50 s per photo on an x86 laptop CPU. On the Orin GPU about 2.1 s per item at 640 px; 1600 px frames run out of GPU memory. Send 640 px frames.
- **Orin GPU memory:** the unpatched engine keeps a host copy of all weights and fails to load on the Orin's shared memory; keep `scripts/patches/locate-anything-streamed-weights.patch`.
- **Orin power:** the Orin Nano dev kit can't be powered over USB-C; use the barrel jack.
<!-- omh:agent-instructions:end -->
