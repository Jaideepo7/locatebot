# Storage: PostgreSQL on the Pi 5 (photos, found items, events, missions, maps)

**Status (2026-11-08):** built and tested against a real PostgreSQL 16 (`tests/storage/`, plus the runtime/app wiring with a fake recorder). Not yet run on the Pi 5 or across the Pi↔Orin cable; `deploy/postgres/setup_pi.sh` is untested on a Pi.

## Why PostgreSQL (not SpacetimeDB, not SQLite)
- **SpacetimeDB keeps all data in memory** (plus a commit log). Robot photos are 50–100 KB each and accumulate for the whole session; on the Pi 5, which also runs motor control, the Telegram bridge and the store agent, that is the wrong shape. Its strength (live subscriptions for a browser dashboard) only matters once that dashboard exists, and it needs server modules in TypeScript/Rust.
- **SQLite** is one file on one machine. Our writers and readers are on two machines in two languages (Orin runtime + store agent in Python, the Spectrum Telegram bridge in Node), so it would need a custom server anyway.
- **PostgreSQL** on the Pi 5: one small server (~40–80 MB), standard Python/Node drivers, crash-safe, reachable over the private Ethernet link, aarch64 packages in Debian.

## Design
- **Images in the database** (`image.data`, bytea), deduplicated by SHA-256, JPEG/PNG only, ≤ 4 MiB each. They are produced on the Orin while Postgres runs on the Pi, so this avoids a second file-transfer path. `RobotDB.prune(max_bytes)` (run by the recorder every 5 min, default cap 2 GiB) deletes the oldest images first but **never** an image marked `keep`, an item's best photo or a map's picture.
- **The robot never waits on the database.** `StorageRecorder` puts each write on a bounded queue (2000 ops, 64 MiB of images; beyond that the picture is dropped and the metadata kept) and one writer thread applies them in order. While Postgres is unreachable, writes go to a **disk spool** (`--spool-dir`, default `spool/`, capped at 256 MiB, oldest evicted first, the session record kept) and are replayed in order when it's back, also after a restart. Bad data (e.g. not an image) is logged and dropped, never retried forever.
- **Sessions:** one `robot_session` per `orin_run.py` start, with a uid made on the robot, so spooled writes replay without needing ids from the database.
- Schema migrations are numbered (`locatebot/storage/schema.py`) and applied automatically on connect under an advisory lock.

## Tables
| Table | Rows |
|---|---|
| `robot_session` | uid, start time, host, map name, meta (llm, mode, source) |
| `image` | id, sha256, mime, size, bytes, kind (`sighting`/`event`/`snapshot`/`verify`/`map`), session, capture time, `keep` |
| `sighting` | every vision fix placed on the map: item, x, y, σ, range, bearing, capture time, box, photo |
| `item` | per session and item name: position, σ, sightings, confirmed, last seen, **best photo** (newest sighting with a photo, kept) |
| `event` | the runtime's event log (found X, skill ended, safety stop, localization lost …) |
| `mission`, `mission_status` | Pi/Telegram missions: current state + full history |
| `map_snapshot` | saved maps (`.lbmap` bytes) + a PNG of each |

## What writes what
- `OrinRuntime` (when `orin_run.py --db` is set): every placed sighting with the exact camera frame vision ran on (the runtime keeps the last 12 frames), item upserts once confirmed, every event.
- `OrinApp.save_map` (operator "Save map"): the `.lbmap` and its PNG.
- `orin_run --pi-host`: every `MissionStatus` the mission bridge publishes.
- Operator page: `GET /api/items` (this session's items with photo URLs), `GET /photos/<id>` (the image), and a "Found item photos" panel. No login: keep the page on the team network; photos may show people.

## Run
On the Pi 5 (once):
```bash
sudo LOCATEBOT_DB_PASSWORD='<strong password>' deploy/postgres/setup_pi.sh <pi-ethernet-ip> <orin-ip>
```
On the Orin:
```bash
.venv/bin/pip install -e '.[db]'
printf '%s\n' '<pi-ethernet-ip>:5432:locatebot:locatebot:<password>' >> ~/.pgpass && chmod 600 ~/.pgpass
export LOCATEBOT_DB_URL=postgresql://locatebot@<pi-ethernet-ip>/locatebot
.venv/bin/python tools/orin_run.py --pi-host <pi-ethernet-ip> ...   # or --sim; --db overrides the env var
```
Never put the password in the URL, the repo or a command line.

## Tests
`tests/storage/` needs a real PostgreSQL for the database tests: set `LOCATEBOT_TEST_PG_DSN`, or install `pgserver` in the test environment, or have `initdb`/`pg_ctl` on the machine (Debian/Ubuntu: `/usr/lib/postgresql/<v>/bin`; runs unprivileged in a temp dir). Without one they skip; the recorder's logic tests (fake database) always run. Verified 2026-11-08: 1183 passed with a real PostgreSQL 16.2 (Python 3.12 + pgserver), 1166 passed / database tests skipped in the main venv.

## Limits / next
- Not yet on the Pi 5 or over the cable; measure Postgres RAM there and photo write latency.
- The Node Telegram bridge can read `item`/`image` directly (or use the Orin's `/api/items` + `/photos/<id>`), e.g. to send a found item's photo.
- The store agent's catalog could move into Postgres (`docs/store/README.md` lists "SpacetimeDB catalog" as not built: this replaces that plan).
- Frames sent to cloud models must not contain people; stored photos may. Decide a retention rule for people before the event.
