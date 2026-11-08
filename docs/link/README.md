# Pi <-> Orin link format

**Status:** message format and ZeroMQ transport (`locatebot/link/transport.py`) built and tested over 127.0.0.1/inproc only. Never run on the real Pi <-> Orin cable yet.

`locatebot/link/messages.py`: `encode(msg) -> [topic, header, payload]`, `decode(parts) -> msg`. Three byte frames per message, ready for a ZeroMQ multipart send.

| Topic | Direction | Header fields | Payload |
|---|---|---|---|
| `scan` | Pi → Orin | `stamp_s`, `seq`, `n` | `n` × float32 angles, `n` × float32 ranges (NaN = unknown), `n` × uint8 quality, little-endian (9 bytes per beam) |
| `frame` | Pi → Orin | `capture_s`, `seq` | JPEG bytes (max 4 MiB) |
| `pi_status` | Pi → Orin | `stamp_s`, `lidar_ok`, `scans_per_s`, `camera_ok`, `estop`, `safety_reason`, `throttle`, `steering`, `cpu_temp_c?` | — |
| `range` | Pi → Orin | `stamp_s`, `sensor` (id, e.g. `"front_ultrasonic"`), `range_m?` (≥ 0; `null` = no echo, unknown), `fov_rad` (full cone, (0, π]) | — |
| `mission_request` | Pi → Orin | `mission_id` (id), `target` (1–120 chars, normalised text), `issued_s`, `expires_s` (Unix wall clock) | — |
| `mission_cancel` | Pi → Orin | `mission_id` (id), `reason` (0–64 chars) | — |
| `drive` | Orin → Pi | `throttle`, `steering` (both [-1, 1]), `seq`, `sent_s` | — |
| `estop` | Orin → Pi | `engage`, `seq`, `sent_s` | — |
| `orin_status` | Orin → Pi | `stamp_s`, `vision_alive`, `vision_latency_s?`, `mission_state`, `cpu_temp_c?`, `gpu_temp_c?`, `boot_id` (id, random per Orin process start) | — |
| `mission_status` | Orin → Pi | `mission_id` (id), `state` (one of `MISSION_STATES`), `message` (0–300 chars), `reason` (0–64 chars, e.g. `NOT_FOUND`; empty = none) | — |

Every header also has `"v": 1` and `"type"` (equal to the topic). `?` fields may be `null`; NaN is sent as `null`.

Text fields of the new messages (`range`, `mission_*`, `orin_status.boot_id`) are checked on encode and decode: an *id* is 1–64 characters and not blank; `target` is 1–120 and not blank; `message` ≤ 300 and `reason` ≤ 64 may be empty; none may contain control characters (Unicode category `Cc`: `\x00`–`\x1f`, `\x7f`–`\x9f`, so no newlines or tabs either). `MISSION_STATES` = `PENDING, ACCEPTED, SEARCHING, TARGET_DETECTED, APPROACHING, COMPLETED, FAILED, CANCELLED` (upper case, exact). The link only checks the shape; mission rules (expiry, duplicates, allowed transitions, reason codes) belong to the sender and receiver (R3).

## Clocks
- Scan stamps, frame capture times and Pi status use the **Pi's monotonic clock**, so the Orin can match frames to scans and poses for item localization.
- `range` stamps are the Pi's monotonic clock too, so the Orin can look up the pose at the reading.
- `mission_request.issued_s` / `expires_s` are **Unix wall-clock** seconds (`time.time()`), the one exception: a mission comes from outside the car (e.g. a customer request). The Orin compares them with its own wall clock plus a skew tolerance, so the Pi and Orin need NTP/chrony.
- `sent_s` on drive/estop is the Orin's clock, for latency diagnostics only. The Pi stamps each command with its own clock **on receipt** before passing it to `SafetySupervisor.evaluate`.

## Errors
`decode` raises `ProtocolError` for a wrong frame count, version, type/topic mismatch, missing or wrongly typed fields, bad payload size, or drive values outside [-1, 1]. Drop the message; for drive commands the safety supervisor's command timeout then stops the car. Unknown extra header fields are ignored (forward compatibility).

## Sockets
- Pi PUB `tcp://*:5555`: `scan`, `frame`, `pi_status`, `range`, `mission_request`, `mission_cancel`. Orin SUB; the frame subscriber is newest-only (see below).
- Orin PUB `tcp://*:5556`: `drive`, `estop`, `orin_status`, `mission_status`. Pi SUB.
- Mission topics (`mission_*`) must never be newest-only: every request, cancel and status matters.

## Drive and e-stop rules for the transport (must hold before motors are connected)
Re-stamping on receipt means the command timeout measures time since *arrival*, so the transport must not deliver old commands:
- The Pi's drive subscriber uses `CONFLATE` (or a high-water mark of 1) so a reconnect or backlog can't flush a queue of stale commands.
- Drop any drive command with `seq <= last_seq` (replayed or reordered). Reset `last_seq` only when the Orin's status shows it restarted, i.e. when `orin_status.boot_id` differs from the last one seen (`tools/orin_run.py` sends one `uuid4().hex` per process).
- Optionally drop a command whose estimated one-way latency (from `sent_s` and a measured clock offset) exceeds the command timeout.
- `estop` with `engage=true` latches the Pi's e-stop. `engage=false` from the network **never** releases it; only a local action on the Pi (button or local dashboard command) does.
- Fixed IPs on the direct Ethernet cable; never put real addresses in the repo.

## Transport
`locatebot/link/transport.py` (needs the `[link]` extra, `pyzmq`). Synthetic tests only: `tests/link/test_transport.py`.

| API | Use |
|---|---|
| `Publisher(endpoint, sndhwm=100, linger_s=0.2, heartbeat_s=1.0)` | Binds a PUB. `send(msg) -> bool` encodes and sends with `NOBLOCK`; never blocks. `endpoint` is the resolved address (`tcp://127.0.0.1:*` picks a port). |
| `Subscriber(endpoint, topics, newest_only=(), rcvhwm=100)` | Connects a SUB. `recv(timeout_s)` returns the next message in arrival order or `None`; `poll(timeout_s)` drains the queue and returns a list, keeping only the last message per `newest_only` topic. Counters: `received`, `malformed`, `superseded`. |
| `DriveReceiver(endpoint, drive_hwm=4, estop_hwm=1000)` | Pi side of the Orin PUB. `poll(timeout_s) -> DriveUpdate(drive, estops)`; `reset()`; `last_seq`; counters `dropped_old`, `superseded`, `malformed`. |
| `bind_endpoint(port, interface="*")`, `pi_pub_endpoint(host=None)`, `orin_pub_endpoint(host=None)` | Endpoint strings. Peer hosts come from the argument or `LOCATEBOT_PI_HOST` / `LOCATEBOT_ORIN_HOST`; with neither, `ValueError`. No address is hard-coded. |
| `tools/link_echo.py <endpoint> [--topic T]... [--newest T]... [--seconds S]` | Bench tool: prints decoded messages and the counters. |

All `close()` calls are idempotent and quick (PUB linger 0.2 s, SUB linger 0); every class is a context manager.

### Newest-only without CONFLATE
`ZMQ_CONFLATE` keeps only the last *frame* and does not support multipart messages, so it would break our 3-frame messages. Instead `Subscriber.poll` drains everything queued right now (ZeroMQ delivers multipart messages whole) and keeps the last message of each `newest_only` topic, at its arrival position; other topics are returned in full and in order. A small `RCVHWM` bounds the local queue, and `max_drain` bounds one poll if a sender floods. The Orin's frame subscriber should use `newest_only={"frame"}`.

A full queue (`SNDHWM`/`RCVHWM`) drops the **newest** messages, not the oldest. That is fine because publishers send continuously: the next drain sees fresh messages.

### Drive and e-stop on the Pi
- `DriveReceiver` opens **two** SUB sockets on the Orin endpoint: `drive` with a tiny queue (`drive_hwm=4`) and `estop` with its own large queue. ZeroMQ filters at the publisher per subscriber, so a drive backlog can never fill the e-stop queue.
- Drive: commands with `seq <= last_seq` are dropped (`dropped_old`); of the rest, the highest seq of the poll is returned. After an Orin restart its seq starts low and every command is dropped (so the supervisor's command timeout stops the car) until the Pi calls `reset()`, which it should do when `orin_status.boot_id` changes (the Orin restarted).
- E-stop: every message is returned, never conflated and never seq-filtered. `engage=False` is passed through as data only; the Pi controller must ignore it and release only on a local action.
- The Pi stamps each returned command with its own clock on receipt, then calls `SafetySupervisor.evaluate`.
- Malformed messages (any `decode` failure, or a topic that only prefix-matches a subscription) are dropped and counted, never raised.
- E-stop delivery over PUB/SUB is best effort (no acks). The Orin should repeat `engage=true` until `pi_status.estop` is true; the Pi's command timeout is the guarantee, not the network.

### Dead links
Both sides send ZMTP heartbeats (`heartbeat_s=1.0`, timeout 3x). A peer that stops answering is disconnected, which discards that connection's queued messages; ZeroMQ reconnects and re-subscribes on its own. Data already in the kernel's TCP buffers when the cable comes back is still delivered; the seq rule and newest-only drain keep it from being acted on as a queue, but because a full queue drops the newest messages, the newest buffered drive command can be as old as the outage, bounded by the heartbeat timeout (about 3 s) after which the connection and its queue are dropped. Test this on the real cable. The optional `sent_s` latency check (above) is not implemented yet.

### Threads
ZeroMQ sockets are not thread-safe. Create, use and close each `Publisher`, `Subscriber` and `DriveReceiver` in one thread. The ZeroMQ context (default `zmq.Context.instance()`) is shared and thread-safe.
