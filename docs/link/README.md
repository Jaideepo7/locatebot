# Pi <-> Orin link format

**Status:** message format built and tested (encode/decode only). No sockets yet: the ZeroMQ transport needs the `pyzmq` dependency.

`locatebot/link/messages.py`: `encode(msg) -> [topic, header, payload]`, `decode(parts) -> msg`. Three byte frames per message, ready for a ZeroMQ multipart send.

| Topic | Direction | Header fields | Payload |
|---|---|---|---|
| `scan` | Pi → Orin | `stamp_s`, `seq`, `n` | `n` × float32 angles, `n` × float32 ranges (NaN = unknown), `n` × uint8 quality, little-endian (9 bytes per beam) |
| `frame` | Pi → Orin | `capture_s`, `seq` | JPEG bytes (max 4 MiB) |
| `pi_status` | Pi → Orin | `stamp_s`, `lidar_ok`, `scans_per_s`, `camera_ok`, `estop`, `safety_reason`, `throttle`, `steering`, `cpu_temp_c?` | — |
| `drive` | Orin → Pi | `throttle`, `steering` (both [-1, 1]), `seq`, `sent_s` | — |
| `estop` | Orin → Pi | `engage`, `seq`, `sent_s` | — |
| `orin_status` | Orin → Pi | `stamp_s`, `vision_alive`, `vision_latency_s?`, `mission_state`, `cpu_temp_c?`, `gpu_temp_c?` | — |

Every header also has `"v": 1` and `"type"` (equal to the topic). `?` fields may be `null`; NaN is sent as `null`.

## Clocks
- Scan stamps, frame capture times and Pi status use the **Pi's monotonic clock**, so the Orin can match frames to scans and poses for item localization.
- `sent_s` on drive/estop is the Orin's clock, for latency diagnostics only. The Pi stamps each command with its own clock **on receipt** before passing it to `SafetySupervisor.evaluate`.

## Errors
`decode` raises `ProtocolError` for a wrong frame count, version, type/topic mismatch, missing or wrongly typed fields, bad payload size, or drive values outside [-1, 1]. Drop the message; for drive commands the safety supervisor's command timeout then stops the car. Unknown extra header fields are ignored (forward compatibility).

## Planned sockets
- Pi PUB `tcp://*:5555`: `scan`, `frame`, `pi_status`. Orin SUB; frame subscriber uses `CONFLATE` (newest only).
- Orin PUB `tcp://*:5556`: `drive`, `estop`, `orin_status`. Pi SUB.
- Fixed IPs on the direct Ethernet cable; never put real addresses in the repo.
