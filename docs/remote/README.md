# Remote control over Photon/Telegram (`locatebot/remote/`, runs on the Pi)

**Status:** R1 (messaging core), R2 (Photon WebSocket inbound + `tools/remote_run.py`) and R3 (mission bridge Pi ⇄ Orin over ZeroMQ) built and tested in software only: SQLite in a temp dir, a scripted `FakeRobotLink` instead of the Orin, a recording provider instead of Photon, a fake Photon Socket.IO server and stdlib HTTP on 127.0.0.1. **Nothing here has talked to a real Photon project, Telegram, the Pi or the Orin.** No FastAPI, uvicorn or httpx; `websockets` (the `remote` extra) is needed only for WebSocket inbound and imported lazily.

**Credit:** the messaging half is chanabyte's work on `feat/photon-dispatch` (commits `97dd665`, `5941277`, `137536a`), moved here and adapted per the review in `docs/ROADMAP.md` section R.

A customer sends `/find blue toolbox` to the Telegram bot. Photon delivers the event to the Pi; `MissionService` creates a mission and sends it to the robot through a `RobotLink`; the robot acknowledges and reports progress; each report becomes a queued Telegram reply that `OutboxSender` sends through Photon.

```text
Telegram ─► Photon ─(Socket.IO over WebSocket, Pi dials out; or webhook)─► MissionService ─► RobotLink ─► robot (FakeRobotLink; Orin via ZeroMQ in R3)
Telegram ◄─ Photon ◄─POST /v1/projects/{id}/messages─ OutboxSender ◄─ SQLite outbox ◄─ MissionService ◄─ MISSION_ACK / MISSION_STATUS
```

Nothing here can move the car. The only frames sent to the robot are `MISSION_REQUEST` (find an object by text) and `MISSION_CANCEL`; there is no field for motion. The Pi's safety supervisor stays the final gate on motion.

## Modules
| Module | From the branch | What it does |
|---|---|---|
| `protocol.py` | `dispatch/protocol.py` | Frames `MISSION_REQUEST`, `MISSION_CANCEL`, `MISSION_ACK`, `MISSION_STATUS`, `ERROR`; `MissionState` (8 states) and `TRANSITIONS`; `normalize_description` (letters, digits, `- ' . , & / # ( ) +`, ≤ 120 chars); strict validation (unknown fields rejected, 4 KB cap) |
| `messaging.py` | `backend/messaging/base.py` | Provider-neutral `Conversation`, `InboundMessage`, `SendResult`, `MessagingProvider`, `ConsoleProvider` |
| `photon_ws.py` | new (R2) | `PhotonWebSocketInbound`: Photon's WebSocket delivery (Socket.IO subset over `websockets`), cursor + dedupe, heartbeats, backoff |
| `photon.py` | `backend/messaging/photon.py` | Standard Webhooks verification (`WebhookVerifier`, `sign_payload`), `parse_event` for `message.received` from `telegram.photon`, `PhotonWebhookHandler` (204/400/503 policy), `PhotonClient.send_text` (urllib) |
| `commands.py` | `backend/commands.py` | `/start`, `/find <text>`, `/status`, `/cancel`, `/help` (also `/find@Bot`) |
| `notifier.py` | `backend/notifier.py` | Customer texts; `OutboxSender` (retries, exponential backoff, `Retry-After`) |
| `store.py` | `backend/store.py` | SQLite: event ids, missions and the reply outbox, written in one transaction; plus the Photon WebSocket cursor |
| `missions.py` | `backend/missions.py` | `MissionService`: missions, ownership, dedupe, rate limit, timeouts, state changes |
| `config.py` | `backend/config.py` | `Settings.from_env`; secrets never in errors or `repr` |
| `robot.py` | new (from `dispatch/sink.py` + `dispatch/agent.py` rules) | `RobotLink` protocol + `FakeRobotLink` (scripted robot for tests/demos) |
| `robot_zmq.py` | new (R3) | `ZmqRobotLink`: missions to the Orin over the ZeroMQ link, statuses back |
| `service.py` | replaces `backend/app.py` | `RemoteService`: store + provider + `MissionService` + `OutboxSender` + inbound (WebSocket or webhook) + link + one background thread |
| `server.py` | replaces `backend/app.py` | stdlib `ThreadingHTTPServer`: the routes below |

Dropped: the FastAPI app, `backend/robot_link.py` (WebSocket device authentication; the Pi talks to the Orin over our private Ethernet link), `dispatch/ws_client.py`, `tools/fake_jetson.py` (use `FakeRobotLink`), the `backend`/`robot-link` extras. `dispatch/agent.py` (`JetsonAgent`) is not copied: its validation rules live on in `FakeRobotLink` and move to the Orin in R3 (`locatebot/orin/missions.py`).

## The robot seam
`MissionService(store, link, robot_id, clock, ...)` talks to the robot only through the link:
- `link.send(msg: dict) -> bool`: queue a frame, never block; `False` when offline.
- `link.connected`: whether the robot is reachable now.
- `link.listener`: set to the `MissionService`; the link calls `on_robot_frame(text)` with JSON frames, `on_robot_connected()` and `on_robot_disconnected()`.
- optional `start()` / `stop()`: `RemoteService` calls them.

`FakeRobotLink(script="happy"|"not_found"|"stall"|"reject"|[Step...], clock=time.time, online=True, cancel_delay_s=0.5, silent=False)` plays the script on `poll()` (or its own thread after `start()`): ACK on request; rejects expired, future-dated (> 30 s), duplicate (same answer again) and concurrent missions; confirms a cancel after `cancel_delay_s`; `drop()` stops the mission at once (`CANCELLED`, `LINK_LOST`) and `connect()` re-reports that real state.

## Mission flow over the link (R3)
```text
Telegram /find marker
  │ Photon (WebSocket or webhook)
  ▼
Pi: MissionService ──MISSION_REQUEST dict──► ZmqRobotLink ──MissionRequest(id, target, issued_s, expires_s)──► PUB :5555
                                                                                                                │
Orin: pi_source SUB ──► MissionBridge.on_request ──(control loop) tick──► set_goal("find marker")              ◄┘
                          │                                               routine: submit_operator(search([marker])) → go_to(item=marker)
                          │                                               LLM: the brain acts on the goal (approve/auto)
                          └── watches executor + item tracker ──► MissionStatus(id, state, message, reason) ──► PUB :5556
Pi: ZmqRobotLink SUB ──► MISSION_ACK / MISSION_STATUS frame ──► MissionService.on_robot_frame ──► outbox ──► Telegram reply
```
`ZmqRobotLink.connected` = an `orin_status` heard within 3 s (the Orin sends one per second). The first status of a mission becomes `MISSION_ACK` (`ACCEPTED`, or `REJECTED` if it is `FAILED` with `BUSY`/`EXPIRED`/`CLOCK_SKEW`/`INVALID_EXPIRY`); later ones are `MISSION_STATUS`. Mission times on the link are Unix wall clock (`issued_s`, `expires_s`): run NTP/chrony on both boards.

| Orin observes | Status sent | Customer reads |
|---|---|---|
| mission started (goal set, routine search issued) | `ACCEPTED` | (nothing; already told "Request received") |
| `search` / `explore` / `look_around` running | `SEARCHING` | "The robot has started searching for: marker." |
| a track named like the target becomes **confirmed** | `TARGET_DETECTED` (message: map position) | "Target detected. Preparing to approach it." |
| `go_to(item=target)` running | `APPROACHING` | "The robot is approaching the target." |
| that `go_to` succeeded | `COMPLETED` | "Search mission completed." |
| a search for the target ended without it | `FAILED` `NOT_FOUND` (message: the executor's found/missing text) | "I couldn't spot marker. The search has ended. (… not found: marker; found: …)" |
| `go_to` found no path | `FAILED` `NO_PATH` | "The search for marker ended without success." |
| mission TTL reached on the Orin | `FAILED` `TIMEOUT` (the robot is stopped) | same |
| executor internal error | `FAILED` `ROBOT_FAULT` (the robot is stopped) | same |
| someone else stopped the robot (operator page STOP, LLM `stop`, fallback) | `FAILED` `STOPPED` | same |
| `/cancel` → operator STOP → executor stopped | `CANCELLED` | "Cancelling…", then "Mission cancelled." |
| `go_to` lost the target, then a search runs again | `SEARCHING` (from `APPROACHING`/`TARGET_DETECTED`) | "Lost sight of the target. Searching again for: marker." |
| request expired / > 5 s in the future / `expires <= issued` / another mission active | `FAILED` `EXPIRED` / `CLOCK_SKEW` / `INVALID_EXPIRY` / `BUSY` → `MISSION_ACK REJECTED` | "The robot couldn't start this search (…)." |

Each state is sent once, in the allowed transition order (`protocol.TRANSITIONS`; the Orin has an identical copy, tested equal). A missing intermediate state is filled in only when it is true (e.g. `go_to` to a target that is already on the map from an earlier search gives `SEARCHING`, `TARGET_DETECTED`, `APPROACHING`). A duplicate request id gets the last status again and is never started twice.

**Cancel:** `/cancel` → `MISSION_CANCEL` → Orin operator `STOP` (with an LLM brain this also pauses it) and the goal is cleared → `CANCELLED` once the executor shows that STOP finished (or after 3 s with nothing running). **Link loss:** the Orin stops the robot when it hears no Pi scan for 2 s and keeps the mission; when scans return it re-reports the current state and resumes (routine: re-issues the current step; LLM: resumes the brain). The Pi's own safety stop already halts the car when Orin commands stop. On the Pi, `MissionService` tells the customer "Lost contact with the robot" when `orin_status` stops for 3 s and fails the mission with `LINK_LOST` after 15 s unless the Orin comes back.

**Wording matters:** the target text is the item name vision searches for, matched case-insensitively and exactly against item tracks. Short common nouns work best ("marker", not "dry erase marker pen"; see `docs/vision/README.md`).

Run: `tools/remote_run.py --robot zmq --orin-host <orin-address>` (or `$LOCATEBOT_ORIN_HOST`; `--pi-bind` picks the interface for the Pi PUB on 5555) on the Pi, and `tools/orin_run.py --pi-host <pi-address> ...` on the Orin (subscribes `mission_request`/`mission_cancel`, publishes `mission_status`). Standalone, `ZmqRobotLink` binds 5555 itself: in the full car the Pi process that already owns 5555 (T1.3) must pass its `Publisher` and call `link.flush_outgoing(pub)` from its own thread.

## Configuration (environment, or `.env` which is git-ignored; see `.env.example`)
| Variable | Required | Meaning |
|---|---|---|
| `PHOTON_API_KEY` | in `live` mode and for WebSocket inbound | Project API key `pho_sk_...` (`project:read` to receive, plus send rights for replies) |
| `PHOTON_PROJECT_ID` | yes | `pho_prj_...` (events for other projects are ignored) |
| `PHOTON_INBOUND` | no | `websocket` (default) or `webhook` |
| `PHOTON_WEBHOOK_SECRET` | in webhook mode | `whsec_...`; enables `POST /api/photon/webhook` |
| `PHOTON_API_VERSION` | no | pin the event payload version (date, e.g. `2026-07-01`); default: Photon's |
| `PHOTON_API_ORIGIN` | no | default `https://api.photon.codes` |
| `OPERATOR_API_TOKEN` | no | ≥ 32 chars; enables `/api/missions*` |
| `PHOTON_MODE` | no | `live` (default) or `console` (print replies) |
| `ROBOT_ID` | no | default `locatebot` |
| `DISPATCH_DB_PATH` | no | default `dispatch.db` (git-ignored) |
| `MISSION_TTL_S`, `ACK_TIMEOUT_S`, `LINK_GRACE_S` | no | 600, 10, 15 |

## Routes (`server.py`, default bind 127.0.0.1:8070)
| Route | Auth | Result |
|---|---|---|
| `GET /healthz` | none | `{"ok", "robot_connected", "inbound", "photon_connected" (null in webhook mode), "photon_last_error", "replies_pending"}` |
| `POST /api/photon/webhook` | Standard Webhooks signature | 204 OK, duplicate or ignored; 400 bad signature; 503 storage down (Photon retries); 413 > 256 KB; 404 unless `PHOTON_INBOUND=webhook` |
| `POST /api/missions` `{"target_description"}` + optional `Idempotency-Key` | `Bearer OPERATOR_API_TOKEN` | 201 created; 200 replay; 409 busy; 422 invalid; 503 robot offline |
| `GET /api/missions/{id}` | operator | 200 / 404 / 422 |
| `POST /api/missions/{id}/cancel` | operator | 202 cancel *requested* (the robot confirms `CANCELLED`); 404 / 409 |

Operator routes answer 401 (`WWW-Authenticate: Bearer`) for a wrong token (constant-time compare) and 403 when no token is configured.

## Behaviour rules
- **One active mission per robot.** A second `/find` gets "already searching", "busy" (another customer), or "unavailable" if the robot is offline; it is never queued.
- **The robot reports, the Pi doesn't guess.** Customer-facing state changes only when the robot reports them, along `protocol.TRANSITIONS`. Only `TARGET_DETECTED` produces a "detected" message. A status after a lost ACK implies acceptance.
- **The Pi can only fail a mission**, on a timeout: `NO_ACK` (10 s without acknowledgement), `EXPIRED` (TTL), `LINK_LOST` (15 s after a disconnect unless the robot reconnects), `ROBOT_OFFLINE` (request could not be sent).
- **Cancel is a request.** `/cancel` sends `MISSION_CANCEL`; the mission is `CANCELLED` only when the robot confirms.
- **Replies are queued and deduplicated.** Event id + mission + replies are written in one SQLite transaction; replies carry deterministic `idempotency-key`s, so a Photon retry never double-posts and a Photon outage never blocks the robot link.
- **Customers only touch their own missions.** `/status` and `/cancel` match on the Photon space plus sender.
- **Rate limit:** 5 `/find` per conversation per minute.

## Photon WebSocket inbound (`photon_ws.py`, R2)
Photon's docs (Beta, read 2026-11-08: [WebSocket](https://docs.photon.codes/docs/beta/message-delivery/websocket.md), [session](https://docs.photon.codes/docs/beta/websocket/session.md), [events](https://docs.photon.codes/docs/beta/websocket/events.md), [cursors](https://docs.photon.codes/docs/beta/websocket/cursors.md), [errors](https://docs.photon.codes/docs/beta/websocket/errors.md), [AsyncAPI](https://api.photon.codes/asyncapi.yaml)) say: *"Photon uses **Socket.IO**, not a raw WebSocket protocol."* We have no Socket.IO client dependency, so `photon_ws.py` implements the subset this endpoint uses over `websockets`:

| Step | On the wire |
|---|---|
| Connect | `wss://api.photon.codes/v1/projects/{projectId}/events/ws?EIO=4&transport=websocket`, header `Authorization: Bearer pho_sk_...`; Engine.IO open `0{sid, pingInterval, pingTimeout}` |
| Namespace | client `40`, server `40{sid}` (or `44{message}` = `connect_error`, treated as an auth failure) |
| Subscribe | client `420["subscribe",{"startSeq": <saved cursor or 0>[, "apiVersion"]}]`, server ack `430[{ok, apiVersion, connectionId, cursor, limits}]`; `ok:false` → reconnect with backoff; no ack in 10 s → reconnect |
| Delivery | server `42["delivery", <event>, {"seq", "replyExpected"}]`; the event is the webhook's event object |
| Errors | server `42["delivery_error", {code, message, fatal, retryable, retryAfterMs?}]`; fatal ones are followed by a disconnect |
| Heartbeat | server `2` → client `3`; silence for `pingInterval + pingTimeout` = dead connection |

Processing (one delivery at a time, in order): skip `seq <= cursor` (replay); skip an event id already in the `events` table (same table and key as the webhook path: Photon's `webhook-id` *is* the event `id`); `parse_event` exactly as for webhooks; `MissionService.handle_inbound` records the event id, mission and replies in one transaction; **then** the cursor is saved in SQLite. There is no per-delivery acknowledgement in the protocol; the saved cursor is the checkpoint. A handler failure (e.g. storage down) closes the connection and resumes from the last saved cursor. Bad frames are counted (`dropped_frames`) and logged; the connection stays up.

Reconnect: 1 s doubling to a 30 s cap, ×0.8–1.2 jitter, reset after a successful subscribe; `retryAfterMs` from a fatal `delivery_error` raises the wait. An auth failure (HTTP 401/403 on the upgrade, or `connect_error`) waits 5 min, never a tight loop, and `status()["last_error"]` says `authentication failed`. `stop()` cancels the session and joins the thread within 1 s. `status()`: `connected` (subscribed), `last_event_at`, `reconnects`, `dropped_frames`, `events`, `duplicates`, `last_error`, `connection_id`, `api_version`, `cursor`.

The API key is never logged: our messages don't include it, exceptions are scrubbed, `repr` shows only the project, and `websockets`' own DEBUG handshake log (which prints `Authorization: Bearer ...`) goes through `locatebot.remote.photon_ws.wire`, a logger with a redacting filter.

Not used: `replyExpected` / the `reply` command (synchronous HTTP replies to an event); chat replies go through the messaging API.

### Offline behaviour
- The robot link does not depend on Photon or the internet: missions in progress continue and are reported as usual.
- Inbound reconnects quietly with backoff; `/healthz` shows `photon_connected: false` and the last error.
- Replies stay in the SQLite outbox (`replies_pending` in `/healthz`) and `OutboxSender` retries them (2 s doubling to 300 s, 12 attempts ≈ 30 min) before giving up on a reply.
- If the robot itself is unreachable, a new `/find` is answered "The robot is currently unavailable" (as soon as Photon is reachable again).

## Run on the Pi (`tools/remote_run.py`)
```bash
cp .env.example .env && chmod 600 .env       # fill in PHOTON_API_KEY, PHOTON_PROJECT_ID
.venv/bin/python tools/remote_run.py          # WebSocket inbound, live replies, operator API on 127.0.0.1:8070
.venv/bin/python tools/remote_run.py --console --debug     # print replies instead of sending; RSS every 60 s
# fully offline desk demo (no Photon at all): webhook inbound + the fake webhook sender
export PHOTON_INBOUND=webhook PHOTON_MODE=console PHOTON_PROJECT_ID=pho_prj_01jabcdefghjkmnpqrstvwxyz1 \
       PHOTON_WEBHOOK_SECRET=$(python3 -c "import base64,os;print('whsec_'+base64.b64encode(os.urandom(32)).decode())")
.venv/bin/python tools/remote_run.py --fake-script happy &
.venv/bin/python tools/send_fake_webhook.py --text "/find blue toolbox"
curl -s http://127.0.0.1:8070/healthz
```
Flags: `--env-file` (default repo `.env`; never overrides variables already set), `--host`/`--port` (default 127.0.0.1:8070), `--robot fake|zmq` (`--orin-host`, `--pi-bind`), `--fake-script happy|not_found|stall|reject`, `--console`, `--debug`. Ctrl+C / SIGTERM shut down cleanly (inbound closed, HTTP server stopped, threads joined).

systemd: `deploy/locatebot-remote.service` (`Restart=always`, `MemoryMax=150M`, reads `.env` via `--env-file`; adjust `User`/paths).

## Measured
- R1: RSS of a running `RemoteService` (console provider, `FakeRobotLink`, background thread) + `server.py` after 20 webhook deliveries: **≈ 30 MB** on the x86 laptop (Python 3.14; 23 MB for the bare interpreter + imports).
- R2: WebSocket mode (`websockets` 17.4 loaded, subscribed to a fake Photon server **in the same process**, 40 deliveries processed, HTTP server up): **≈ 38 MB**, an upper bound since it includes the fake server and pytest imports.
- Target on the Pi: < 80 MB (`MemoryMax=150M` in the unit). Not measured on the Pi yet.

## Tests
`tests/remote/` (all run without FastAPI/uvicorn/httpx/websockets, no network): the branch's unit tests for protocol, commands, config, photon, notifier, missions; `test_robot.py` (FakeRobotLink); `test_server.py` (real server on 127.0.0.1 port 0, urllib); `test_e2e.py` (scenarios E1–E12 from the branch, driven through `FakeRobotLink`); `test_photon_ws.py` (fake Photon Socket.IO server on 127.0.0.1: happy path, duplicate id / replayed cursor, malformed frames, drop + backoff + resume, backoff cap and jitter, HTTP 401 and `connect_error` without a tight loop, rejected/timed-out subscribe, heartbeat, fatal `delivery_error` with `retryAfterMs`, handler failure resumes from the saved cursor, `stop()` < 1 s, key never in logs/`repr`/status, `RemoteService` end to end); `test_tools.py` (`tools/send_fake_webhook.py`, `tools/remote_run.py` as a real process: webhook in, reply out, SIGTERM exit 0). R3: `test_robot_zmq.py` (127.0.0.1 ZeroMQ with a fake Orin: request/cancel encoding, status → ACK/STATUS frames, connected/disconnected from `orin_status` timing, malformed messages dropped, shared publisher); `tests/orin/test_missions.py` (bridge unit tests with a fake runtime); `tests/orin/test_mission_e2e.py` (`RemoteService` + `ZmqRobotLink` ⇄ ZeroMQ ⇄ `MissionBridge` + `OrinRuntime` on the simulated car, no LLM: `/find marker` → accepted, searching, detected, approaching, completed in ≈ 45 s sim / 7 s wall, car ends < 1.4 m from the marker; `/find unicorn` → `NOT_FOUND` with the found/missing text (40 s search timeout, ≈ 7 s wall); `/cancel` mid-search → cancelled and the executor stopped, < 1 s wall).

## What comes next
- **R5 (live):** real Photon project + Telegram bot. Confirm: the WebSocket handshake and our Socket.IO subset against the real server (path, `Authorization` header, `40` → subscribe ack, ping/pong timing); the subscribe ack fields and `limits`; that `delivery` events are `message.received` / `telegram.photon` with the shape `parse_event` expects; cursor resume after a Pi restart and after pulling the uplink; `delivery_error` codes seen in practice; that `senderId` = bot user from `receivedBy` and `format: "plain"` are accepted for replies; that the key's scopes cover both receiving (`project:read`) and sending. Capture one real event (PII removed) to replace the synthetic `tests/fixtures/photon_message_received.json`. Measure RSS on the Pi. Mission bridge on the real cable: Pi ⇄ Orin clocks in sync (NTP/chrony; `CLOCK_SKEW` otherwise), `mission_status` delivery and order, link-loss stop/resume by pulling the cable, a real `/find` with the camera and LocateAnything (item wording), and ZmqRobotLink inside the Pi process that owns port 5555.
