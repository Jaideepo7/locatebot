# Remote control over Photon/Telegram (`locatebot/remote/`, runs on the Pi)

**Status:** R1 (messaging core) built and tested in software only: SQLite in a temp dir, a scripted `FakeRobotLink` instead of the Orin, a recording provider instead of Photon, stdlib HTTP on 127.0.0.1. **Nothing here has talked to a real Photon project, Telegram, or the Orin.** No FastAPI, uvicorn, httpx or websockets needed.

**Credit:** the messaging half is chanabyte's work on `feat/photon-dispatch` (commits `97dd665`, `5941277`, `137536a`), moved here and adapted per the review in `docs/ROADMAP.md` section R.

A customer sends `/find blue toolbox` to the Telegram bot. Photon delivers the event to the Pi; `MissionService` creates a mission and sends it to the robot through a `RobotLink`; the robot acknowledges and reports progress; each report becomes a queued Telegram reply that `OutboxSender` sends through Photon.

```text
Telegram ─► Photon ─(webhook today; WebSocket in R2)─► MissionService ─► RobotLink ─► robot (FakeRobotLink; Orin via ZeroMQ in R3)
Telegram ◄─ Photon ◄─POST /v1/projects/{id}/messages─ OutboxSender ◄─ SQLite outbox ◄─ MissionService ◄─ MISSION_ACK / MISSION_STATUS
```

Nothing here can move the car. The only frames sent to the robot are `MISSION_REQUEST` (find an object by text) and `MISSION_CANCEL`; there is no field for motion. The Pi's safety supervisor stays the final gate on motion.

## Modules
| Module | From the branch | What it does |
|---|---|---|
| `protocol.py` | `dispatch/protocol.py` | Frames `MISSION_REQUEST`, `MISSION_CANCEL`, `MISSION_ACK`, `MISSION_STATUS`, `ERROR`; `MissionState` (8 states) and `TRANSITIONS`; `normalize_description` (letters, digits, `- ' . , & / # ( ) +`, ≤ 120 chars); strict validation (unknown fields rejected, 4 KB cap) |
| `messaging.py` | `backend/messaging/base.py` | Provider-neutral `Conversation`, `InboundMessage`, `SendResult`, `MessagingProvider`, `ConsoleProvider` |
| `photon.py` | `backend/messaging/photon.py` | Standard Webhooks verification (`WebhookVerifier`, `sign_payload`), `parse_event` for `message.received` from `telegram.photon`, `PhotonWebhookHandler` (204/400/503 policy), `PhotonClient.send_text` (urllib) |
| `commands.py` | `backend/commands.py` | `/start`, `/find <text>`, `/status`, `/cancel`, `/help` (also `/find@Bot`) |
| `notifier.py` | `backend/notifier.py` | Customer texts; `OutboxSender` (retries, exponential backoff, `Retry-After`) |
| `store.py` | `backend/store.py` | SQLite: event ids, missions and the reply outbox, written in one transaction |
| `missions.py` | `backend/missions.py` | `MissionService`: missions, ownership, dedupe, rate limit, timeouts, state changes |
| `config.py` | `backend/config.py` | `Settings.from_env`; secrets never in errors or `repr` |
| `robot.py` | new (from `dispatch/sink.py` + `dispatch/agent.py` rules) | `RobotLink` protocol + `FakeRobotLink` (scripted robot for tests/demos) |
| `service.py` | replaces `backend/app.py` | `RemoteService`: store + provider + `MissionService` + `OutboxSender` + webhook handler + link + one background thread |
| `server.py` | replaces `backend/app.py` | stdlib `ThreadingHTTPServer`: the routes below |

Dropped: the FastAPI app, `backend/robot_link.py` (WebSocket device authentication; the Pi talks to the Orin over our private Ethernet link), `dispatch/ws_client.py`, `tools/fake_jetson.py` (use `FakeRobotLink`), the `backend`/`robot-link` extras. `dispatch/agent.py` (`JetsonAgent`) is not copied: its validation rules live on in `FakeRobotLink` and move to the Orin in R3 (`locatebot/orin/missions.py`).

## The robot seam
`MissionService(store, link, robot_id, clock, ...)` talks to the robot only through the link:
- `link.send(msg: dict) -> bool`: queue a frame, never block; `False` when offline.
- `link.connected`: whether the robot is reachable now.
- `link.listener`: set to the `MissionService`; the link calls `on_robot_frame(text)` with JSON frames, `on_robot_connected()` and `on_robot_disconnected()`.
- optional `start()` / `stop()`: `RemoteService` calls them.

`FakeRobotLink(script="happy"|"not_found"|"stall"|"reject"|[Step...], clock=time.time, online=True, cancel_delay_s=0.5, silent=False)` plays the script on `poll()` (or its own thread after `start()`): ACK on request; rejects expired, future-dated (> 30 s), duplicate (same answer again) and concurrent missions; confirms a cancel after `cancel_delay_s`; `drop()` stops the mission at once (`CANCELLED`, `LINK_LOST`) and `connect()` re-reports that real state.

## Configuration (environment, or `.env` which is git-ignored; see `.env.example`)
| Variable | Required | Meaning |
|---|---|---|
| `PHOTON_API_KEY` | in `live` mode | Project API key `pho_sk_...` |
| `PHOTON_PROJECT_ID` | yes | `pho_prj_...` (events for other projects are ignored) |
| `PHOTON_WEBHOOK_SECRET` | no | `whsec_...`; enables `POST /api/photon/webhook` |
| `OPERATOR_API_TOKEN` | no | ≥ 32 chars; enables `/api/missions*` |
| `PHOTON_MODE` | no | `live` (default) or `console` (print replies) |
| `ROBOT_ID` | no | default `locatebot` |
| `DISPATCH_DB_PATH` | no | default `dispatch.db` (git-ignored) |
| `MISSION_TTL_S`, `ACK_TIMEOUT_S`, `LINK_GRACE_S` | no | 600, 10, 15 |

## Routes (`server.py`, default bind 127.0.0.1:8070)
| Route | Auth | Result |
|---|---|---|
| `GET /healthz` | none | `{"ok": true, "robot_connected": bool}` |
| `POST /api/photon/webhook` | Standard Webhooks signature | 204 OK, duplicate or ignored; 400 bad signature; 503 storage down (Photon retries); 413 > 256 KB; 404 when no secret is set |
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

## Measured
- RSS of a running `RemoteService` (console provider, `FakeRobotLink`, background thread) + `server.py` after 20 webhook deliveries: **≈ 30 MB** on the x86 laptop (Python 3.14; 23 MB for the bare interpreter + imports). Target on the Pi: < 80 MB. Not measured on the Pi yet.

## Tests
`tests/remote/` (all run without FastAPI/uvicorn/httpx/websockets, no network): the branch's unit tests for protocol, commands, config, photon, notifier, missions; `test_robot.py` (FakeRobotLink); `test_server.py` (real server on 127.0.0.1 port 0, urllib); `test_e2e.py` (scenarios E1–E12 from the branch, driven through `FakeRobotLink`); `test_tools.py` (`tools/send_fake_webhook.py`).

## What comes next
- **R2:** Photon WebSocket inbound (no public URL needed), `tools/remote_run.py`, systemd unit.
- **R3:** `ZmqRobotLink` (Pi) + `MissionBridge` (Orin) over the existing ZeroMQ link.
- **R5:** live Photon project + Telegram bot; confirm `senderId` = bot user from `receivedBy` and that `format: "plain"` is accepted; replace the synthetic `tests/fixtures/photon_message_received.json` with a captured event (PII removed).
