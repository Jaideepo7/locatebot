# Dispatch: Telegram (via Photon) → FastAPI backend → Jetson mission requests

**Status (branch `feat/photon-dispatch`):**
- **Built:** the protocol, the Jetson agent, the backend services, the FastAPI app, the Jetson WebSocket client and two test tools.
- **Tested with no robot and no network:** 220 tests pass with Python 3.13, including the end-to-end scenarios E1–E12.
- **Written but not yet run:** the FastAPI route tests (`tests/backend/test_app.py`) and the live-socket test (`tests/backend/test_live_socket.py`). They skip until the `backend` and `robot-link` extras are installed.
- **Not yet tried:** a real Photon project, Telegram, or the Jetson.

A customer sends `/find blue toolbox` to the Telegram bot, and Photon posts a signed webhook to the backend. The backend creates a mission and sends it to the Jetson over a WebSocket that the **Jetson opens outbound**. The Jetson acknowledges and reports progress, and the backend turns each report into a Telegram reply through Photon.

```text
Telegram ─► Photon ─signed webhook─► POST /api/photon/webhook ─► MissionService ─► RobotLink ◄─WS /ws/robot (Jetson dials out)─ JetsonAgent ─► MissionSink
Telegram ◄─ Photon ◄─POST /v1/projects/{id}/messages─ OutboxSender ◄─ replies ◄─ MissionService ◄─ MISSION_ACK / MISSION_STATUS ◄─ JetsonAgent
```

Out of scope: detection, tracking, mapping, navigation, product lookup. The Jetson side ends at `MissionSink`, which the perception and mission lanes implement later.

## Quick start (laptop only, no robot, no Photon)
```bash
python3.13 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev,backend,robot-link]'
python -m pytest -q                      # every test, including the FastAPI and live-socket ones

cp .env.example .env                     # fill in; for local runs set PHOTON_MODE=console
set -a; . ./.env; set +a

# terminal 1: the backend (console mode prints the Telegram replies)
uvicorn --factory locatebot.backend.app:create_app --port 8000 --ws-ping-interval 10 --ws-ping-timeout 10
# terminal 2: the fake Jetson (real agent + scripted robot)
python tools/fake_jetson.py --url ws://127.0.0.1:8000/ws/robot --script happy
# terminal 3: the "customer"
python tools/send_fake_webhook.py --text "/find blue toolbox"
```
For local console mode you still need `BACKEND_PUBLIC_URL=https://...` (any https value) and a `whsec_` secret. Generate one with:
```bash
python -c "import base64,os;print('whsec_'+base64.b64encode(os.urandom(32)).decode())"
```
More drills are in [TEST-PLAN.md](TEST-PLAN.md): `--script not_found|stall|reject`, `--drop-after 4`, `--silent`, `--repeat 3`, `--customer b`, `--bad-signature`.

## Where it fits in the repo
The backend runs **off the robot**, because Photon only delivers to public HTTPS and blocks localhost. The Jetson runs only the agent. All decision logic is standard-library Python; FastAPI and `websockets` are thin adapters at the edges.

| Path | Runs on | Needs | What it does |
|---|---|---|---|
| `locatebot/dispatch/protocol.py` | both | stdlib | Frames `MISSION_REQUEST`, `MISSION_CANCEL`, `MISSION_ACK`, `MISSION_STATUS`, `ERROR`; the 8 states and allowed transitions; target-description rules; strict validation (unknown fields rejected, 4 KB cap) |
| `locatebot/dispatch/agent.py` | Jetson | stdlib | `JetsonAgent`: rejects expired, future-dated, duplicate and concurrent missions; relays only what the sink reports; calls `on_link_lost` on disconnect; re-reports its state on reconnect |
| `locatebot/dispatch/sink.py` | Jetson | stdlib | `MissionSink` interface (`start`, `cancel`, `on_link_lost`) and `ScriptedSink`, the fake robot |
| `locatebot/dispatch/ws_client.py` | Jetson | `websockets` | Dials out with `Authorization: Bearer` + `X-Device-Id`; reconnects with backoff; keeps ticking while offline |
| `locatebot/backend/messaging/base.py` | backend | stdlib | Provider-neutral `InboundMessage`, `Conversation`, `MessagingProvider`, `ConsoleProvider` |
| `locatebot/backend/messaging/photon.py` | backend | stdlib | Standard Webhooks verification, `message.received` parsing, webhook HTTP status policy, send client (`urllib`) |
| `locatebot/backend/commands.py` | backend | stdlib | Parses `/start`, `/find`, `/status`, `/cancel`, `/help` |
| `locatebot/backend/missions.py` | backend | stdlib | `MissionService`: missions, ownership, deduplication, rate limit, timeouts, state changes |
| `locatebot/backend/notifier.py` | backend | stdlib | Customer texts and `OutboxSender` (retries, backoff, `Retry-After`) |
| `locatebot/backend/robot_link.py` | backend | stdlib | Jetson authentication (constant-time compare) and the single live connection (newest wins) |
| `locatebot/backend/store.py` | backend | stdlib | SQLite: webhook event IDs, missions and the reply outbox, written in one transaction |
| `locatebot/backend/config.py` | backend | stdlib | Environment validation; secrets never appear in errors or `repr` |
| `locatebot/backend/app.py` | backend | FastAPI | The five routes plus `/healthz`, and a background thread running timeouts and the outbox |
| `tools/fake_jetson.py`, `tools/send_fake_webhook.py` | laptop or Jetson | see above | Stand-ins for the robot and for Photon |

Why `dispatch` and not `mission`: `docs/lanes/MISSION.md` already uses "mission" for the onboard planner that produces bounded motion intent. This feature only *delivers* a request. The planner and LocateAnything get `target_description` through `MissionSink.start`.

### Plugging in the real robot later
Write a class with `start(mission_id, target_description, report)`, `cancel(mission_id)` and `on_link_lost(mission_id)`:
- **`start`** passes the text to the LocateAnything prompt, using NVIDIA's exact template and the short-noun advice in `docs/vision/README.md`. It enables the mission planner and returns quickly.
- **`report`** is called as `report(MissionState.SEARCHING, "message")` whenever the robot's real state changes. `report(MissionState.FAILED, "...", "NOT_FOUND")` ends the search.
- **`cancel` and `on_link_lost`** stop the search, then confirm with `report(MissionState.CANCELLED, ...)`. The safety supervisor, not this code, decides how the car actually stops.

Pass the sink to `JetsonAgent` inside `run_client` (see `tools/fake_jetson.py`).

## Endpoints
| Route | Auth | Result |
|---|---|---|
| `POST /api/photon/webhook` | Standard Webhooks signature | 204 OK, duplicate or ignored; 400 bad signature; 503 storage down (Photon retries) |
| `POST /api/missions` `{"target_description"}` + optional `Idempotency-Key` | `Bearer OPERATOR_API_TOKEN` | 201 created; 200 replay; 409 busy; 422 invalid; 503 robot offline |
| `GET /api/missions/{id}` | operator | 200 / 404 / 422 |
| `POST /api/missions/{id}/cancel` | operator | 202 cancel *requested* (the robot confirms `CANCELLED`); 404 / 409 |
| `WS /ws/robot` | headers `Authorization: Bearer JETSON_AUTH_TOKEN`, `X-Device-Id` | Credentials in the query string are refused; binary frames close the socket |
| `GET /healthz` | none | `{"ok": true, "robot_connected": bool}` |

## Behaviour rules
- **One active mission per robot.** A second `/find` gets "already searching", "busy", or "unavailable" if the robot is offline; it is never queued.
- **The robot reports, the backend doesn't guess.** Customer-facing state changes only when the Jetson reports them. Only `TARGET_DETECTED` produces a "detected" message.
- **The backend can only fail a mission.** The one exception to the rule above is a timeout, which can only *fail* a mission:
  - `NO_ACK` after 10 s with no acknowledgement;
  - `EXPIRED` when the TTL runs out;
  - `LINK_LOST` 15 s after a disconnect, unless the robot reconnects;
  - `ROBOT_OFFLINE` if the request could not be sent.
- **Link loss is handled on the robot.** The customer is told right away. The Jetson calls `on_link_lost`, and `ScriptedSink`'s policy is to stop immediately. On reconnect the agent reports the real state.
- **Replies are queued and deduplicated.** They go through the SQLite outbox with deterministic `idempotency-key`s, so a Photon retry never double-posts and a Photon outage never blocks the robot link.
- **Customers only touch their own missions.** `/status` and `/cancel` match on the Photon space plus sender.
- **Rate limit:** 5 `/find` per conversation per minute.

## Photon facts relied on (Beta 2.0, read 2026-11-07)
Sources: `https://photon.codes/docs/beta/message-delivery/webhook.md` and `https://api.photon.codes/openapi.json`.
- **Auth:** `Authorization: Bearer pho_sk_...` (project API key); project IDs are `pho_prj_...`.
- **Telegram setup:** done once through `POST /v1/projects/{id}/platforms/telegram/bots` or the dashboard.
- **Inbound:** the event JSON arrives unwrapped. Telegram text is `type: message.received`, `source: telegram.photon`, `data.object` with:
  - `space.id`: the conversation
  - `sentBy.id` / `handle`: the customer
  - `receivedBy` or `addressedTo`: our bot
  - `content {type: text, format, text}`
- **Signature:** Standard Webhooks.
  - Headers: `webhook-id` (stays the same across retries, so it is the dedupe key), `webhook-timestamp`, and `webhook-signature` (`v1,<b64>` values, space-separated during secret rotation).
  - HMAC-SHA256 over `id.ts.rawBody`, keyed with the base64-decoded part after `whsec_`.
  - ±5 minutes tolerance.
- **Retries:** 2xx = success. 404, 408, 429, 5xx and timeouts are retried for up to 4 days or 20 attempts. Other 4xx are dropped. 410 disables the destination. Timeout is 20 s.
- **Outbound:** `POST /v1/projects/{id}/messages` with an `idempotency-key` header and body `{senderId, spaceId, content: {type: "text", format: "plain", text}}` → 202 with a status.

**Confirm on the live project (TEST-PLAN L4):**
1. That `senderId` should be the bot user from `receivedBy`.
2. That `format: "plain"` is accepted for Telegram.
3. Capture one real delivery, with personal data removed, to replace the synthetic `tests/fixtures/photon_message_received.json`.

## Decisions and open items
1. **Dependencies.** `AGENTS.md` says "standard library only", but the spec requires FastAPI. So the logic is stdlib-only, and FastAPI, uvicorn, httpx and `websockets` are optional extras in `pyproject.toml` (`backend`, `robot-link`). Their tests skip when the extras are missing. Changing `pyproject.toml` is a root-config change: integrator, please review.
2. **Python.** The repo needs >=3.10; developed and tested with 3.13.5. The code avoids 3.11+-only APIs, but no 3.10 interpreter was available to check that.
3. **Link-loss policy.** `ScriptedSink` stops at once. The real sink's policy belongs to the safety lane.
4. **Not built:** a Photon WebSocket delivery mode (an alternative to the webhook that needs no public URL), multi-robot routing, and a queue of pending missions.
