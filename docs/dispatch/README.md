# Dispatch: Telegram (via Photon) → backend → Jetson mission requests

**Status:** design and test plan only, on branch `feat/photon-dispatch`. No code yet. Not tested against Photon or a Jetson.

A customer messages a Telegram bot such as `/find blue toolbox`. Photon delivers the message to our FastAPI backend. The backend creates a mission and sends it to the Jetson over a WebSocket that the **Jetson opens outbound**. The Jetson sends back acknowledgements and status updates, and the backend turns those into Telegram replies through Photon.

```text
Telegram ──► Photon ──webhook (Standard Webhooks signed)──► FastAPI backend ◄──WS /ws/robot (Jetson dials out, token auth)── Jetson agent ──► MissionSink (perception/mission stack, later)
Telegram ◄── Photon ◄──POST /v1/projects/{id}/messages──── FastAPI backend ◄──MISSION_ACK / MISSION_STATUS──────────────── Jetson agent
```

Not in scope: detection, tracking, mapping, navigation, product lookup. The Jetson side stops at a `MissionSink` interface that the perception and mission lanes plug into later.

## Where it fits in the repo

The backend runs **off the robot**, on a laptop or a cloud VM with a public HTTPS URL, because Photon cannot deliver to localhost or private networks. The Jetson only runs the small agent. So the feature is split into a shared protocol, a Jetson-side agent, and a backend. The backend's web dependencies never need to be installed on the robot.

| Path | Runs on | Dependencies | What it is |
|---|---|---|---|
| `locatebot/dispatch/protocol.py` | both | stdlib | Wire messages (`MISSION_REQUEST`, `MISSION_ACK`, `MISSION_STATUS`, `MISSION_CANCEL`, `HELLO`), the `MissionState` enum, allowed transitions, `validate_*` functions, and target-description rules. The single source of truth for both sides. |
| `locatebot/dispatch/agent.py` | Jetson | stdlib | `JetsonAgent`: a pure state machine with an injected clock and transport. It validates, deduplicates, rejects expired missions, calls the `MissionSink`, and relays statuses. It does not do network I/O itself. |
| `locatebot/dispatch/sink.py` | Jetson | stdlib | `MissionSink` protocol (`start`, `cancel`, `on_link_lost`) plus `ScriptedSink`, a fake robot for tests and desk demos. |
| `locatebot/dispatch/ws_client.py` | Jetson | `websockets` (optional extra) | Thin adapter: dial, authenticate, reconnect with backoff, and feed frames to `JetsonAgent`. |
| `locatebot/backend/app.py` | backend host | `fastapi`, `uvicorn`, `httpx` (optional extra) | FastAPI routes: `POST /api/photon/webhook`, `POST /api/missions`, `GET /api/missions/{id}`, `POST /api/missions/{id}/cancel`, `WS /ws/robot`. |
| `locatebot/backend/messaging/base.py` | backend | stdlib | `MessagingProvider` interface: `parse_inbound(...) -> InboundMessage` and `send(conversation, text)`. This is what lets another provider replace Photon later. |
| `locatebot/backend/messaging/photon.py` | backend | stdlib + `httpx` | Photon adapter: Standard Webhooks verification, parsing `message.received` events, and `POST /v1/projects/{id}/messages` with an `Idempotency-Key`. |
| `locatebot/backend/commands.py` | backend | stdlib | Message parsing: `/start`, `/find`, `/status`, `/cancel`, `/help`, and description validation. |
| `locatebot/backend/missions.py` | backend | stdlib | Mission manager: creation, dedupe, one active mission per robot, expiry, state store. |
| `locatebot/backend/robot_link.py` | backend | stdlib | Jetson link: one authenticated connection, send and track, disconnect detection. Transport injected. |
| `locatebot/backend/notifier.py` | backend | stdlib | Status updates: mission state → customer text. Nothing says "found" unless the Jetson reported `TARGET_DETECTED`. |
| `tools/fake_jetson.py` | laptop or Jetson | `websockets` | Runs the real agent with `ScriptedSink`, so the backend can be tested with no robot. |
| `tools/send_fake_webhook.py` | laptop | stdlib | Posts correctly signed `message.received` events to a local backend. |
| `tests/dispatch/`, `tests/backend/` | CI/laptop | stdlib (+ FastAPI for HTTP tests) | See [TEST-PLAN.md](TEST-PLAN.md). |
| `.env.example` | — | — | Placeholders only. |

Why `dispatch` and not `mission`: `docs/lanes/MISSION.md` already reserves "mission" for the onboard planner that turns targets into bounded motion intent. This feature only *delivers* a request. The onboard planner later consumes `target_description` through `MissionSink.start`.

How it connects to the other subsystems later:
- **Perception**, `locatebot/vision/` (planned): `MissionSink.start` hands `target_description` to the LocateAnything prompt builder. That builder should use NVIDIA's exact template and the short-noun rules in `docs/vision/README.md`.
- **Mission planner / safety:** the sink enables the planner, and the planner emits bounded intent. The safety supervisor still has the final say on motion. The backend and Photon never send motion: the protocol has no message type for it.

## Photon facts this design relies on (Beta 2.0 docs, read 2026-11-07)
Source: `https://photon.codes/docs/beta/message-delivery/webhook.md` and the live schema at `https://api.photon.codes/openapi.json`.

- **Auth:** `Authorization: Bearer <key>`. The project API key `pho_sk_...` works only under `/v1/projects/{projectId}`, and project IDs look like `pho_prj_...`.
- **Telegram:** connected through `POST /v1/projects/{id}/platforms/telegram/bots`, which returns an operation with `detail.setupUrl`. This is a one-time dashboard/API setup, not something our code calls.
- **Inbound:** a webhook destination posts the event JSON unwrapped. For Telegram text, the event has `type: "message.received"`, `source: "telegram.photon"`, `id: pho_evt_...`, and `data.object` (a `ReceivedTelegramMessage`) with:
  - `space.id`: the conversation (`pho_spc_...`)
  - `sentBy.id` and `sentBy.handle`: the customer
  - `receivedBy` / `addressedTo`: our bot user
  - `content`: `{type: "text", format, text}`
- **Signature:** Standard Webhooks. Each delivery carries three headers: `webhook-id` (the event `id`, which stays the same across retries), `webhook-timestamp` (Unix seconds), and `webhook-signature`, which holds one or more space-separated `v1,<base64>` values. The signature is HMAC-SHA256 over `id.timestamp.rawBody`, keyed with the base64-decoded part of the secret after `whsec_`. Reject timestamps more than 5 minutes off. During secret rotation, Photon sends two signatures and either may match. Verification is implementable with `hmac` + `hashlib` from the standard library.
- **Delivery semantics:**
  - Any `2xx` is a success.
  - `404`, `408`, `429`, `5xx` and timeouts are retried, for up to 4 days or 20 attempts.
  - Any other `4xx` drops the delivery.
  - `410` disables the destination. Never return 410.
  - The timeout is 20 s, so acknowledge fast and do the work asynchronously.
  - Return `400` when the signature is bad and `503` when we can't persist the event. Return `2xx` for duplicates.
  - Ordering is guaranteed only per event type per destination.
- **Outbound:** `POST /v1/projects/{id}/messages` requires an `idempotency-key` header and the body `{senderId: pho_usr_..., spaceId: pho_spc_..., content: {type: "text", format, text}}`. It answers `202` with a status (`dispatched|accepted|delivered|failed`). Errors are RFC 9457 `application/problem+json`.

**To confirm against the live project before relying on it:**
1. Which user ID goes in `senderId` when replying (expected: the bot user from `receivedBy`).
2. The accepted `format` values for text content.
3. A real `message.received` body, captured once to use as a test fixture.

## Environment (`.env.example`)
```env
PHOTON_API_KEY=pho_sk_...          # project API key
PHOTON_PROJECT_ID=pho_prj_...
PHOTON_WEBHOOK_SECRET=whsec_...    # from the webhook destination; not the API key
JETSON_DEVICE_ID=
JETSON_AUTH_TOKEN=                 # long random string, compared in constant time
BACKEND_PUBLIC_URL=https://...     # public HTTPS URL Photon posts to
```

## Decisions to review
1. **Dependencies.** `AGENTS.md` says "standard library only", but FastAPI and a WebSocket client are required by the spec. Proposal:
   - Keep all logic in stdlib modules, tested with `unittest`.
   - Add optional extras `backend = [fastapi, uvicorn, httpx]` and `robot-link = [websockets]` to `pyproject.toml`. That is root config, so the integrator approves it.
   - HTTP and WebSocket tests skip cleanly when the extras are missing.
2. **Python version.** The local `python3` is 3.9.6, but the repo requires >=3.10. Use the repo `.venv` built with 3.10+.
3. **Link-loss behaviour.** When the WebSocket drops mid-mission, the Jetson calls `MissionSink.on_link_lost()`. The proposed default is "stop the mission and come to a safe stop". The onboard safety layer decides what the car actually does; the backend never assumes the robot stopped.
4. **One active mission per robot.** A new `/find` while one is active gets "already searching for X — /cancel first". There is no queue.
5. **State store.** Use SQLite (stdlib `sqlite3`) for the webhook-ID store and missions. The event-ID record and the mission row are written in one transaction, as Photon's docs require, so a restart neither loses nor duplicates work.
