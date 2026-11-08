# Dispatch test plan: testing without the robot

The RC car isn't running yet, so every layer is tested against a stand-in for the parts we don't have. Here is what replaces each real component, from cheapest to most realistic:

| Real thing | Stand-in | Used in levels |
|---|---|---|
| Photon webhook delivery | Locally signed `message.received` JSON (test secret, same Standard Webhooks algorithm) | 1–4 |
| Photon send API | `FakeMessagingProvider` that records every outbound message and can be told to fail (5xx, 429, timeout) | 1–4 |
| Jetson + robot stack | `JetsonAgent` + `ScriptedSink`, the real agent driven by a script of statuses instead of LocateAnything | 1–5 |
| Network | In-memory transport (L1–L2), then real localhost WebSocket (L3), then the internet (L4–L5) | — |
| Wall clock | Injected `clock()` so expiry and replay windows are tested exactly, with no sleeps | 1–3 |

## Where each level lives (status 2026-11-07)
| Level | Automated in | Runs now? |
|---|---|---|
| L1 | `tests/dispatch/test_protocol.py`, `test_agent.py`, `test_sink.py`; `tests/backend/test_commands.py`, `test_photon.py`, `test_missions.py`, `test_notifier.py`, `test_robot_link.py`, `test_config.py`, `test_tools.py` | Yes, stdlib + pytest |
| L1.5 | `tests/backend/test_e2e.py`: E1–E12 plus extra drills, with the real services wired together through an in-memory socket | Yes |
| L2 | `tests/backend/test_app.py`: FastAPI `TestClient`, the real routes and WebSocket | Skips until `pip install -e '.[backend]'` |
| L3 | `tests/backend/test_live_socket.py`: real uvicorn + real `websockets` client on localhost; plus the manual 3-terminal run below | Skips until `.[backend,robot-link]` |
| L4, L5 | Manual checklist below | Needs a Photon project, a tunnel and the Jetson |

Test IDs in the code are noted in comments (e.g. `# W4`) so each case below can be traced to its test.

## The test levels

**L1 — Unit tests.** No network and no dependencies (`python3 -m unittest discover -s tests -v`). These cover the pure logic: parsing, the signature check, protocol validation, the mission store, the agent state machine and message text. The run is fast and deterministic, and this level holds most of the cases below.

**L2 — In-process integration.** FastAPI `TestClient` with a WebSocket. Signed events go into `POST /api/photon/webhook` and a fake Jetson on `TestClient.websocket_connect("/ws/robot")`, with the fake provider's outbox checked afterwards. This exercises the whole backend in one process, with no ports open. It needs the `backend` extra, and its tests skip when FastAPI is missing.

**L3 — Local end-to-end on one laptop.** Three terminals:
```bash
uvicorn --factory locatebot.backend.app:create_app --port 8000 --ws-ping-interval 10 --ws-ping-timeout 10
python tools/fake_jetson.py --url ws://127.0.0.1:8000/ws/robot --script happy   # or not_found|stall|reject, --drop-after 4, --silent
python tools/send_fake_webhook.py --text "/find blue toolbox"   # signs with PHOTON_WEBHOOK_SECRET; --repeat 3, --customer b, --bad-signature
```
`PHOTON_MODE=console` prints the Telegram replies in terminal 1 instead of calling Photon.
This proves the real WebSocket code, reconnect logic and timing. Kill the fake Jetson mid-mission (Ctrl-C) to test disconnects.

**L4 — Real Photon + Telegram, fake robot.** The backend runs on a laptop behind a public HTTPS tunnel. Photon blocks localhost, so the tunnel is required (any HTTPS tunnel works). Set it as the webhook destination URL. Message the bot from a real Telegram account while `fake_jetson.py` plays the scripts. This is the only level that proves Photon's real payload, signatures and reply `senderId`. Capture one real delivery as a fixture for L1 (remove personal data first).

**L5 — Real Jetson on the desk, no motors.** Run the agent on the Orin with `ScriptedSink` (proves the network, auth and reconnect from the robot). Then run it with a sink that runs LocateAnything on still photos and reports `TARGET_DETECTED` from real boxes. The wheels stay off the ground or the ESC stays unplugged. **None of L1–L5 proves the car moves safely.** That belongs to the safety and hardware gates in `docs/05-VERIFICATION-AND-HARDWARE-GATES.md`.

### `ScriptedSink` scripts (used in L2–L5)
| Script | Jetson behaviour |
|---|---|
| `happy` | ACK ACCEPTED → SEARCHING → TARGET_DETECTED → APPROACHING → COMPLETED, 1 s apart |
| `not_found` | ACK → SEARCHING → FAILED ("Target not found") |
| `reject` | ACK REJECTED (reason `BUSY`) |
| `silent` | Never ACKs (tests the backend's ACK timeout) |
| `stall` | ACK → SEARCHING, then nothing (tests the status timeout and `/status`) |
| `drop_mid_mission` | ACK → SEARCHING → closes the socket |
| `garbage` | Sends malformed JSON, an unknown `type`, and a status for an unknown mission_id |
| `illegal_jump` | ACK → COMPLETED with no SEARCHING (backend must flag it, not show it to the customer) |
| `cancel_ok` | On cancel → CANCELLED |

## Test cases

Format: **ID** — given / when → expected. The level is in brackets. "Fake provider" means the recording `FakeMessagingProvider`.

### P — Command parsing (`commands.py`) [L1]
- **P1** `/find blue toolbox` → `Find("blue toolbox")`.
- **P2** `/find   Blue   Toolbox  ` → whitespace collapsed and trimmed. Case is kept for display; the comparison key is lowercase.
- **P3** `/find` and `/find    ` → `NeedsDescription`; the reply asks for a specific description.
- **P4** `/find@MyRobotBot red cup` (Telegram group form) → `Find("red cup")`.
- **P5** `/FIND red cup` → treated the same as `/find`.
- **P6** `/start`, `/help`, `/status`, `/cancel` → their commands, with trailing text ignored.
- **P7** Plain text with no slash, e.g. `blue toolbox` → help hint, no mission. (Decision to confirm: auto-treat as `/find`? The default is no.)
- **P8** Unknown `/dance` → "Unknown command" + help.
- **P9** A description over 120 characters → rejected with the reason, no mission.
- **P10** A description of only punctuation or emoji (`/find ???`, `/find 🔧`) → `NeedsDescription`.
- **P11** Shell or command-looking text: `/find $(rm -rf /)`, `/find ; reboot`, `/find ../../etc` → **never executed**. It is either rejected by the allowed-character rule or passed as inert text. Assert that no `subprocess`/`os.system` is reachable from the parser; a test greps the module.
- **P12** Motion-looking text: `/find forward 5m full speed` → becomes only a `target_description` string. The protocol has no motion field (also see S1).
- **P13** Control characters, NUL and newlines inside the description → stripped or rejected. `\n` must not reach the Jetson.
- **P14** Non-text content (photo, voice, sticker) → polite "send text" reply, no mission.

### W — Photon webhook verification and parsing (`messaging/photon.py`) [L1, then L2 through the route]
- **W1** Known-answer vector: secret `whsec_MfKQ9r8GKYqrTwjUPD8ILPZIo2LaLaSw`, id `msg_p5jXN8AQM9LWM0D4loKWxJek`, timestamp `1614265330`, body `{"test": 2432232314}` → `v1,g0hM9SsE+OTPJTGt/tmIKtSyZlE3uFJELVlNIOLJ1OE=` is valid. Checked on 2026-11-07 with stdlib `hmac`.
- **W2** Same body with one byte changed → invalid → HTTP **400**, nothing processed.
- **W3** Body re-serialised (`json.dumps(json.loads(body))`) → invalid. Proves we verify the raw bytes.
- **W4** Timestamp 301 s in the past or 301 s in the future → 400. 299 s → OK. Uses the injected clock.
- **W5** Missing any one of the three headers → 400.
- **W6** Rotation: the header has two signatures `v1,<old> v1,<new>` → accepted if either matches the configured secret.
- **W7** Wrong secret → 400. Wrong version prefix (`v2,...`) → ignored, so 400 if it is the only one.
- **W8** Constant-time compare: assert `hmac.compare_digest` is used, by reading the code.
- **W9** The secret without the `whsec_` prefix, or invalid base64 → refuse to start the app (config error), not a runtime 500.
- **W10** A valid Telegram `message.received` fixture → `InboundMessage(event_id, provider="photon", conversation_id=space.id, sender_id=sentBy.id, sender_handle, bot_user_id=receivedBy.id, text)`.
- **W11** A valid event of another type (`message.delivered`, `space.updated`) → 204, ignored.
- **W12** `source` is not `telegram.photon` (e.g. SMS) → 204, ignored (or handled the same way if we enable other platforms later).
- **W13** `data.projectId` ≠ `PHOTON_PROJECT_ID` → 204, ignored, logged. Proves we only act on our own project.
- **W14** A valid signature but a body that fails the schema check (no `space.id`) → 2xx plus a log entry, no crash, no mission. It must not be 5xx: 5xx would make Photon retry a bad event for days.
- **W15** The handler responds in < 1 s even when the Jetson or Photon send is slow. Work is queued, never done inside the request.

### D — Duplicates and idempotency [L1 store, L2 route]
- **D1** The same `webhook-id` delivered twice (Photon retry) → one mission, one "Request received" reply, and both deliveries get 2xx.
- **D2** Same event ID, different timestamp and re-signed (a real retry looks like this) → still a duplicate.
- **D3** Two *different* event IDs both saying `/find blue toolbox` from the same space within 10 s (the customer double-tapped) → one mission, and the second gets "Already searching for blue toolbox".
- **D4** Crash between saving the event and creating the mission → after restart, the event is either fully done or not done (one SQLite transaction). The test simulates an exception inside the transaction.
- **D5** Storage unavailable (DB locked or read-only) → **503**, so Photon retries.
- **D6** `POST /api/missions` with the same `Idempotency-Key` twice → the same mission_id.
- **D7** An outbound Photon send for the same logical reply uses a deterministic `idempotency-key` (e.g. `{mission_id}:{state}`). Retrying after a timeout must not double-post to Telegram.

### M — Mission manager (`missions.py`) [L1]
- **M1** Create → `mission_id` is a server-generated UUID4. The customer cannot supply it.
- **M2** `issued_at` = now and `expires_at` = now + TTL (configurable, default e.g. 10 min), both ISO-8601 UTC with `Z`.
- **M3** While a mission is active, a new `/find` → refused with the active target named. After a terminal state, `/find` works again.
- **M4** Legal transitions only: `PENDING → ACCEPTED → SEARCHING → TARGET_DETECTED → APPROACHING → COMPLETED`; `SEARCHING → FAILED`; `TARGET_DETECTED → SEARCHING` (target lost); any non-terminal → `CANCELLED` / `FAILED`. Illegal (e.g. `PENDING → COMPLETED`, or anything after `COMPLETED`) → rejected, logged, not shown to the customer.
- **M5** Terminal states (`COMPLETED`, `FAILED`, `CANCELLED`) are final. A late `SEARCHING` after `CANCELLED` is ignored.
- **M6** Expiry: no terminal state by `expires_at` → the backend marks it `FAILED (expired)`, sends a `MISSION_CANCEL` to the Jetson if connected, and tells the customer.
- **M7** ACK timeout: no `MISSION_ACK` within N s (`silent` script) → `FAILED`, "The robot didn't respond".
- **M8** `GET /api/missions/{id}` → the state, target, timestamps and last message. An unknown ID → 404. A malformed (non-UUID) ID → 422.
- **M9** `POST /api/missions/{id}/cancel` on an active mission → sends `MISSION_CANCEL` and the state shows "cancel requested" **until the Jetson confirms `CANCELLED`**. The backend does not claim the robot stopped.
- **M10** Cancel on a terminal or unknown mission → 409 or 404, with a clear message.
- **M11** `/status` and `/cancel` from customer B cannot see or cancel customer A's mission. Ownership = the `conversation_id` + `sender_id` that created it.
- **M12** The `/api/missions*` REST endpoints require an operator token. They are not public; without it → 401.

### J — Protocol validation (`protocol.py`), shared by both sides [L1]
- **J1** A valid `MISSION_REQUEST` round-trips encode → decode with every field intact.
- **J2** Missing a field, a wrong type, an extra unknown field, or an unknown `type` → `ProtocolError`. Never a crash.
- **J3** `mission_type` other than `FIND_OBJECT` → rejected.
- **J4** `target_description` that fails the P-rules (too long, control characters) → rejected on the **Jetson side too**. Defence in depth: don't trust the backend.
- **J5** Timestamps that are non-ISO, without a timezone, NaN, or year 9999 → rejected.
- **J6** `MISSION_STATUS.status` not in the 8 states → rejected.
- **J7** Frames over 4 KB → rejected before parsing.

### A — Jetson agent (`agent.py`) with `ScriptedSink` and a fake clock [L1]
- **A1** A valid request → `MISSION_ACK ACCEPTED`, then `sink.start(mission_id, "blue toolbox")` is called **once**.
- **A2** `expires_at` already passed → `MISSION_ACK REJECTED reason=EXPIRED`, sink not called.
- **A3** `issued_at` more than 30 s in the future (clock skew or forgery) → `REJECTED reason=CLOCK_SKEW`.
- **A4** The same `mission_id` received twice (replay or a resend after reconnect) → the second gets the same ACK result again, the sink is **not** started twice.
- **A5** A different mission while one is active → `REJECTED reason=BUSY`.
- **A6** A malformed frame → `ERROR` frame or log, the agent keeps running, the connection stays up.
- **A7** `MISSION_CANCEL` for the active mission → `sink.cancel` is called; the agent sends `CANCELLED` **only after the sink confirms**.
- **A8** `MISSION_CANCEL` for an unknown or finished mission → ignored with a log, no sink call.
- **A9** The sink reports a status → the agent emits `MISSION_STATUS` with the right `mission_id`. Statuses from the sink for a non-active mission are dropped.
- **A10** The agent never sets a status on its own except `ACCEPTED`/`REJECTED`. It reports what the sink says. This proves the Jetson reports the "actual state".
- **A11** The link drops mid-mission → `sink.on_link_lost()` is called once. The agent does **not** auto-resume after reconnect. On reconnect it reports the mission's current real state.
- **A12** A mission expires on the Jetson side while active → the agent calls `sink.cancel` and reports `FAILED reason=EXPIRED`.
- **A13** The `replay` cache of seen mission_ids is bounded (e.g. 1000) and kept until well past `expires_at`.

### L — Jetson link and auth (`robot_link.py`, `WS /ws/robot`) [L1 logic, L2/L3 real WS]
- **L1** Connect with no token, a wrong token or a wrong `JETSON_DEVICE_ID` → closed with policy-violation code 1008, nothing sent.
- **L2** The token is accepted only in a header or the first `HELLO` frame, **never in the query string**, which ends up in logs. The test asserts query tokens are refused.
- **L3** Token compare uses `hmac.compare_digest`.
- **L4** A second connection for the same device ID → the policy decides (proposal: the newest wins and the old one is closed). The test pins the chosen rule.
- **L5** `/find` while no Jetson is connected → "The robot is currently unavailable", and the mission is **not** left PENDING forever (`FAILED: robot offline`, or not created).
- **L6** The Jetson disconnects mid-mission (`drop_mid_mission`) → the customer gets "Lost contact with the robot. It will stop on its own for safety." The mission becomes `FAILED (link lost)` after a grace period (e.g. 15 s) if it doesn't reconnect.
- **L7** The Jetson reconnects within the grace period → the agent re-reports the current state, the backend resumes updates, and the customer gets no duplicate messages.
- **L8** Heartbeat: no ping/pong for N s → the connection is treated as dead on **both** sides, so a half-open TCP connection is detected.
- **L9** The Jetson sends a status for a mission it wasn't sent (`garbage`) → ignored and logged.
- **L10** The WebSocket only carries JSON text frames. A binary frame → the connection is closed.

### N — Customer messages (`notifier.py`) [L1, L2 through the fake provider]
- **N1** Each state maps to exactly the text in the spec, e.g. `SEARCHING` → "The robot has started searching for: blue toolbox."
- **N2** No message contains "found" or "detected" unless the Jetson reported `TARGET_DETECTED`. A parametrised check runs over all states.
- **N3** `/start` → the intro text. `/help` → lists all 5 commands.
- **N4** A reply goes to the **same** `spaceId` the request came from, with `senderId` = our bot user. Never to another customer's space.
- **N5** Status changes for a mission are delivered in order, and repeated identical statuses are collapsed (no "searching…" spam).
- **N6** Photon send returns 5xx or 429 → retried with backoff (honouring `Retry-After`), the same idempotency-key, max N attempts, then logged. **Mission processing does not stop** because Telegram failed.
- **N7** Photon send returns 4xx `problem+json` (e.g. 422) → not retried, logged with the problem `code`.
- **N8** Photon timeout → retried. Thanks to D7 this does not double-post.
- **N9** The API key never appears in logs, exceptions or responses. A test captures the logs and greps for the key.

### S — Security and safety [L1/L2]
- **S1** The protocol schema has **no motion, speed or shell field**. There is a test listing the allowed message types and fields, and adding one fails the test until it is reviewed.
- **S2** Nothing in `locatebot/backend` or `locatebot/dispatch` imports `subprocess` or calls `os.system`, `eval` or `exec` (static check test).
- **S3** `.env` is git-ignored (already true), and `.env.example` contains only placeholders. A test fails if it has a value matching `pho_sk_[A-Za-z0-9]{8,}` or `whsec_[A-Za-z0-9+/]{8,}`.
- **S4** An unsigned or forged webhook can't create a mission, even with a valid-looking body (W2 + D-route).
- **S5** Rate limit per conversation (e.g. 5 `/find` per minute) → refused with a reply. This keeps a spammer from flooding the Jetson.
- **S6** The backend restarts mid-mission → missions reload from SQLite. The Jetson reconnects and re-reports, and the customer gets a correct `/status`.

### E — End-to-end scenarios [L2 automated; L3/L4 by hand with the same steps]
| ID | Steps | Expected customer messages, in order |
|---|---|---|
| E1 happy path | Jetson `happy`, send `/find blue toolbox` | Request received → started searching for: blue toolbox → Target detected. Preparing to approach it. → Search mission completed. |
| E2 not found | `not_found` | Request received → started searching → "I couldn't find blue toolbox." No "detected" message. |
| E3 robot offline | No Jetson, `/find mug` | "Robot unavailable"; no mission left PENDING |
| E4 duplicate | Same webhook delivered 3× | One mission, one set of replies |
| E5 cancel | `stall`, then `/cancel` | …searching → "Cancelling…" → "Mission cancelled." (only after the Jetson's CANCELLED) |
| E6 disconnect | `drop_mid_mission` | …searching → "Lost contact with the robot…" → after grace: "Mission ended: robot offline" |
| E7 reconnect | Drop, then reconnect within the grace period | Updates continue, no duplicates |
| E8 expired | TTL = 5 s, `stall` | "Mission timed out" + cancel sent to the Jetson |
| E9 busy | `/find a` then `/find b` | Second: "Already searching for a — /cancel first" |
| E10 bad input | `/find`, `/find ;reboot`, a photo | Ask for a description / rejected / "send text"; Jetson receives nothing |
| E11 forged | Unsigned POST with a `/find` body | HTTP 400; no Telegram reply; Jetson receives nothing |
| E12 two customers | A and B in different spaces | Each sees only their own updates; B can't cancel A's mission |

### L4 / L5 manual checklist (real Photon, real Jetson, no motors)
1. In the Photon dashboard: connect a Telegram bot, add a webhook destination to the tunnel URL with only `message.received`, and save `whsec_` to `.env`.
2. Send `/start`. A reply arrives within ~3 s. Save the raw delivery (from the logs) as `tests/fixtures/photon_message_received.json` after removing personal data.
3. Run E1, E3, E5 and E6 from a real phone with `fake_jetson.py` on the laptop.
4. Repeat E1 and E6 with the agent running **on the Jetson** (ScriptedSink). For E6, pull the Wi-Fi or Ethernet and confirm `on_link_lost` is logged on the Jetson.
5. Record: the round-trip time from Telegram send to the "started searching" reply, any Photon delivery failures in the dashboard log, and the reconnect time.

## Order to build in (test-first, one slice per commit)
1. `protocol.py` + J tests. 2. `commands.py` + P. 3. Photon verify and parse + W1–W14. 4. `missions.py` + M, D-store. 5. `agent.py` + `ScriptedSink` + A. 6. `notifier.py` + N1–N5. 7. FastAPI routes + `robot_link` + L, D-route, E (L2). 8. Photon send client + N6–N9. 9. `fake_jetson.py`, `send_fake_webhook.py` → L3. 10. L4/L5 by hand.
