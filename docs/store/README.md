# Store-guide agent (`locatebot/store/`)

A shopper texts; the agent checks stock, offers a guide, leads them to the aisle with the robot, finds the item with LocateAnything and checks it with Nemotron VL. Branch `pooja`. The LLM is `NemotronClient` (`locatebot/agent/llm.py`).

## Modules
| File | API |
|---|---|
| `catalog.py` | `Product(id, name, aisle, shelf, qty, aliases, attributes, grounding_query)`; `Catalog.search(query, limit=3)`; `MemoryCatalog` (+ `from_json`). Search: exact name/alias, all words present, or fuzzy (≥ 0.75). Exact matches win; several matches = ambiguous. |
| `robot.py` | `Robot.go(waypoint) -> "arrived"/"blocked"/"failed"`, `Robot.frame() -> jpeg`; `Locator.locate(jpeg, query) -> [Box]`; `Box` in 0..1 frame fractions; `position_text(box)` → `"middle shelf, left side"`; `MockRobot`, `FakeLocator`; `crop_jpeg` (needs Pillow, else `None`). |
| `agent.py` | `StoreAgent(llm, catalog, robot, locator, verifier, StoreConfig(store_code=...))`, `handle_text(user, text) -> reply`. |
| `verify.py` | `NemotronVerifier(llm).verify(frame, box, product) -> Verdict(match, confidence, reason)`; strict JSON reply. Without Pillow, sends the whole frame and describes the box in words. |
| `enrich.py` | `enrich_product(llm, name, image=, notes=) -> attributes` with exactly `category, major_color, shape, text_on_label, distinguishing, grounding_query`; retries once on bad JSON. |

Tools: `tools/store_agent.py --chat | --serve` (POST `/message` `{user, text}` → `{reply}`), `tools/enrich_products.py`. Sample store: `data/mock_store.json`. Secrets: `.env` (`NVIDIA_API_KEY`, `STORE_CODE`), template `.env.example`.

## Rules the code enforces (not the model)
| State | Tools exposed |
|---|---|
| IDLE | search_product |
| OFFERED | search_product, record_decision |
| WANTS_GUIDE | search_product |
| GUIDING | navigate_to, find_in_view, verify_crop, finish_guidance |

- Out of stock keeps IDLE. Check-in = the message is exactly `STORE_CODE` while in WANTS_GUIDE, matched in code.
- Every tool call is re-checked against the state; `navigate_to` accepts only the current product's aisle or `ENTRANCE`; `find_in_view`/`verify_crop` only after `navigate_to` returned `arrived` there.
- One guided shopper at a time (`robot_owner`); `finish_guidance` sends the robot home and frees it.
- **Grounding guard:** a reply that names an aisle no tool returned this turn (other than the current product's), or claims stock with no lookup, is sent back once with a correction; a second time the shopper gets "which product are you looking for?".
- `max_steps` (8) per message; API error or truncation → one retry (truncation with 2× `max_tokens`), then the staff fallback. First reply to a new user starts with the mock-inventory notice. History keeps the last 6 text turns, no tool messages.

## Live findings (Nemotron, 2026-11-08, mock robot + fake locator)
- Full flow worked: search → offer → code → `navigate_to` → `find_in_view` → `verify_crop` → honest "shelf looks empty" (frame had no paper towels; verifier said `match: false`) → `finish_guidance`.
- The model sent stray args to no-arg tools (`find_in_view({"box": "0"})`) and the box index as a string; both now accepted.
- Without the guard it answered "chips?" with an invented "Aisle 4" and no tool call.
- Rejected: drive to the parking lot; wrong entrance code.

## Limits / not done
- The robot is a mock. A real `Robot` must go through the Pi LiDAR safety stop; `navigate_to` blocks the reply until the robot stops.
- `--locator locate-anything` wraps `locatebot.vision.engine` but was not run here (needs the built engine and model on a GPU).
- The grounding guard is a regex for aisles and stock words, not a full fact check; claims about the shelf rely on the prompt plus `verify_crop`.
- No session expiry: a shopper who never finishes keeps the robot until `finish_guidance`.
- SpacetimeDB catalog, Photon bridge wiring and `.lbmap`-based navigation are not built. Frames sent to the hosted API must contain no people.
