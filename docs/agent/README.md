# LLM agent (`locatebot/agent/`)

The LLM decides *what* the robot does next by calling one skill tool; Nav-lite decides *how*. See `docs/PROJECT.md` §8a. Only shared contracts (`locatebot.mission.types`, `locatebot.lidar.types`) and the stdlib are imported. No new dependencies.

## Modules
| File | API |
|---|---|
| `summary.py` | `render_world_state(ws) -> str`: pose/heading, explored %, mission, safety; sector clearances (8 sectors named front, front-left, …; other counts in degrees; NaN/inf = `unknown`); up to 6 items (confirmed first, then most recent) with range, bearing (`30° left`, `behind (170° right)`), age; skill line; last event. Deterministic, one-line fields, truncated. |
| `tools.py` | `tool_definitions()` -> `[{"name","description","input_schema"}]`, one per `SkillName`, JSON Schema with `additionalProperties: false` and an optional `reason` string. `parse_tool_call(name, args, source="llm") -> SkillCall` raises `InvalidSkillCall`. `validate_skill_call(call)` re-checks operator/fallback calls. |
| `llm.py` | `LLMClient` protocol: `decide(system, user, tools, timeout_s) -> ToolChoice(name, args, text) \| None`. `ClaudeClient`, `OpenAICompatClient`, `FakeLLM`. Errors: `LLMError(status, retryable)`, `LLMTimeout`. |
| `brain.py` | `Brain(llm, mode=..., ...)`: `on_world_state(ws)`, `poll()`, `pending()`, `approve()`, `reject()`, `submit_operator(call)`, `resume()`, `set_mode()`, `status()`, `history()`, `wait_idle()`. |

## Validation rules (`parse_tool_call`)
Unknown skill, non-object args, unknown or missing args → rejected. Strings: non-empty after whitespace collapse, ≤ 40 chars, no control characters. Numbers: int or float (not bool), finite, |v| ≤ 100 m. `search.items`: list of 1–8 such strings (duplicates dropped). `go_to`: exactly `item`, or both `x` and `y`.

## Clients
- **`ClaudeClient(model="claude-opus-5-5", base_url="https://api.anthropic.com", max_tokens=16000, effort="low", refusal_fallback=True)`**. Raw HTTP to `POST /v1/messages` with `anthropic-version: 2026-10-21`. Key from `ANTHROPIC_API_KEY` at call time, never stored or logged. `tool_choice: {"type":"auto","disable_parallel_tool_use":true}`: forced `any`/`tool` returns a 400 on Opus 5.5, so the system prompt asks for a tool call. Thinking stays at the model default (adaptive; Opus 5.5 can't turn it off), and effort is set explicitly (`low`, because decisions are small and latency matters). No sampling params: version `2026-10-21` rejects them. `fallbacks: "default"` + `anthropic-beta: server-side-fallback-2026-07-01` retries classifier refusals on another model server-side. The system block has `cache_control` so the fixed tools and system prefix are cached. Refusal, `max_tokens` truncation, bad shapes, HTTP errors (429/5xx/408/409 flagged retryable) → `LLMError`.
- **`OpenAICompatClient(base_url="http://127.0.0.1:11434/v1", model="qwen2.5:7b-instruct", api_key_env=None)`** for Ollama or llama.cpp server: `POST {base_url}/chat/completions`, tools as `{"type":"function","function":{name, description, parameters}}`, `tool_choice: "auto"`. The model name is a placeholder: pick one that supports tool calls.
- **`NemotronClient(model="nvidia/nemotron-3.5-super-vl-120b-a12b", base_url="https://integrate.api.nvidia.com/v1", thinking=False, max_tokens=2048)`**: NVIDIA-hosted Nemotron, a configured `OpenAICompatClient`. Key from `NVIDIA_API_KEY` at call time (missing key → non-retryable `LLMError`). Sends `chat_template_kwargs: {"enable_thinking": false}` (reasoning tokens count against `max_tokens`) and `parallel_tool_calls: false`. `tools/orin_run.py --llm nemotron`. **Verified live 2026-11-08:** tool calls (~0.8 s) and image input as a data URL (~1.1 s); the response reports model `private/nvidia/nemotron-3.5-super-120b-a12b`. In `--sim --mode auto` it chose `explore()` as the first skill. Needs internet.
- **`chat(messages, tools=None, timeout_s, max_tokens=None) -> ChatReply(content, tool_calls, finish_reason, message)`** on `OpenAICompatClient`/`NemotronClient`: one multi-turn request for callers that keep their own history (tool results, images via `image_part(bytes, mime)`). `finish_reason: length` raises `LLMTruncated` (an `LLMError`), in `decide` too.
- **Timeouts:** each request runs in a daemon thread with socket timeout = `timeout_s`, and the caller gets `LLMTimeout` at `timeout_s` wall-clock. The Brain also enforces `llm_timeout_s` on its own clock.

## Brain behaviour
- **When it asks:** first state; skill entered SUCCEEDED/FAILED/CANCELLED; a newly *confirmed* item; safety reason changed to something other than `ok`/`slowed`; `last_event` changed; or every `decide_interval_s` while idle (no skill running, nothing pending). Triggers that arrive during a request are kept and served after it.
- **One request at a time**, in a background thread. `on_world_state` only takes a short lock and never waits on the LLM.
- **auto:** a valid call is emitted via `on_skill`. **approve:** it becomes `pending()`. `approve()` emits it, `reject()` drops it, and a newer proposal replaces a stale one (history outcome `replaced`).
- **Failures:** timeout, `LLMError`, invalid call or no tool call. Each one counts, sets `last_error`, and retries after `retry_delay_s`. After `max_failures` in a row (or with `llm=None`, or `max_abandoned` timed-out threads still alive) the brain enters **fallback**: the routine (default `[STOP]`) runs one step at a time, starting once the current skill is no longer running. It moves to the next step when the world state shows that step's call in a terminal state with a newer `stamp_s`. During fallback the LLM is probed every `decide_interval_s`, and the first valid answer ends fallback and is handled normally.
- **Operator:** `submit_operator(call)` validates the call and emits it at once (source `operator`). It cancels the pending proposal and the fallback routine. `STOP` also abandons the in-flight request (its late result is dropped) and pauses decisions until `resume()`.
- **History:** the last `history_len` (8) decisions with outcomes and skill results go into every prompt.
- **`status()`:** `mode, paused, pending, failures, total_failures, in_fallback, fallback_step, last_error, last_latency_s, inflight, history`.
- `on_skill` runs with the brain's lock held, so a late result can't overtake an operator STOP. Keep it quick (put the call on a queue).

## Limits
- Not run against a real API or a local model: the tests use canned JSON and a 127.0.0.1 server. The model's skill choices are untested.
- A cloud LLM breaks the "no internet at runtime" rule (§8a open question). Where it runs is still undecided.
- Abandoned (timed-out) threads can't be killed. They end on their socket timeout, and the Brain caps them with `max_abandoned`.
- A fallback step whose call the executor never shows in the world state (rejected, or replaced by another call) stalls the routine. The robot then holds its last state until the LLM or the operator acts.
- After operator `STOP` + `resume()`, an unfinished fallback routine doesn't restart. The robot stays idle until the LLM answers.
- Local models that put tool calls in plain text instead of `tool_calls` count as "no tool call".
