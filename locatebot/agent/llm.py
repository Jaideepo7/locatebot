"""Pluggable LLM clients: one decision = one request that returns at most one tool call.

All HTTP is stdlib (``urllib``); no SDK dependency. Every failure (HTTP status, network, bad
JSON, unexpected response shape, refusal, truncation) is raised as ``LLMError``; a request
that exceeds ``timeout_s`` raises ``LLMTimeout`` (a subclass).

- ``ClaudeClient``: Anthropic Messages API, ``POST {base_url}/v1/messages``. API key read from
  the ``ANTHROPIC_API_KEY`` environment variable at call time, never stored in the repo.
- ``OpenAICompatClient``: local OpenAI-compatible server (Ollama, llama.cpp server),
  ``POST {base_url}/chat/completions`` with ``tools``.
- ``FakeLLM``: scripted responses for tests.
"""
from __future__ import annotations

import json
import os
import socket
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Protocol

DEFAULT_CLAUDE_MODEL = "claude-opus-5-5"
ANTHROPIC_VERSION = "2026-10-21"
ANTHROPIC_FALLBACK_BETA = "server-side-fallback-2026-07-01"
DEFAULT_ANTHROPIC_URL = "https://api.anthropic.com"
DEFAULT_LOCAL_URL = "http://127.0.0.1:11434/v1"  # Ollama's OpenAI-compatible endpoint
DEFAULT_LOCAL_MODEL = "qwen2.5:7b-instruct"
MAX_RESPONSE_BYTES = 4 * 1024 * 1024


@dataclass(frozen=True)
class ToolChoice:
    name: str
    args: dict = field(default_factory=dict)
    text: str = ""          # any text the model wrote alongside the call


class LLMError(Exception):
    def __init__(self, message: str, *, status: int | None = None, retryable: bool = True):
        super().__init__(message)
        self.status = status
        self.retryable = retryable


class LLMTimeout(LLMError):
    def __init__(self, message: str = "timeout"):
        super().__init__(message, retryable=True)


class LLMClient(Protocol):
    def decide(self, system: str, user: str, tools: list[dict], timeout_s: float) -> ToolChoice | None:
        """Ask for one decision. None = the model answered without calling a tool."""
        ...


# --------------------------------------------------------------------------- HTTP helper

def _read_with_deadline(resp, deadline: float) -> bytes:
    chunks: list[bytes] = []
    total = 0
    while True:
        if time.monotonic() > deadline:
            raise LLMTimeout("timeout while reading response")
        chunk = resp.read1(65536) if hasattr(resp, "read1") else resp.read(65536)
        if not chunk:
            return b"".join(chunks)
        total += len(chunk)
        if total > MAX_RESPONSE_BYTES:
            raise LLMError("response too large", retryable=False)
        chunks.append(chunk)


def _error_summary(raw: bytes) -> str:
    try:
        data = json.loads(raw)
        err = data.get("error", data) if isinstance(data, dict) else data
        if isinstance(err, dict):
            return f"{err.get('type', '')}: {err.get('message', '')}".strip(": ")[:300]
        return str(err)[:300]
    except (ValueError, AttributeError):
        return raw[:200].decode("utf-8", "replace")


def _post_once(url: str, headers: dict[str, str], body: dict, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            raw = _read_with_deadline(resp, deadline)
    except urllib.error.HTTPError as e:
        try:
            raw = e.read(65536)
        except Exception:  # noqa: BLE001 - best effort body for the message
            raw = b""
        status = e.code
        retryable = status in (408, 409, 429) or status >= 500
        raise LLMError(f"HTTP {status}: {_error_summary(raw)}", status=status, retryable=retryable) from None
    except LLMError:
        raise
    except (socket.timeout, TimeoutError):
        raise LLMTimeout("timeout") from None
    except urllib.error.URLError as e:
        if isinstance(e.reason, (socket.timeout, TimeoutError)):
            raise LLMTimeout("timeout while connecting") from None
        raise LLMError(f"connection failed: {e.reason}") from None
    except (OSError, ValueError) as e:  # connection reset, bad URL, http.client errors
        raise LLMError(f"request failed: {type(e).__name__}: {e}") from None
    try:
        return json.loads(raw)
    except ValueError:
        raise LLMError("response is not valid JSON") from None


def post_json(url: str, headers: dict[str, str], body: dict, timeout_s: float) -> Any:
    """POST JSON and parse the JSON reply; hard wall-clock limit of ``timeout_s``.

    The request runs in a daemon thread; if it is not done by the deadline ``LLMTimeout`` is
    raised and the thread is left to die on its own socket timeout (also ``timeout_s``).
    """
    if not (timeout_s > 0):
        raise LLMError("timeout_s must be > 0", retryable=False)
    box: dict[str, Any] = {}

    def run() -> None:
        try:
            box["ok"] = _post_once(url, headers, body, timeout_s)
        except BaseException as e:  # noqa: BLE001 - re-raised in the caller
            box["err"] = e

    t = threading.Thread(target=run, name="llm-http", daemon=True)
    t.start()
    t.join(timeout_s)
    if t.is_alive():
        raise LLMTimeout(f"no response within {timeout_s:g} s")
    if "err" in box:
        err = box["err"]
        if isinstance(err, LLMError):
            raise err
        raise LLMError(f"request failed: {type(err).__name__}: {err}")
    return box.get("ok")


# --------------------------------------------------------------------------- Claude

class ClaudeClient:
    """Anthropic Messages API with tool use, stdlib HTTP.

    Defaults (from the claude-api skill): model ``claude-opus-5-5``; ``anthropic-version:
    2026-10-21`` (so no ``temperature``/``top_p``/``top_k``); ``tool_choice`` ``auto`` with
    ``disable_parallel_tool_use`` because forced ``any``/``tool`` returns a 400 on this model
    (the system prompt tells the model to always call a tool); thinking left at the model
    default (adaptive; it cannot be disabled on Opus 5.5) and ``output_config.effort`` set
    explicitly, ``low`` by default because decisions are short and latency-bound;
    server-side refusal fallback ``fallbacks: "default"`` on by default; the system prompt is
    marked for prompt caching since it and the tools are identical on every request.
    """

    def __init__(
        self,
        model: str = DEFAULT_CLAUDE_MODEL,
        *,
        base_url: str = DEFAULT_ANTHROPIC_URL,
        max_tokens: int = 16000,
        effort: str | None = "low",
        refusal_fallback: bool = True,
        cache_system_prompt: bool = True,
    ) -> None:
        if not model:
            raise ValueError("model must be non-empty")
        self.model = model
        self.url = base_url.rstrip("/") + "/v1/messages"
        self.max_tokens = int(max_tokens)
        self.effort = effort
        self.refusal_fallback = refusal_fallback
        self.cache_system_prompt = cache_system_prompt

    def __repr__(self) -> str:
        return f"ClaudeClient(model={self.model!r}, url={self.url!r})"

    def build_request(self, system: str, user: str, tools: list[dict], api_key: str) -> tuple[dict, dict]:
        headers = {
            "content-type": "application/json",
            "x-api-key": api_key,
            "anthropic-version": ANTHROPIC_VERSION,
        }
        sys_block: dict[str, Any] = {"type": "text", "text": system}
        if self.cache_system_prompt:
            sys_block["cache_control"] = {"type": "ephemeral"}
        body: dict[str, Any] = {
            "model": self.model,
            "max_tokens": self.max_tokens,
            "system": [sys_block],
            "messages": [{"role": "user", "content": user}],
            "tools": tools,
            "tool_choice": {"type": "auto", "disable_parallel_tool_use": True},
        }
        if self.effort:
            body["output_config"] = {"effort": self.effort}
        if self.refusal_fallback:
            headers["anthropic-beta"] = ANTHROPIC_FALLBACK_BETA
            body["fallbacks"] = "default"
        return headers, body

    @staticmethod
    def _parse_response(data: Any) -> ToolChoice | None:
        if not isinstance(data, dict) or not isinstance(data.get("content"), list):
            raise LLMError("unexpected response shape", retryable=False)
        stop = data.get("stop_reason")
        if stop == "refusal":
            details = data.get("stop_details") or {}
            cat = details.get("category") if isinstance(details, dict) else None
            raise LLMError(f"model refusal (category {cat})", retryable=False)
        if stop == "max_tokens":
            raise LLMError("response truncated at max_tokens")
        texts: list[str] = []
        tool = None
        for block in data["content"]:
            if not isinstance(block, dict):
                continue
            if block.get("type") == "text" and isinstance(block.get("text"), str):
                texts.append(block["text"])
            elif block.get("type") == "tool_use" and tool is None:
                tool = block
        text = " ".join(t.strip() for t in texts if t.strip())
        if tool is None:
            return None
        name, args = tool.get("name"), tool.get("input")
        if not isinstance(name, str) or not isinstance(args, dict):
            raise LLMError("malformed tool_use block", retryable=False)
        return ToolChoice(name, args, text)

    def decide(self, system: str, user: str, tools: list[dict], timeout_s: float) -> ToolChoice | None:
        api_key = os.environ.get("ANTHROPIC_API_KEY", "")
        if not api_key:
            raise LLMError("ANTHROPIC_API_KEY is not set", retryable=False)
        headers, body = self.build_request(system, user, tools, api_key)
        return self._parse_response(post_json(self.url, headers, body, timeout_s))


# --------------------------------------------------------------------------- OpenAI-compatible

class OpenAICompatClient:
    """OpenAI-compatible ``/chat/completions`` with ``tools`` (Ollama, llama.cpp ``--jinja``).

    ``api_key_env`` names an environment variable holding a bearer token, if the server needs
    one (Ollama and llama.cpp do not by default).
    """

    def __init__(
        self,
        base_url: str = DEFAULT_LOCAL_URL,
        model: str = DEFAULT_LOCAL_MODEL,
        *,
        api_key_env: str | None = None,
        temperature: float | None = 0.0,
        max_tokens: int | None = 1024,
    ) -> None:
        if not model:
            raise ValueError("model must be non-empty")
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.model = model
        self.api_key_env = api_key_env
        self.temperature = temperature
        self.max_tokens = max_tokens

    def __repr__(self) -> str:
        return f"OpenAICompatClient(model={self.model!r}, url={self.url!r})"

    def build_request(self, system: str, user: str, tools: list[dict]) -> tuple[dict, dict]:
        headers = {"content-type": "application/json"}
        if self.api_key_env:
            key = os.environ.get(self.api_key_env, "")
            if key:
                headers["authorization"] = f"Bearer {key}"
        body: dict[str, Any] = {
            "model": self.model,
            "messages": [{"role": "system", "content": system}, {"role": "user", "content": user}],
            "tools": [
                {"type": "function", "function": {
                    "name": t["name"], "description": t["description"], "parameters": t["input_schema"]}}
                for t in tools
            ],
            "tool_choice": "auto",
            "stream": False,
        }
        if self.temperature is not None:
            body["temperature"] = self.temperature
        if self.max_tokens:
            body["max_tokens"] = self.max_tokens
        return headers, body

    @staticmethod
    def _parse_response(data: Any) -> ToolChoice | None:
        try:
            choice = data["choices"][0]
            msg = choice["message"]
        except (KeyError, IndexError, TypeError):
            raise LLMError("unexpected response shape", retryable=False) from None
        if not isinstance(msg, dict):
            raise LLMError("unexpected message shape", retryable=False)
        if choice.get("finish_reason") == "length":
            raise LLMError("response truncated (finish_reason=length)")
        text = msg.get("content") if isinstance(msg.get("content"), str) else ""
        calls = msg.get("tool_calls") or []
        if not isinstance(calls, list) or not calls:
            return None
        fn = calls[0].get("function") if isinstance(calls[0], dict) else None
        if not isinstance(fn, dict) or not isinstance(fn.get("name"), str):
            raise LLMError("malformed tool call", retryable=False)
        raw = fn.get("arguments", {})
        if isinstance(raw, str):
            if not raw.strip():
                raw = {}
            else:
                try:
                    raw = json.loads(raw)
                except ValueError:
                    raise LLMError("tool arguments are not valid JSON", retryable=False) from None
        if not isinstance(raw, dict):
            raise LLMError("tool arguments are not an object", retryable=False)
        return ToolChoice(fn["name"], raw, text.strip())

    def decide(self, system: str, user: str, tools: list[dict], timeout_s: float) -> ToolChoice | None:
        headers, body = self.build_request(system, user, tools)
        return self._parse_response(post_json(self.url, headers, body, timeout_s))


# --------------------------------------------------------------------------- Fake

ScriptItem = "ToolChoice | None | BaseException | Callable[[str, str, list, float], ToolChoice | None]"


class FakeLLM:
    """Scripted LLM for tests.

    Each ``decide`` pops the next script item: a ``ToolChoice`` or ``None`` is returned, an
    exception instance is raised, a callable is called with the request. When the script is
    empty ``default`` is used. ``delay_s`` sleeps before answering; clearing ``hang`` makes
    calls block until it is set again (``started`` is set when a call begins). ``active`` /
    ``max_active`` count concurrent calls.
    """

    def __init__(self, script: Iterable[Any] = (), *, default: Any = None, delay_s: float = 0.0) -> None:
        self.script: list[Any] = list(script)
        self.default = default
        self.delay_s = delay_s
        self.calls: list[tuple[str, str, list, float]] = []
        self.hang = threading.Event()
        self.hang.set()
        self.started = threading.Event()
        self.active = 0
        self.max_active = 0
        self._lock = threading.Lock()

    def decide(self, system: str, user: str, tools: list[dict], timeout_s: float) -> ToolChoice | None:
        with self._lock:
            self.calls.append((system, user, tools, timeout_s))
            item = self.script.pop(0) if self.script else self.default
            self.active += 1
            self.max_active = max(self.max_active, self.active)
        self.started.set()
        try:
            if self.delay_s:
                time.sleep(self.delay_s)
            self.hang.wait()
            if isinstance(item, BaseException):
                raise item
            if callable(item):
                return item(system, user, tools, timeout_s)
            return item
        finally:
            with self._lock:
                self.active -= 1
