import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from locatebot.agent.llm import (
    ANTHROPIC_VERSION,
    DEFAULT_CLAUDE_MODEL,
    ClaudeClient,
    FakeLLM,
    LLMError,
    LLMTimeout,
    OpenAICompatClient,
    ToolChoice,
)
from locatebot.agent.tools import tool_definitions

TOOLS = tool_definitions()


class _Server:
    """Local 127.0.0.1 HTTP server with a scripted response; records requests."""

    def __init__(self):
        self.requests = []
        self.status = 200
        self.body = b"{}"
        self.delay_s = 0.0
        self.release = threading.Event()
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                outer.requests.append((self.path, dict(self.headers), json.loads(self.rfile.read(n))))
                if outer.delay_s:
                    outer.release.wait(outer.delay_s)
                body = outer.body if isinstance(outer.body, bytes) else json.dumps(outer.body).encode()
                try:
                    self.send_response(outer.status)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except OSError:
                    pass

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.httpd.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True).start()

    def close(self):
        self.release.set()
        self.httpd.shutdown()
        self.httpd.server_close()


@pytest.fixture
def server():
    s = _Server()
    yield s
    s.close()


CLAUDE_TOOL_RESPONSE = {
    "id": "msg_1", "type": "message", "role": "assistant", "model": "claude-opus-5-5",
    "stop_reason": "tool_use",
    "content": [
        {"type": "thinking", "thinking": "", "signature": "x"},
        {"type": "text", "text": "Marker is confirmed, driving to it."},
        {"type": "tool_use", "id": "toolu_1", "name": "go_to", "input": {"item": "marker"}},
    ],
    "usage": {"input_tokens": 10, "output_tokens": 5},
}


def test_claude_request_shape(server, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-test-not-real")
    server.body = CLAUDE_TOOL_RESPONSE
    c = ClaudeClient(base_url=server.url)
    out = c.decide("SYS", "USER", TOOLS, timeout_s=5.0)
    assert out == ToolChoice("go_to", {"item": "marker"}, "Marker is confirmed, driving to it.")
    path, headers, body = server.requests[0]
    assert path == "/v1/messages"
    h = {k.lower(): v for k, v in headers.items()}
    assert h["x-api-key"] == "sk-test-not-real"
    assert h["anthropic-version"] == ANTHROPIC_VERSION
    assert h["content-type"] == "application/json"
    assert "server-side-fallback-2026-07-01" in h["anthropic-beta"]
    assert body["model"] == DEFAULT_CLAUDE_MODEL == "claude-opus-5-5"
    assert body["system"][0]["text"] == "SYS" and body["system"][0]["cache_control"] == {"type": "ephemeral"}
    assert body["messages"] == [{"role": "user", "content": "USER"}]
    assert body["tools"] == TOOLS
    # forced tool choice ("any"/"tool") is a 400 on this model: auto + one call at most
    assert body["tool_choice"] == {"type": "auto", "disable_parallel_tool_use": True}
    assert body["output_config"] == {"effort": "low"}
    assert body["fallbacks"] == "default"
    for k in ("temperature", "top_p", "top_k", "thinking"):
        assert k not in body


def test_claude_options(server, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "k")
    server.body = CLAUDE_TOOL_RESPONSE
    ClaudeClient(model="claude-sonnet-5-5", base_url=server.url + "/", effort="medium",
                 refusal_fallback=False, max_tokens=2000).decide("s", "u", TOOLS, 5.0)
    _, headers, body = server.requests[0]
    assert body["model"] == "claude-sonnet-5-5" and body["max_tokens"] == 2000
    assert body["output_config"] == {"effort": "medium"}
    assert "fallbacks" not in body
    assert "anthropic-beta" not in {k.lower() for k in headers}


def test_claude_no_tool_returns_none(server, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "k")
    server.body = {"stop_reason": "end_turn", "content": [{"type": "text", "text": "nothing to do"}]}
    assert ClaudeClient(base_url=server.url).decide("s", "u", TOOLS, 5.0) is None


def test_claude_parse_errors():
    p = ClaudeClient._parse_response
    with pytest.raises(LLMError, match="refusal"):
        p({"stop_reason": "refusal", "content": [], "stop_details": {"category": "cyber"}})
    with pytest.raises(LLMError, match="max_tokens"):
        p({"stop_reason": "max_tokens", "content": [{"type": "tool_use", "name": "stop", "input": {}}]})
    with pytest.raises(LLMError):
        p({"stop_reason": "tool_use", "content": [{"type": "tool_use", "name": "stop", "input": "x"}]})
    with pytest.raises(LLMError):
        p({"content": "nope"})
    with pytest.raises(LLMError):
        p([])


def test_claude_missing_key(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    with pytest.raises(LLMError, match="ANTHROPIC_API_KEY") as e:
        ClaudeClient(base_url="http://127.0.0.1:9").decide("s", "u", TOOLS, 1.0)
    assert e.value.retryable is False


def test_claude_http_error_is_typed_and_hides_key(server, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-secret-123")
    server.status = 529
    server.body = {"type": "error", "error": {"type": "overloaded_error", "message": "Overloaded"}}
    c = ClaudeClient(base_url=server.url)
    with pytest.raises(LLMError) as e:
        c.decide("s", "u", TOOLS, 5.0)
    assert e.value.status == 529 and e.value.retryable is True
    assert "Overloaded" in str(e.value) and "sk-secret" not in str(e.value)
    assert "sk-secret" not in repr(c)
    server.status = 400
    with pytest.raises(LLMError) as e:
        c.decide("s", "u", TOOLS, 5.0)
    assert e.value.status == 400 and e.value.retryable is False


def test_claude_bad_json(server, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "k")
    server.body = b"<html>proxy</html>"
    with pytest.raises(LLMError, match="JSON"):
        ClaudeClient(base_url=server.url).decide("s", "u", TOOLS, 5.0)


def test_timeout_enforced(server, monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "k")
    server.delay_s = 5.0
    for client in (ClaudeClient(base_url=server.url), OpenAICompatClient(base_url=server.url, model="m")):
        t0 = time.monotonic()
        with pytest.raises(LLMTimeout):
            client.decide("s", "u", TOOLS, timeout_s=0.3)
        assert time.monotonic() - t0 < 1.5


def test_connection_refused_is_llm_error():
    with pytest.raises(LLMError):
        OpenAICompatClient(base_url="http://127.0.0.1:9/v1", model="m").decide("s", "u", TOOLS, 1.0)


def test_openai_request_and_parse(server, monkeypatch):
    server.body = {"choices": [{"finish_reason": "tool_calls", "message": {
        "role": "assistant", "content": "ok",
        "tool_calls": [{"id": "c1", "type": "function",
                        "function": {"name": "search", "arguments": "{\"items\": [\"marker\"]}"}}]}}]}
    monkeypatch.setenv("LOCAL_LLM_KEY", "abc")
    c = OpenAICompatClient(base_url=server.url + "/v1", model="qwen-test", api_key_env="LOCAL_LLM_KEY")
    out = c.decide("SYS", "USER", TOOLS, 5.0)
    assert out == ToolChoice("search", {"items": ["marker"]}, "ok")
    path, headers, body = server.requests[0]
    assert path == "/v1/chat/completions"
    assert {k.lower(): v for k, v in headers.items()}["authorization"] == "Bearer abc"
    assert body["model"] == "qwen-test" and body["stream"] is False
    assert body["messages"] == [{"role": "system", "content": "SYS"}, {"role": "user", "content": "USER"}]
    assert body["tool_choice"] == "auto"
    t = body["tools"][0]
    assert t["type"] == "function" and t["function"]["name"] == "explore"
    assert t["function"]["parameters"] == TOOLS[0]["input_schema"]


def test_openai_parse_variants():
    p = OpenAICompatClient._parse_response
    msg = lambda m, fr="stop": {"choices": [{"finish_reason": fr, "message": m}]}
    assert p(msg({"content": "hi"})) is None
    assert p(msg({"content": None, "tool_calls": [{"function": {"name": "stop", "arguments": {}}}]})) == \
        ToolChoice("stop", {}, "")
    assert p(msg({"tool_calls": [{"function": {"name": "stop", "arguments": ""}}]})) == ToolChoice("stop", {}, "")
    with pytest.raises(LLMError):
        p(msg({"tool_calls": [{"function": {"name": "stop", "arguments": "{bad"}}]}))
    with pytest.raises(LLMError):
        p(msg({"content": "x"}, fr="length"))
    with pytest.raises(LLMError):
        p({"choices": []})


def test_fake_llm_script():
    err = LLMError("boom")
    f = FakeLLM([ToolChoice("stop", {}), None, err], default=ToolChoice("explore", {}))
    assert f.decide("s", "u", [], 1.0) == ToolChoice("stop", {})
    assert f.decide("s", "u", [], 1.0) is None
    with pytest.raises(LLMError):
        f.decide("s", "u", [], 1.0)
    assert f.decide("s", "u", [], 1.0) == ToolChoice("explore", {})
    assert len(f.calls) == 4 and f.calls[0] == ("s", "u", [], 1.0)


def test_fake_llm_hang_until_released():
    f = FakeLLM([ToolChoice("stop", {})])
    f.hang.clear()
    out = []
    t = threading.Thread(target=lambda: out.append(f.decide("s", "u", [], 1.0)))
    t.start()
    assert f.started.wait(1.0)
    time.sleep(0.05)
    assert out == [] and f.active == 1
    f.hang.set()
    t.join(1.0)
    assert out == [ToolChoice("stop", {})] and f.active == 0 and f.max_active == 1
