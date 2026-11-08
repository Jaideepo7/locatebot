import base64
import pathlib
import re

import pytest

from locatebot.remote.config import ConfigError, Settings

ROOT = pathlib.Path(__file__).resolve().parents[2]
GOOD = {
    "PHOTON_API_KEY": "pho_sk_" + "a" * 40,
    "PHOTON_PROJECT_ID": "pho_prj_01jabcdefghjkmnpqrstvwxyz1",
    "PHOTON_WEBHOOK_SECRET": "whsec_" + base64.b64encode(b"k" * 32).decode(),
    "OPERATOR_API_TOKEN": "o" * 40,
}


def test_valid_env():
    s = Settings.from_env(GOOD)
    assert s.photon_mode == "live" and s.mission_ttl_s == 600
    assert s.operator_enabled and s.robot_id == "locatebot"
    assert s.photon_api_origin == "https://api.photon.codes" and s.photon_api_version == ""


def test_console_webhook_mode_does_not_need_photon_api_key():
    env = dict(GOOD, PHOTON_MODE="console", PHOTON_INBOUND="webhook")
    del env["PHOTON_API_KEY"]
    assert Settings.from_env(env).photon_mode == "console"


def test_websocket_is_the_default_inbound_and_needs_the_api_key():
    s = Settings.from_env(GOOD)
    assert s.photon_inbound == "websocket" and not s.webhook_enabled
    env = dict(GOOD, PHOTON_MODE="console")
    del env["PHOTON_API_KEY"]
    with pytest.raises(ConfigError) as e:
        Settings.from_env(env)
    assert "PHOTON_API_KEY" in str(e.value)


def test_webhook_mode_needs_the_secret():
    env = dict(GOOD, PHOTON_INBOUND="webhook")
    assert Settings.from_env(env).webhook_enabled
    del env["PHOTON_WEBHOOK_SECRET"]
    with pytest.raises(ConfigError) as e:
        Settings.from_env(env)
    assert "PHOTON_WEBHOOK_SECRET" in str(e.value)


def test_webhook_secret_and_operator_token_are_optional():
    env = {"PHOTON_API_KEY": GOOD["PHOTON_API_KEY"], "PHOTON_PROJECT_ID": GOOD["PHOTON_PROJECT_ID"]}
    s = Settings.from_env(env)
    assert s.photon_inbound == "websocket" and not s.operator_enabled


def test_jetson_and_public_url_settings_are_no_longer_needed():
    s = Settings.from_env(GOOD)
    assert not hasattr(s, "jetson_auth_token") and not hasattr(s, "backend_public_url")


@pytest.mark.parametrize("key,value", [("PHOTON_API_KEY", "sk_live_x"), ("PHOTON_PROJECT_ID", "123"),
                                       ("PHOTON_WEBHOOK_SECRET", "secret"), ("OPERATOR_API_TOKEN", "short"),
                                       ("ROBOT_ID", "bad id!"), ("PHOTON_MODE", "carrier-pigeon"),
                                       ("PHOTON_INBOUND", "carrier-pigeon"), ("PHOTON_API_VERSION", "latest"),
                                       ("PHOTON_API_ORIGIN", "http://api.photon.codes"),
                                       ("MISSION_TTL_S", "-1")])
def test_bad_values_refused(key, value):
    with pytest.raises(ConfigError) as e:
        Settings.from_env(dict(GOOD, **{key: value}))
    assert key in str(e.value)


def test_errors_never_echo_secrets():  # N9
    with pytest.raises(ConfigError) as e:
        Settings.from_env(dict(GOOD, OPERATOR_API_TOKEN="short-secret"))
    assert "short-secret" not in str(e.value)
    r = repr(Settings.from_env(GOOD))
    for name in ("PHOTON_API_KEY", "PHOTON_WEBHOOK_SECRET", "OPERATOR_API_TOKEN"):
        assert GOOD[name] not in r


def test_env_example_has_only_placeholders():  # S3
    text = (ROOT / ".env.example").read_text()
    for name in ["PHOTON_API_KEY", "PHOTON_PROJECT_ID", "PHOTON_WEBHOOK_SECRET", "OPERATOR_API_TOKEN"]:
        assert re.search(rf"^#?\s*{name}=\s*$", text, re.M), name
    assert not re.search(r"pho_sk_[A-Za-z0-9]{8,}|whsec_[A-Za-z0-9+/]{8,}|pho_prj_[0-9a-z]{8,}", text)


def test_env_file_is_gitignored():  # S3
    assert ".env" in (ROOT / ".gitignore").read_text().split()
