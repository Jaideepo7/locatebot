import base64
import pathlib
import re

import pytest

from locatebot.backend.config import ConfigError, Settings

ROOT = pathlib.Path(__file__).resolve().parents[2]
GOOD = {
    "PHOTON_API_KEY": "pho_sk_" + "a" * 40,
    "PHOTON_PROJECT_ID": "pho_prj_01jabcdefghjkmnpqrstvwxyz1",
    "PHOTON_WEBHOOK_SECRET": "whsec_" + base64.b64encode(b"k" * 32).decode(),
    "JETSON_DEVICE_ID": "jetson-01",
    "JETSON_AUTH_TOKEN": "j" * 40,
    "OPERATOR_API_TOKEN": "o" * 40,
    "BACKEND_PUBLIC_URL": "https://robot.example.com",
}


def test_valid_env():
    s = Settings.from_env(GOOD)
    assert s.photon_mode == "live" and s.mission_ttl_s == 600


def test_console_mode_does_not_need_photon_api_key():
    env = dict(GOOD, PHOTON_MODE="console")
    del env["PHOTON_API_KEY"]
    assert Settings.from_env(env).photon_mode == "console"


@pytest.mark.parametrize("key,value", [("PHOTON_API_KEY", "sk_live_x"), ("PHOTON_PROJECT_ID", "123"),
                                       ("PHOTON_WEBHOOK_SECRET", "secret"), ("JETSON_AUTH_TOKEN", "short"),
                                       ("JETSON_DEVICE_ID", ""), ("OPERATOR_API_TOKEN", "short"),
                                       ("BACKEND_PUBLIC_URL", "http://insecure.example.com"),
                                       ("PHOTON_MODE", "carrier-pigeon"), ("MISSION_TTL_S", "-1")])
def test_bad_values_refused(key, value):
    with pytest.raises(ConfigError) as e:
        Settings.from_env(dict(GOOD, **{key: value}))
    assert key in str(e.value)


def test_errors_never_echo_secrets():  # N9
    with pytest.raises(ConfigError) as e:
        Settings.from_env(dict(GOOD, JETSON_AUTH_TOKEN="short-secret"))
    assert "short-secret" not in str(e.value)
    assert GOOD["PHOTON_API_KEY"] not in repr(Settings.from_env(GOOD))


def test_env_example_has_only_placeholders():  # S3
    text = (ROOT / ".env.example").read_text()
    for name in ["PHOTON_API_KEY", "PHOTON_PROJECT_ID", "PHOTON_WEBHOOK_SECRET", "JETSON_DEVICE_ID",
                 "JETSON_AUTH_TOKEN", "BACKEND_PUBLIC_URL"]:
        assert re.search(rf"^{name}=\s*$", text, re.M), name
    assert not re.search(r"pho_sk_[A-Za-z0-9]{8,}|whsec_[A-Za-z0-9+/]{8,}", text)


def test_env_file_is_gitignored():  # S3
    assert ".env" in (ROOT / ".gitignore").read_text().split()
