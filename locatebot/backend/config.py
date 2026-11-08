"""Backend settings from environment variables. Validated at startup; secrets never appear in errors or repr."""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Mapping, Optional

from .messaging.photon import _PROJECT_RE, _decode_secret


class ConfigError(ValueError):
    pass


def _secret(default: str = "") -> str:
    return field(default=default, repr=False)


@dataclass(frozen=True)
class Settings:
    photon_project_id: str
    jetson_device_id: str
    backend_public_url: str
    photon_api_key: str = _secret()
    photon_webhook_secret: str = _secret()
    jetson_auth_token: str = _secret()
    operator_api_token: str = _secret()
    photon_mode: str = "live"          # live | console (print replies instead of sending them)
    db_path: str = "dispatch.db"
    mission_ttl_s: float = 600.0
    ack_timeout_s: float = 10.0
    link_grace_s: float = 15.0

    @classmethod
    def from_env(cls, env: Optional[Mapping[str, str]] = None) -> "Settings":
        env = os.environ if env is None else env
        get = lambda k, d="": (env.get(k) or d).strip()
        errors = []

        mode = get("PHOTON_MODE", "live")
        if mode not in ("live", "console"):
            errors.append("PHOTON_MODE must be live or console")
        api_key = get("PHOTON_API_KEY")
        if mode == "live" and not re.match(r"^pho_sk_\S{8,}$", api_key):
            errors.append("PHOTON_API_KEY must be a Photon project API key (pho_sk_...)")
        project = get("PHOTON_PROJECT_ID")
        if not _PROJECT_RE.match(project):
            errors.append("PHOTON_PROJECT_ID must look like pho_prj_ followed by 26 characters")
        secret = get("PHOTON_WEBHOOK_SECRET")
        try:
            _decode_secret(secret)
        except ValueError:
            errors.append("PHOTON_WEBHOOK_SECRET must be the destination's whsec_... signing secret")
        device = get("JETSON_DEVICE_ID")
        if not re.match(r"^[A-Za-z0-9_.-]{1,64}$", device):
            errors.append("JETSON_DEVICE_ID must be 1-64 letters, digits, '.', '_' or '-'")
        for name in ("JETSON_AUTH_TOKEN", "OPERATOR_API_TOKEN"):
            if len(get(name)) < 32:
                errors.append(f"{name} must be at least 32 characters (e.g. python -c 'import secrets; "
                              f"print(secrets.token_urlsafe(32))')")
        url = get("BACKEND_PUBLIC_URL")
        if not url.startswith("https://"):
            errors.append("BACKEND_PUBLIC_URL must be an https:// URL (Photon only delivers to public HTTPS)")
        numbers = {}
        for name, default in (("MISSION_TTL_S", 600), ("ACK_TIMEOUT_S", 10), ("LINK_GRACE_S", 15)):
            try:
                numbers[name] = float(get(name, str(default)))
                if not 0 < numbers[name] < 86400:
                    raise ValueError
            except ValueError:
                errors.append(f"{name} must be a positive number of seconds")
        if errors:
            raise ConfigError("Invalid configuration:\n  " + "\n  ".join(errors))
        return cls(photon_project_id=project, jetson_device_id=device, backend_public_url=url,
                   photon_api_key=api_key, photon_webhook_secret=secret, jetson_auth_token=get("JETSON_AUTH_TOKEN"),
                   operator_api_token=get("OPERATOR_API_TOKEN"), photon_mode=mode,
                   db_path=get("DISPATCH_DB_PATH", "dispatch.db"), mission_ttl_s=numbers["MISSION_TTL_S"],
                   ack_timeout_s=numbers["ACK_TIMEOUT_S"], link_grace_s=numbers["LINK_GRACE_S"])
