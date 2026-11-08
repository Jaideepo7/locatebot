"""Remote-control settings from environment variables. Validated at startup; secrets never appear
in errors or repr.

Ported from chanabyte's `locatebot/backend/config.py` (feat/photon-dispatch). Dropped there:
`JETSON_DEVICE_ID`, `JETSON_AUTH_TOKEN` (the Pi talks to the Orin over our private link, not an
authenticated WebSocket) and `BACKEND_PUBLIC_URL` (the Pi has no public URL). The webhook secret
and operator token are optional: without them the webhook route / operator API are disabled.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Mapping, Optional

from .photon import _PROJECT_RE, _decode_secret


class ConfigError(ValueError):
    pass


def _secret(default: str = "") -> str:
    return field(default=default, repr=False)


@dataclass(frozen=True)
class Settings:
    photon_project_id: str
    photon_api_key: str = _secret()
    photon_webhook_secret: str = _secret()  # "" = webhook route disabled
    operator_api_token: str = _secret()     # "" = operator API disabled
    photon_mode: str = "live"          # live | console (print replies instead of sending them)
    robot_id: str = "locatebot"
    db_path: str = "dispatch.db"
    mission_ttl_s: float = 600.0
    ack_timeout_s: float = 10.0
    link_grace_s: float = 15.0

    @property
    def webhook_enabled(self) -> bool:
        return bool(self.photon_webhook_secret)

    @property
    def operator_enabled(self) -> bool:
        return bool(self.operator_api_token)

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
        if secret:
            try:
                _decode_secret(secret)
            except ValueError:
                errors.append("PHOTON_WEBHOOK_SECRET must be the destination's whsec_... signing secret")
        operator = get("OPERATOR_API_TOKEN")
        if operator and len(operator) < 32:
            errors.append("OPERATOR_API_TOKEN must be at least 32 characters (e.g. python -c 'import secrets; "
                          "print(secrets.token_urlsafe(32))')")
        robot_id = get("ROBOT_ID", "locatebot")
        if not re.match(r"^[A-Za-z0-9_.-]{1,64}$", robot_id):
            errors.append("ROBOT_ID must be 1-64 letters, digits, '.', '_' or '-'")
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
        return cls(photon_project_id=project, photon_api_key=api_key, photon_webhook_secret=secret,
                   operator_api_token=operator, photon_mode=mode, robot_id=robot_id,
                   db_path=get("DISPATCH_DB_PATH", "dispatch.db"), mission_ttl_s=numbers["MISSION_TTL_S"],
                   ack_timeout_s=numbers["ACK_TIMEOUT_S"], link_grace_s=numbers["LINK_GRACE_S"])
