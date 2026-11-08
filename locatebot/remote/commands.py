"""Customer command parsing. Provider-independent: input is the plain message text.

The result is data, never something executed. A description only ever becomes the
`target_description` string of a FIND_OBJECT mission.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .protocol import InvalidDescription, normalize_description


class Kind(str, Enum):
    START = "start"
    HELP = "help"
    FIND = "find"
    STATUS = "status"
    CANCEL = "cancel"
    UNKNOWN = "unknown"          # starts with "/" but is not one of ours
    NOT_COMMAND = "not_command"  # plain text
    NOT_TEXT = "not_text"        # photo, voice, sticker...


@dataclass(frozen=True)
class Command:
    kind: Kind
    description: Optional[str] = None  # set only for a valid /find
    error: Optional[str] = None        # customer-facing reason a /find was refused


_SIMPLE = {"start": Kind.START, "help": Kind.HELP, "status": Kind.STATUS, "cancel": Kind.CANCEL}


def parse_command(text: Optional[str]) -> Command:
    if text is None:
        return Command(Kind.NOT_TEXT)
    stripped = text.strip()
    if not stripped.startswith("/"):
        return Command(Kind.NOT_COMMAND)
    head, _, rest = stripped.partition(" ")
    if "\n" in head:  # "/find\nred cup"
        head, _, more = head.partition("\n")
        rest = more + " " + rest
    name = head[1:].split("@", 1)[0].lower()  # "/find@MyRobotBot" in group chats
    if name in _SIMPLE:
        return Command(_SIMPLE[name])
    if name != "find":
        return Command(Kind.UNKNOWN)
    try:
        return Command(Kind.FIND, description=normalize_description(rest))
    except InvalidDescription as e:
        return Command(Kind.FIND, error=str(e))
