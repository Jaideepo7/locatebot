"""What the store agent needs from the robot and the item finder, plus mocks for tests/demos.

The agent only ever calls ``Robot.go(waypoint)`` with a whitelisted waypoint name. The real
robot must put every move through the LiDAR safety gate; nothing here touches motors.
"""
from __future__ import annotations

import io
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class Box:
    """Detection box as fractions of the frame (0..1), x to the right, y down."""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float = 1.0

    def __post_init__(self) -> None:
        if not (0.0 <= self.x1 < self.x2 <= 1.0 and 0.0 <= self.y1 < self.y2 <= 1.0):
            raise ValueError(f"box out of range: {self}")


def position_text(box: Box) -> str:
    """Shopper-friendly position: 'middle shelf, left side'."""
    cx, cy = (box.x1 + box.x2) / 2, (box.y1 + box.y2) / 2
    row = "top" if cy < 1 / 3 else "middle" if cy < 2 / 3 else "bottom"
    col = "left side" if cx < 1 / 3 else "center" if cx < 2 / 3 else "right side"
    return f"{row} shelf, {col}"


class Robot(Protocol):
    def go(self, waypoint: str) -> str:
        """Drive to a waypoint; blocks. Returns 'arrived', 'blocked' or 'failed'."""
        ...

    def frame(self) -> bytes | None:
        """Newest camera frame as JPEG, or None."""
        ...


class Locator(Protocol):
    def locate(self, jpeg: bytes, query: str) -> list[Box]: ...


class MockRobot:
    """No hardware: records visits, returns scripted statuses (default 'arrived')."""

    def __init__(self, frame: bytes | None = None, statuses: list[str] | None = None) -> None:
        self._frame = frame
        self.statuses = list(statuses or [])
        self.visits: list[str] = []

    def go(self, waypoint: str) -> str:
        status = self.statuses.pop(0) if self.statuses else "arrived"
        if status == "arrived":
            self.visits.append(waypoint)
        return status

    def frame(self) -> bytes | None:
        return self._frame


class FakeLocator:
    def __init__(self, boxes: list[Box]) -> None:
        self.boxes = list(boxes)
        self.queries: list[str] = []

    def locate(self, jpeg: bytes, query: str) -> list[Box]:
        self.queries.append(query)
        return list(self.boxes)


def crop_jpeg(jpeg: bytes, box: Box, pad: float = 0.08, max_side: int = 640) -> bytes | None:
    """Crop the box (plus padding) out of a JPEG. None if Pillow is not installed."""
    try:
        from PIL import Image  # optional; not a project dependency
    except ImportError:
        return None
    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
    w, h = img.size
    x1, y1 = max(0.0, box.x1 - pad), max(0.0, box.y1 - pad)
    x2, y2 = min(1.0, box.x2 + pad), min(1.0, box.y2 + pad)
    crop = img.crop((int(x1 * w), int(y1 * h), max(int(x1 * w) + 1, int(x2 * w)), max(int(y1 * h) + 1, int(y2 * h))))
    crop.thumbnail((max_side, max_side))
    out = io.BytesIO()
    crop.save(out, format="JPEG", quality=80)
    return out.getvalue()
