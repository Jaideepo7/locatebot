"""Clean up LocateAnything detections and turn boxes into bearings.

Boxes are pixel coordinates ``(x1, y1, x2, y2)`` in the input image, as returned by
locate-anything.cpp's C API (``{"detections":[{"label":..,"box":[x1,y1,x2,y2]}]}``).
The engine gives no confidence score, so filtering is purely geometric.

Bearings follow the robot frame: radians, counter-clockwise positive, so an item left
of the image centre has a positive bearing (+y is left).
"""
from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass

Box = tuple[float, float, float, float]

PROMPT_TEMPLATE = "Locate all the instances that matches the following description: {}."
MULTI_ITEM_SEPARATOR = "</c>"


@dataclass(frozen=True)
class Detection:
    label: str
    box: Box  # pixels: x1, y1, x2, y2


@dataclass(frozen=True)
class Camera:
    """Pinhole intrinsics needed for a horizontal bearing."""

    cx: float  # principal point x, pixels
    fx: float  # focal length x, pixels

    @classmethod
    def from_hfov(cls, width_px: int, hfov_rad: float) -> "Camera":
        return cls(cx=width_px / 2.0, fx=(width_px / 2.0) / math.tan(hfov_rad / 2.0))


def build_prompt(item: str) -> str:
    """NVIDIA's prompt template for one item. Multi-item prompts are rejected:
    in testing only the first item of ``a</c>b`` was ever found."""
    item = item.strip()
    if not item:
        raise ValueError("empty item")
    if MULTI_ITEM_SEPARATOR in item:
        raise ValueError("one item per prompt; multi-item prompts only find the first item")
    return PROMPT_TEMPLATE.format(item)


def parse_detections(raw: str) -> list[Detection]:
    out = []
    for d in json.loads(raw)["detections"]:
        box = d["box"]
        if len(box) != 4:
            raise ValueError(f"box must have 4 numbers, got {box!r}")
        out.append(Detection(str(d["label"]), tuple(float(v) for v in box)))
    return out


def _area(b: Box) -> float:
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def iou(a: Box, b: Box) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = _area(a) + _area(b) - inter
    return inter / union if union > 0 else 0.0


def merge_duplicates(dets: list[Detection], iou_threshold: float = 0.7) -> list[Detection]:
    """Drop later same-label boxes that overlap an earlier kept one by more than the threshold."""
    kept: list[Detection] = []
    for d in dets:
        if not any(k.label == d.label and iou(k.box, d.box) > iou_threshold for k in kept):
            kept.append(d)
    return kept


def drop_oversized(dets: list[Detection], width: int, height: int,
                   max_fraction: float = 0.8) -> list[Detection]:
    """Remove boxes covering more than ``max_fraction`` of the image (seen when the item is absent)."""
    limit = max_fraction * width * height
    return [d for d in dets if _area(d.box) <= limit]


def clean_detections(dets: list[Detection], width: int, height: int) -> list[Detection]:
    return drop_oversized(merge_duplicates(dets), width, height)


def bearing_rad(box: Box, cam: Camera) -> float:
    cx = (box[0] + box[2]) / 2.0
    return -math.atan((cx - cam.cx) / cam.fx)


_SOF_MARKERS = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


def jpeg_size(data: bytes) -> tuple[int, int]:
    """(width, height) from a JPEG's SOF header, without decoding the image."""
    if data[:2] != b"\xff\xd8":
        raise ValueError("not a JPEG")
    i = 2
    while i + 4 <= len(data):
        if data[i] != 0xFF:
            raise ValueError("corrupt JPEG marker stream")
        marker = data[i + 1]
        if marker == 0xFF:  # fill byte
            i += 1
            continue
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        (seg_len,) = struct.unpack(">H", data[i + 2:i + 4])
        if marker in _SOF_MARKERS:
            height, width = struct.unpack(">HH", data[i + 5:i + 9])
            return width, height
        i += 2 + seg_len
    raise ValueError("no SOF segment in JPEG")
