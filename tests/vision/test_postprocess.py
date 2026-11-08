"""Tests for locatebot.vision.postprocess (pure Python, no model needed)."""
from __future__ import annotations

import math
import struct

import pytest

from locatebot.vision.postprocess import (
    Camera,
    Detection,
    bearing_rad,
    build_prompt,
    clean_detections,
    drop_oversized,
    iou,
    jpeg_size,
    merge_duplicates,
    parse_detections,
)


def det(label, x1, y1, x2, y2):
    return Detection(label, (float(x1), float(y1), float(x2), float(y2)))


# --- parsing the engine's JSON ---------------------------------------------

def test_parse_detections_reads_engine_json():
    raw = '{"detections":[{"label":"cat","box":[1.5,2.0,30.25,40.0]},{"label":"remote","box":[5,6,7,8]}]}'
    assert parse_detections(raw) == [det("cat", 1.5, 2, 30.25, 40), det("remote", 5, 6, 7, 8)]


def test_parse_detections_empty_list():
    assert parse_detections('{"detections":[]}') == []


def test_parse_detections_rejects_malformed_box():
    with pytest.raises(ValueError):
        parse_detections('{"detections":[{"label":"x","box":[1,2,3]}]}')


# --- prompt ------------------------------------------------------------------

def test_build_prompt_uses_nvidia_template():
    assert build_prompt("  tape measure ") == (
        "Locate all the instances that matches the following description: tape measure."
    )


@pytest.mark.parametrize("bad", ["", "   ", "cat</c>dog"])
def test_build_prompt_rejects_empty_or_multi_item(bad):
    # Multi-item prompts only found the first item in testing: one item per call.
    with pytest.raises(ValueError):
        build_prompt(bad)


# --- IoU and duplicate merging ---------------------------------------------

def test_iou_identical_disjoint_and_half():
    a = (0.0, 0.0, 10.0, 10.0)
    assert iou(a, a) == pytest.approx(1.0)
    assert iou(a, (20.0, 20.0, 30.0, 30.0)) == 0.0
    # overlap 50 of union 150
    assert iou(a, (5.0, 0.0, 15.0, 10.0)) == pytest.approx(50 / 150)


def test_iou_degenerate_box_is_zero():
    assert iou((5.0, 5.0, 5.0, 5.0), (5.0, 5.0, 5.0, 5.0)) == 0.0


def test_merge_duplicates_keeps_first_of_near_identical_same_label():
    dets = [det("cat", 0, 0, 100, 100), det("cat", 2, 2, 101, 101)]
    assert merge_duplicates(dets) == [dets[0]]


def test_merge_duplicates_keeps_overlapping_boxes_of_different_labels():
    dets = [det("cat", 0, 0, 100, 100), det("dog", 0, 0, 100, 100)]
    assert merge_duplicates(dets) == dets


def test_merge_duplicates_keeps_two_separate_instances():
    dets = [det("remote", 0, 0, 10, 10), det("remote", 50, 50, 60, 60)]
    assert merge_duplicates(dets) == dets


# --- whole-image boxes -------------------------------------------------------

def test_drop_oversized_removes_box_covering_most_of_frame():
    dets = [det("bolt", 0, 0, 640, 480), det("bolt", 100, 100, 200, 200)]
    assert drop_oversized(dets, 640, 480) == [dets[1]]


def test_drop_oversized_keeps_box_just_under_limit():
    d = det("board", 0, 0, 640, 0.79 * 480)
    assert drop_oversized([d], 640, 480) == [d]


def test_clean_detections_merges_then_drops():
    dets = [
        det("cat", 0, 0, 100, 100),
        det("cat", 1, 1, 100, 100),
        det("cat", 0, 0, 640, 480),
    ]
    assert clean_detections(dets, 640, 480) == [dets[0]]


# --- bearing -----------------------------------------------------------------

def test_camera_from_hfov():
    cam = Camera.from_hfov(640, math.radians(90))
    assert cam.cx == pytest.approx(320)
    assert cam.fx == pytest.approx(320)  # tan(45 deg) = 1


def test_bearing_centre_is_zero_left_positive_right_negative():
    cam = Camera.from_hfov(640, math.radians(90))
    assert bearing_rad((310, 0, 330, 10), cam) == pytest.approx(0.0)
    # box centred at the left edge -> +45 deg (robot frame: +y left, CCW positive)
    assert bearing_rad((-10, 0, 10, 10), cam) == pytest.approx(math.radians(45))
    assert bearing_rad((630, 0, 650, 10), cam) == pytest.approx(math.radians(-45))


# --- JPEG size without PIL ---------------------------------------------------

def _fake_jpeg(width, height, sof=0xC0):
    app0 = b"\xff\xe0" + struct.pack(">H", 16) + b"JFIF\x00" + b"\x00" * 9
    sof_seg = b"\xff" + bytes([sof]) + struct.pack(">HBHHB", 11, 8, height, width, 1) + b"\x01\x11\x00"
    return b"\xff\xd8" + app0 + sof_seg + b"\xff\xd9"


def test_jpeg_size_baseline_and_progressive():
    assert jpeg_size(_fake_jpeg(1600, 900)) == (1600, 900)
    assert jpeg_size(_fake_jpeg(900, 1600, sof=0xC2)) == (900, 1600)


def test_jpeg_size_rejects_non_jpeg():
    with pytest.raises(ValueError):
        jpeg_size(b"\x89PNG\r\n\x1a\n")


# --- review fixes ------------------------------------------------------------

def test_parse_detections_rejects_non_finite_box():
    with pytest.raises(ValueError):
        parse_detections('{"detections":[{"label":"x","box":[NaN,0,1,1]}]}')


def test_jpeg_size_truncated_sof_is_value_error():
    with pytest.raises(ValueError):
        jpeg_size(b"\xff\xd8\xff\xc0\x00\x11\x08\x01")


def test_jpeg_size_rejects_zero_dimension():
    with pytest.raises(ValueError):
        jpeg_size(_fake_jpeg(0, 480))


def test_camera_rejects_bad_fov_and_width():
    with pytest.raises(ValueError):
        Camera.from_hfov(640, 0.0)
    with pytest.raises(ValueError):
        Camera.from_hfov(0, 1.0)
