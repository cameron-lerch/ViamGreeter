'''
run all of the tests in this directory with:
from the main ViamGreeter directory, run:
pytest tests/*

--
Tests for the pinhole camera mapping in detect_person.pixel_to_direction:
pixel position -> (bearing_deg, elevation_deg). See README section 5.2.
'''

import math

import pytest

from detect_person import pixel_to_direction

W, H = 1920, 1080   # webcam resolution
HFOV = 70.0         # webcam horizontal field of view in degrees (typical for most webcams)


def test_image_center_is_straight_ahead():
    bearing, elevation = pixel_to_direction(W / 2, H / 2, W, H, hfov_deg=HFOV)
    assert bearing == pytest.approx(0.0)
    assert elevation == pytest.approx(0.0)


def test_image_edges_are_half_the_field_of_view():
    left, _ = pixel_to_direction(0, H / 2, W, H, hfov_deg=HFOV)
    right, _ = pixel_to_direction(W, H / 2, W, H, hfov_deg=HFOV)
    assert left == pytest.approx(+HFOV / 2)    # camera's left is positive (counter-clockwise)
    assert right == pytest.approx(-HFOV / 2)


def test_known_value_three_quarters_across():
    # u = 3W/4 is W/4 right of center: bearing = -atan((W/4) / f), f = (W/2) / tan(HFOV/2)
    expected = -math.degrees(math.atan(0.5 * math.tan(math.radians(HFOV / 2))))
    bearing, _ = pixel_to_direction(3 * W / 4, H / 2, W, H, hfov_deg=HFOV)
    assert bearing == pytest.approx(expected)
    assert bearing == pytest.approx(-19.29, abs=0.01)


def test_bearing_is_symmetric_about_the_center():
    for offset in (50, 300, 700):
        left, _ = pixel_to_direction(W / 2 - offset, H / 2, W, H, hfov_deg=HFOV)
        right, _ = pixel_to_direction(W / 2 + offset, H / 2, W, H, hfov_deg=HFOV)
        assert left == pytest.approx(-right)


def test_same_image_fraction_gives_same_bearing_at_any_resolution():
    a, _ = pixel_to_direction(0.8 * 640, 240, 640, 480, hfov_deg=HFOV)
    b, _ = pixel_to_direction(0.8 * 1920, 540, 1920, 1080, hfov_deg=HFOV)
    assert a == pytest.approx(b)


def test_mirrored_camera_flips_the_bearing():
    normal, _ = pixel_to_direction(1500, H / 2, W, H, hfov_deg=HFOV, mirrored=False)
    mirrored, _ = pixel_to_direction(1500, H / 2, W, H, hfov_deg=HFOV, mirrored=True)
    assert mirrored == pytest.approx(-normal)


def test_top_of_image_is_above_the_horizon():
    _, top = pixel_to_direction(W / 2, 0, W, H, hfov_deg=HFOV)
    _, bottom = pixel_to_direction(W / 2, H, W, H, hfov_deg=HFOV)
    assert top > 0 > bottom


# --- PersonDetector.detect(): which detections count as people to greet -----------

import asyncio
from types import SimpleNamespace

from detect_person import MIN_SIZE, PersonDetector


def box(label, confidence, height_frac, x_center=0.5):
    ''' A fake detection: a box of the given height (as a fraction of a 1920x1080 image). '''
    h = height_frac * H
    return SimpleNamespace(class_name=label, confidence=confidence,
                           x_min=x_center * W - 50, x_max=x_center * W + 50,
                           y_min=(H - h) / 2, y_max=(H + h) / 2)


def detect_with(detections):
    ''' Run PersonDetector.detect() against a fake vision service returning these detections. '''
    async def capture_all_from_camera(*args, **kwargs):
        return SimpleNamespace(image=SimpleNamespace(width=W, height=H), detections=detections)
    detector = PersonDetector.__new__(PersonDetector)   # skip __init__: no robot connection needed
    detector.vision = SimpleNamespace(capture_all_from_camera=capture_all_from_camera)
    detector.camera_name, detector.min_confidence, detector.min_size = "cam", 0.5, MIN_SIZE
    return asyncio.run(detector.detect())


def test_far_away_people_are_ignored():
    # Measured live: the person at the booth filled ~85% of the image height,
    # background passersby 12-17% (and flickered in and out).
    people = detect_with([box("Person", 0.88, 0.85), box("Person", 0.70, 0.15, x_center=0.8)])
    assert len(people) == 1
    assert people[0].size == pytest.approx(0.85)


def test_low_confidence_and_non_person_detections_are_ignored():
    people = detect_with([box("Person", 0.40, 0.8), box("Suitcase", 0.95, 0.8)])
    assert people == []


def test_people_are_returned_closest_first():
    people = detect_with([box("Person", 0.9, 0.4, x_center=0.2), box("person", 0.9, 0.7, x_center=0.7)])
    assert [round(p.size, 2) for p in people] == [0.7, 0.4]
