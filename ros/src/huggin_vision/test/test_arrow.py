import cv2
import numpy as np
import pytest

from huggin_vision.arrow import (
    BODY_WIDTH, HEAD_WIDTH, SHAPES, angle_to_dir, class_from_length,
    distance_from_head, measure_arrow)


def render_arrow(name, scale=3000, angle_deg=0, size=400, grow=0):
    """Return the contour of a real arrow pointing at angle_deg (0 = right).

    scale is pixels per meter. grow moves the outline out that many pixels,
    like a color threshold that makes the blob a little thicker.
    """
    body, head = SHAPES[name]
    hw, bw = HEAD_WIDTH / 2, BODY_WIDTH / 2
    poly = np.array([[0, -bw], [body, -bw], [body, -hw], [body + head, 0],
                     [body, hw], [body, bw], [0, bw]]) * scale
    poly -= [(body + head) * scale / 2, 0]  # centered in the image
    a = np.radians(angle_deg)
    rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    poly = poly @ rot.T + size / 2  # image y points down: +angle is ccw
    img = np.zeros((size, size), np.uint8)
    cv2.fillPoly(img, [np.round(poly * 8).astype(np.int32)], 255, shift=3)
    if grow:
        img = cv2.dilate(img, np.ones((3, 3), np.uint8), iterations=grow)
    return cv2.findContours(img, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_NONE)[0][0]


@pytest.mark.parametrize('name', list(SHAPES))
@pytest.mark.parametrize('angle', [0, 30, -30, 90, 180, 250])
def test_class_and_direction_from_shape(name, angle):
    # Turned up to 30 degrees, as in the competition
    measure = measure_arrow(render_arrow(name, 3000, angle))
    assert measure is not None
    assert measure.length_class == name
    error = (measure.angle_deg - angle + 180) % 360 - 180
    assert abs(error) < 5


@pytest.mark.parametrize('name', list(SHAPES))
def test_small_arrows_still_classified(name):
    # about 28 px of head width, like an arrow seen from further away
    measure = measure_arrow(render_arrow(name, 1000, 20))
    assert measure is not None
    assert measure.length_class == name


@pytest.mark.parametrize('name', list(SHAPES))
def test_thicker_blob_keeps_its_class(name):
    # A pixel more all around (light, threshold) must not change the class
    measure = measure_arrow(render_arrow(name, 1500, 0, grow=1))
    assert measure is not None
    assert measure.length_class == name


@pytest.mark.parametrize('name', list(SHAPES))
@pytest.mark.parametrize('depth_error', [0.8, 1.0, 1.2])
def test_class_from_real_length(name, depth_error):
    # 2000 px per meter: a camera with a 700 px focal at 0.35 m. The wall
    # distance may be 20 % off and the class must not change
    measure = measure_arrow(render_arrow(name, 2000, 10))
    focal, depth = 700.0, 0.35
    found, confidence = class_from_length(
        measure, depth * depth_error, focal)
    assert found == name
    if depth_error == 1.0:
        assert confidence > 0.7


def test_distance_from_head_width():
    measure = measure_arrow(render_arrow('normal', 3000, 0))
    focal = 500.0
    assert distance_from_head(measure, focal) == pytest.approx(
        focal / 3000, rel=0.05)


def test_rejects_squares_and_small_blobs():
    square = np.zeros((100, 100), np.uint8)
    square[20:80, 20:80] = 255
    contour = cv2.findContours(square, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_NONE)[0][0]
    assert measure_arrow(contour) is None
    assert measure_arrow(render_arrow('normal'), min_area=10 ** 9) is None


def test_angle_to_dir():
    assert angle_to_dir(5) == 'right'
    assert angle_to_dir(330) == 'right'  # turned 30 degrees
    assert angle_to_dir(185) == 'left'
    assert angle_to_dir(120) == 'up'
    assert angle_to_dir(45) == 'UNK'
