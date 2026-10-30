import cv2
import numpy as np
import pytest

from huggin_vision.arrow import (
    BODY_LENGTHS, BODY_WIDTH, HEAD_WIDTH, TIP_LENGTH, angle_to_dir,
    distance_from_head, length_class, measure_arrow)


def render_arrow(body, scale=1000, angle_deg=0, size=400):
    """Return the contour of an arrow pointing at angle_deg (0 = right)."""
    hw, bw, tip = HEAD_WIDTH / 2, BODY_WIDTH / 2, TIP_LENGTH
    poly = np.array([[0, -bw], [body, -bw], [body, -hw], [body + tip, 0],
                     [body, hw], [body, bw], [0, bw]]) * scale
    a = np.radians(angle_deg)
    rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    poly = poly @ rot.T + size / 2  # image y points down: +angle is ccw
    img = np.zeros((size, size), np.uint8)
    cv2.fillPoly(img, [np.round(poly).astype(np.int32)], 255)
    return cv2.findContours(img, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_NONE)[0][0]


@pytest.mark.parametrize('name', list(BODY_LENGTHS))
@pytest.mark.parametrize('angle', [0, 37, 90, 180, 250])
def test_length_class_and_direction(name, angle):
    measure = measure_arrow(render_arrow(BODY_LENGTHS[name], 1000, angle))
    assert measure is not None
    assert length_class(measure)[0] == name
    error = (measure.angle_deg - angle + 180) % 360 - 180
    assert abs(error) < 5


@pytest.mark.parametrize('name', list(BODY_LENGTHS))
def test_small_arrows_still_classified(name):
    # about 8 px of head width, like a far away arrow
    measure = measure_arrow(render_arrow(BODY_LENGTHS[name], 200, 20), 20)
    assert measure is not None
    assert length_class(measure)[0] == name


def test_distance_from_head_width():
    measure = measure_arrow(render_arrow(0.06, 1000, 0))
    focal = 500.0
    assert distance_from_head(measure, focal) == pytest.approx(
        focal * HEAD_WIDTH / (HEAD_WIDTH * 1000), rel=0.03)


def test_rejects_squares_and_small_blobs():
    square = np.zeros((100, 100), np.uint8)
    square[20:80, 20:80] = 255
    contour = cv2.findContours(square, cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_NONE)[0][0]
    assert measure_arrow(contour) is None
    assert measure_arrow(render_arrow(0.06, 1000), min_area=10 ** 9) is None


def test_angle_to_dir():
    assert angle_to_dir(5) == 'right'
    assert angle_to_dir(355) == 'right'
    assert angle_to_dir(185) == 'left'
    assert angle_to_dir(90) == 'up'
    assert angle_to_dir(45) == 'UNK'
