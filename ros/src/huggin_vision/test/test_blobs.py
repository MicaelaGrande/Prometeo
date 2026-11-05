import cv2
import numpy as np
import pytest

from huggin_vision.blobs import DetectorConfig, detect_blobs
from huggin_vision.camera_model import view_from_mount
from huggin_vision.geometry import cube_vertices
from test_arrow import render_arrow
from test_camera_zone import SHAPE, MOUNT, K, zone_contour

CAMERA = view_from_mount(K, MOUNT, 90.0)


def fill(mask, contour):
    cv2.fillPoly(mask, [contour], 255)
    return mask


def cube_contour(position):
    pts = cv2.projectPoints(cube_vertices(0.02), np.zeros(3),
                            np.array(position), K, None)[0]
    hull = cv2.convexHull(pts.reshape(-1, 2).astype(np.float32))
    return np.round(hull).astype(np.int32)


def kinds(detections):
    return sorted(d.kind for d in detections)


def test_one_detection_per_object():
    mask = np.zeros(SHAPE, np.uint8)
    fill(mask, zone_contour(CAMERA, (0.8, 0.0)))
    arrow = render_arrow('normal', 1500, 90, size=240)  # 240x240 at the corner
    fill(mask, arrow + [10, 10])
    ray = CAMERA.rotation.T @ np.array([0.45, -0.1, 0.01]) \
        - CAMERA.rotation.T @ CAMERA.translation
    fill(mask, cube_contour(ray))
    found = detect_blobs('green', mask, CAMERA, DetectorConfig())
    assert kinds(found) == ['arrow', 'cube', 'zone']
    arrow_found = [d for d in found if d.kind == 'arrow'][0]
    assert arrow_found.direction == 'up'
    assert arrow_found.length_class == 'normal'
    cube_found = [d for d in found if d.kind == 'cube'][0]
    assert abs(cube_found.position[0] - 0.45) < 0.05
    assert abs(cube_found.position[1] + 0.1) < 0.05


def test_nearby_arrow_is_not_a_zone():
    # Seen from nearby the arrow is bigger than min_area_zone, and still an arrow
    mask = fill(np.zeros(SHAPE, np.uint8),
                render_arrow('normal', 3500, 90) + [10, 10])
    config = DetectorConfig()
    assert cv2.countNonZero(mask) >= config.min_area_zone
    found = detect_blobs('green', mask, CAMERA, config)
    assert kinds(found) == ['arrow']
    assert found[0].length_class == 'normal'


def test_arrow_cut_by_the_edge_is_not_reported():
    mask = fill(np.zeros(SHAPE, np.uint8),
                render_arrow('far', 2000, 0) - [150, 0])  # tail out of view
    found = detect_blobs('red', mask, CAMERA, DetectorConfig())
    assert 'arrow' not in kinds(found)


@pytest.mark.parametrize('name', ['close', 'normal', 'far'])
def test_wall_distance_gives_class_and_position(name):
    # The camera (focal 500 px) sees the arrow on a wall 0.4 m away
    depth = 0.4
    mask = fill(np.zeros(SHAPE, np.uint8),
                render_arrow(name, K[0, 0] / depth, 0) + [120, 40])
    found = detect_blobs('blue', mask, CAMERA,
                         DetectorConfig(wall_depth=depth))
    assert kinds(found) == ['arrow']
    assert found[0].length_class == name
    # In the camera frame the arrow is at the wall distance
    in_camera = CAMERA.rotation.T @ (np.array(found[0].position)
                                     - CAMERA.translation)
    assert in_camera[2] == pytest.approx(depth, rel=0.01)


@pytest.mark.parametrize('distance', [0.10, 0.15])
def test_nearby_cube_is_not_lost(distance):
    # Seen from nearby the cube is bigger than min_area_zone, and still a cube
    mask = fill(np.zeros(SHAPE, np.uint8), cube_contour([0.0, 0.01, distance]))
    config = DetectorConfig()
    assert cv2.countNonZero(mask) >= config.min_area_zone
    found = detect_blobs('red', mask, CAMERA, config)
    assert kinds(found) == ['cube']
    assert abs(found[0].camera_pose.position.z - distance) < 0.02


def test_empty_mask():
    mask = np.zeros(SHAPE, np.uint8)
    assert detect_blobs('red', mask, CAMERA, DetectorConfig()) == []
