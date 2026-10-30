import cv2
import numpy as np

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
    arrow = render_arrow(0.06, 600, 90, size=240)  # 240x240 at the corner
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


def test_empty_mask():
    mask = np.zeros(SHAPE, np.uint8)
    assert detect_blobs('red', mask, CAMERA, DetectorConfig()) == []
