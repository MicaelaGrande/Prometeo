import cv2
import numpy as np

from huggin_vision.cube_fit import estimate_cube_poses
from huggin_vision.geometry import cube_vertices

CAMERA = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
SIZE = 0.02


def render_cube_mask(position):
    pts = cv2.projectPoints(cube_vertices(SIZE), np.zeros(3),
                            np.array(position), CAMERA, None)[0]
    hull = cv2.convexHull(pts.reshape(-1, 2).astype(np.float32))
    mask = np.zeros((480, 640), np.uint8)
    cv2.fillConvexPoly(mask, np.round(hull).astype(np.int32), 255)
    return mask


def test_estimates_position_of_face_on_cube():
    truth = (0.05, -0.03, 0.5)
    poses = estimate_cube_poses(render_cube_mask(truth), CAMERA, SIZE, 100)
    assert len(poses) == 1
    p = poses[0].position
    assert np.allclose([p.x, p.y, p.z], truth, atol=0.03)


def test_ignores_small_and_elongated_blobs():
    small = np.zeros((480, 640), np.uint8)
    small[100:105, 100:105] = 255
    bar = np.zeros((480, 640), np.uint8)
    bar[100:110, 100:300] = 255
    assert estimate_cube_poses(small, CAMERA, SIZE, 100) == []
    assert estimate_cube_poses(bar, CAMERA, SIZE, 100) == []
