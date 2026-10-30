import cv2
import numpy as np
import pytest

from huggin_vision.camera_model import CameraMount, view_from_mount
from huggin_vision.zone import ZoneConfig, detect_zone

K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
SHAPE = (480, 640)
MOUNT = CameraMount(x=0.1, y=0.0, height=0.25, pitch_deg=30.0)


def floor_to_pixel(camera, point):
    """Project a floor point (x, y) into the image."""
    p = camera.rotation.T @ (np.array([*point, 0.0]) - camera.translation)
    return K[:2, :2] @ (p[:2] / p[2]) + K[:2, 2]


def zone_contour(camera, center, size=0.4, yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    corners = [(center[0] + c * dx - s * dy, center[1] + s * dx + c * dy)
               for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))
               for dx, dy in [(dx * size / 2, dy * size / 2)]]
    pts = np.array([floor_to_pixel(camera, p) for p in corners])
    img = np.zeros(SHAPE, np.uint8)
    cv2.fillPoly(img, [np.round(pts).astype(np.int32)], 255)
    return cv2.findContours(img, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_NONE)[0][0]


def test_pixel_to_floor_round_trip():
    camera = view_from_mount(K, MOUNT, 90.0)
    for point in [(0.6, 0.0), (0.5, 0.2), (0.8, -0.3)]:
        u, v = floor_to_pixel(camera, point)
        assert camera.pixel_to_floor_plane(u, v)[:2] == pytest.approx(
            point, abs=1e-6)


def test_pan_turns_the_view_left():
    ahead = view_from_mount(K, MOUNT, 90.0)
    left = view_from_mount(K, MOUNT, 120.0)
    center_ahead = ahead.pixel_to_floor_plane(320, 400)
    center_left = left.pixel_to_floor_plane(320, 400)
    assert center_left[1] > center_ahead[1] + 0.05


def test_rays_above_the_horizon_miss_the_floor():
    camera = view_from_mount(K, CameraMount(pitch_deg=0.0), 90.0)
    assert camera.pixel_to_floor_plane(320, 100) is None


@pytest.mark.parametrize('yaw', [0.0, 0.3])
def test_zone_position_and_size(yaw):
    camera = view_from_mount(K, MOUNT, 90.0)
    contour = zone_contour(camera, (0.7, 0.05), yaw=yaw)
    zone = detect_zone('green', contour, camera, SHAPE, ZoneConfig())
    assert zone is not None and not zone.partial
    assert zone.position[:2] == pytest.approx((0.7, 0.05), abs=0.03)
    assert len(zone.polygon) == 4


def test_zone_cut_by_the_image_border_is_partial():
    camera = view_from_mount(K, MOUNT, 90.0)
    contour = zone_contour(camera, (0.5, 0.45))
    zone = detect_zone('red', contour, camera, SHAPE, ZoneConfig())
    assert zone is not None and zone.partial


def test_wrong_sized_blob_is_not_a_zone():
    camera = view_from_mount(K, MOUNT, 90.0)
    contour = zone_contour(camera, (0.7, 0.0), size=0.15)
    assert detect_zone('blue', contour, camera, SHAPE, ZoneConfig()) is None
