from types import SimpleNamespace

import cv2
import numpy as np

from huggin_vision.geometry import (
    cube_vertices, quaternion_to_rotation, rotation_to_quaternion)


def test_quaternion_round_trip():
    for rvec in ([0, 0, 0], [0.3, -0.2, 0.5], [3.0, 0.1, 0.0], [0, 3.1, 0]):
        R = cv2.Rodrigues(np.array(rvec, dtype=np.float64))[0]
        x, y, z, w = rotation_to_quaternion(R)
        back = quaternion_to_rotation(SimpleNamespace(x=x, y=y, z=z, w=w))
        assert np.allclose(R, back, atol=1e-9)


def test_cube_vertices():
    v = cube_vertices(0.04)
    assert v.shape == (8, 3)
    assert np.allclose(np.abs(v), 0.02)
    assert len({tuple(p) for p in v}) == 8
