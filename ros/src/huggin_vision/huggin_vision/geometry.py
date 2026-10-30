"""Rotation and cube-model helpers shared by the fit and the drawing code."""
import numpy as np

# Vertex pairs forming the 12 edges of the model from cube_vertices()
CUBE_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6),
              (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def cube_vertices(size):
    """Return the 8 corners (8x3) of an axis-aligned cube centered at 0."""
    h = size / 2
    return np.array([[x, y, z] for x in (-h, h) for y in (-h, h)
                     for z in (-h, h)], dtype=np.float64)


def rotation_to_quaternion(rotation):
    """Return (x, y, z, w) for a 3x3 rotation matrix."""
    R = rotation
    trace = np.trace(R)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def quaternion_to_rotation(q):
    """Return the 3x3 rotation matrix of a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
