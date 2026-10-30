"""Cube pose estimation from a binary color mask.

The cube center is pinned to the blob centroid's viewing ray, so only the
depth and the rotation are searched, minimizing 1 - IoU between the blob and
the projected cube silhouette.
"""
import cv2
import numpy as np
from geometry_msgs.msg import Pose

from huggin_vision.geometry import cube_vertices, rotation_to_quaternion

# Blob shape filter: cubes look roughly square
MIN_ASPECT_RATIO = 0.6  # short side / long side of the min-area rectangle
MIN_FILL_RATIO = 0.7  # blob area / min-area rectangle area

# Fit settings
MIN_DEPTH = 0.05  # m, closer candidates are rejected
FACE_ON_WEIGHT = 0.03  # prior favoring face-on, a square blob is ambiguous
DEPTH_SCAN = (0.6, 1.5, 25)  # (low, high, samples) around the initial depth
DEPTH_GUESS_SCALE = 1.2  # initial depth from blob area is a bit short
REFINE_ITERS = 80
DEPTH_STEP = 0.03  # relative to depth
ROTATION_STEP = 0.2  # rad
# Initial rotation vectors (Rodrigues) tried; the cost has local minima
ROTATION_STARTS = [(0, 0, 0), (0.6, 0, 0), (0, 0.6, 0), (0.5, 0.5, 0.4),
                   (-0.5, 0.4, 0.2), (0, 0, 0.78)]


def nelder_mead(f, x0, step, iters=60):
    """Minimal Nelder-Mead minimizer; returns (best_x, best_value)."""
    n = len(x0)
    pts = [np.array(x0, dtype=np.float64)]
    for i in range(n):
        p = pts[0].copy()
        p[i] += step[i]
        pts.append(p)
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = np.argsort(vals)
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        centroid = np.mean(pts[:-1], axis=0)
        xr = centroid + (centroid - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = centroid + 2 * (centroid - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = centroid + 0.5 * (pts[-1] - centroid)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (p - pts[0]) for p in pts[1:]]
                vals = [vals[0]] + [f(p) for p in pts[1:]]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def blob_roi_mask(contour):
    """Return (filled blob as a bool array, its (x, y) origin).

    The array covers the bounding box plus padding, so each silhouette
    render in the fit is cheap.
    """
    x, y, w, h = cv2.boundingRect(contour)
    pad = max(w, h)
    origin = (x - pad, y - pad)
    target = np.zeros((h + 2 * pad, w + 2 * pad), np.uint8)
    cv2.drawContours(target, [contour - origin], -1, 255, cv2.FILLED)
    return target > 0, origin


def silhouette_iou_cost(params, vertices, ray, camera_matrix, target, origin):
    """Cost of params = (depth, rvec): 1 - IoU of cube silhouette vs target,
    plus a small face-on prior."""
    depth, rvec = params[0], params[1:4]
    if depth <= MIN_DEPTH:
        return 1.0
    pts = cv2.projectPoints(vertices, rvec, depth * ray, camera_matrix,
                            None)[0].reshape(-1, 2)
    hull = cv2.convexHull((pts - origin).astype(np.float32))
    render = np.zeros(target.shape, np.uint8)
    cv2.fillConvexPoly(render, np.round(hull).astype(np.int32), 255)
    render = render > 0
    union = np.count_nonzero(render | target)
    if not union:
        return 1.0
    iou = np.count_nonzero(render & target) / union
    return 1.0 - iou + FACE_ON_WEIGHT * np.dot(rvec, rvec)


def fit_cube_silhouette(contour, camera_matrix, cube_size):
    """Fit a cube to a contour; returns (position, rotation matrix).

    Depth is well determined even though the orientation of a tilted cube
    is only loosely constrained.
    """
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    vertices = cube_vertices(cube_size)
    target, origin = blob_roi_mask(contour)

    m = cv2.moments(contour)
    u, v = m['m10'] / m['m00'], m['m01'] / m['m00']
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

    def cost(params):
        return silhouette_iou_cost(params, vertices, ray, camera_matrix,
                                   target, origin)

    depth_guess = (DEPTH_GUESS_SCALE * fx * cube_size
                   / np.sqrt(cv2.contourArea(contour)))
    low, high, samples = DEPTH_SCAN
    best, best_cost = None, np.inf
    for rvec in ROTATION_STARTS:
        # The cost is sharp in depth, so scan it before refining
        depth = min(np.linspace(low * depth_guess, high * depth_guess,
                                samples),
                    key=lambda d: cost(np.array([d, *rvec])))
        step = [DEPTH_STEP * depth] + [ROTATION_STEP] * 3
        x, c = nelder_mead(cost, [depth, *rvec], step, REFINE_ITERS)
        if c < best_cost:
            best, best_cost = x, c
    return best[0] * ray, cv2.Rodrigues(best[1:4])[0]


def is_cube_shaped(contour, min_area):
    """True if the blob is big enough and roughly square and filled."""
    area = cv2.contourArea(contour)
    if area < min_area:
        return False
    w, h = cv2.minAreaRect(contour)[1]
    if min(w, h) == 0:
        return False
    return (min(w, h) / max(w, h) >= MIN_ASPECT_RATIO
            and area / (w * h) >= MIN_FILL_RATIO)


def make_pose(position, rotation):
    """Build a geometry_msgs/Pose from a position and a rotation matrix."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    quat = rotation_to_quaternion(rotation)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = map(float, quat)
    return pose


def estimate_cube_poses(mask, camera_matrix, cube_size, min_area):
    """Return a list of geometry_msgs/Pose, one per cube found in a mask.

    Poses are the cube center in the camera optical frame (x right, y down,
    z forward), in meters. camera_matrix must match the mask resolution.
    The orientation is only a best fit: a silhouette says little about tilt.
    """
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [make_pose(*fit_cube_silhouette(c, camera_matrix, cube_size))
            for c in contours if is_cube_shaped(c, min_area)]
