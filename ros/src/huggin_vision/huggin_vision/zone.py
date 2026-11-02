"""Colored floor zones, located by projecting their outline onto the floor."""
from dataclasses import dataclass

import cv2
import numpy as np

from huggin_vision.detections import ZONE, Detection

BORDER_PX = 2  # blobs this close to the image edge are cut by it
HULL_EPSILON = 0.02  # polygon simplification, fraction of the perimeter


@dataclass
class ZoneConfig:
    """What a zone looks like."""

    size: float = 0.40  # m, side of the square
    tolerance: float = 0.25  # allowed relative error on the measured sides


def touches_border(contour, image_shape):
    """True if the blob reaches the edge of the image."""
    x, y, w, h = cv2.boundingRect(contour)
    height, width = image_shape[:2]
    return (x <= BORDER_PX or y <= BORDER_PX
            or x + w >= width - BORDER_PX or y + h >= height - BORDER_PX)


def floor_polygon(contour, camera):
    """Project the simplified convex outline of a blob onto the floor.

    Returns an Nx2 array (x, y), or None if a point does not reach the floor
    (it is at or above the horizon).
    """
    hull = cv2.convexHull(contour)
    poly = cv2.approxPolyDP(hull, HULL_EPSILON * cv2.arcLength(hull, True),
                            True).reshape(-1, 2)
    points = [camera.pixel_to_floor_plane(u, v) for u, v in poly]
    if any(p is None for p in points):
        return None
    return np.array([p[:2] for p in points])


def detect_zone(color, contour, camera, image_shape, config):
    """Return the Detection of a zone blob, or None if it is not one.

    A fully visible zone must measure size x size on the floor. A zone cut by
    the image border only has to be no bigger than that.
    """
    ground = floor_polygon(contour, camera)
    if ground is None or len(ground) < 3:
        return None
    partial = touches_border(contour, image_shape)
    low = config.size * (1 - config.tolerance)
    high = config.size * (1 + config.tolerance)

    (cx, cy), (w, h), angle = cv2.minAreaRect(ground.astype(np.float32))
    if max(w, h) > high or (not partial and min(w, h) < low):
        return None

    if partial:
        center = ground.mean(axis=0)
        corners = ground
        yaw, confidence = 0.0, 0.5
    else:
        center = np.array([cx, cy])
        corners = cv2.boxPoints(((cx, cy), (w, h), angle))
        yaw = np.radians((angle + 45) % 90 - 45)
        error = max(abs(w - config.size), abs(h - config.size)) / config.size
        confidence = float(max(0.0, 1.0 - error / config.tolerance))

    return Detection(
        kind=ZONE, color=color, position=np.array([*center, 0.0]),
        angle=float(yaw), polygon=[(float(x), float(y), 0.0)
                                   for x, y in corners],
        confidence=confidence, partial=partial, contour=contour)
