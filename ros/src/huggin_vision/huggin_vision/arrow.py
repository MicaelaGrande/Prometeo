"""Arrow detection on a wall, from its outline in the image.

The arrow is a block arrow: a 30 mm wide body and a 70 mm wide, 50 mm long
triangular head. Its body is 30 (close), 60 (normal) or 100 mm (far) long.
Only the image outline is used, so the length class comes from the ratio of
the length to the head width, which does not depend on the distance.
"""
from dataclasses import dataclass

import cv2
import numpy as np

HEAD_WIDTH = 0.070  # m
BODY_WIDTH = 0.030  # m
TIP_LENGTH = 0.050  # m
BODY_LENGTHS = {'close': 0.030, 'normal': 0.060, 'far': 0.100}  # m

# Notches where the body meets the head: depth relative to sqrt(area)
MIN_NOTCH_DEPTH = 0.12
MIN_NOTCH_SYMMETRY = 0.5  # shallower notch / deeper notch
# body width / head width is 30 / 70 = 0.43
BODY_HEAD_RATIO = (0.25, 0.65)
TAPER_RANGE = (0.75, 0.95)  # part of each half used to tell head from tail
MAX_TIP_WIDTH = 0.6  # head width near the tip / head width

DIRECTIONS = {'right': 0, 'up': 90, 'left': 180, 'down': 270}


@dataclass
class ArrowMeasure:
    """Arrow outline measured in the image (pixels, y down)."""

    tail: np.ndarray
    tip: np.ndarray
    length: float
    head_width: float

    @property
    def center(self):
        return (self.tail + self.tip) / 2

    @property
    def angle_deg(self):
        """Direction of the arrow, counter-clockwise from the image right."""
        dx, dy = self.tip - self.tail
        return float(np.degrees(np.arctan2(-dy, dx)) % 360)


def _notches(contour):
    """Return the two deepest convexity defects as (point_a, point_b, depth).

    None if there are not two. Depth is in pixels.
    """
    hull = cv2.convexHull(contour, returnPoints=False)
    if hull is None or len(hull) < 4:
        return None
    try:
        defects = cv2.convexityDefects(contour, hull)
    except cv2.error:
        return None
    if defects is None or len(defects) < 2:
        return None
    deepest = sorted(defects[:, 0], key=lambda d: -d[3])[:2]
    a, b = (contour[d[2], 0].astype(np.float64) for d in deepest)
    return a, b, deepest[0][3] / 256.0, deepest[1][3] / 256.0


def _half_width(along, across, sign):
    """Width of one half of the arrow near its far end (0 if too few points).
    """
    half = along * sign
    extent = half.max()
    if extent <= 0:
        return 0.0, 0.0
    near_end = (half >= TAPER_RANGE[0] * extent) & \
        (half <= TAPER_RANGE[1] * extent)
    if np.count_nonzero(near_end) < 2:
        return extent, 0.0
    return extent, float(np.ptp(across[near_end]))


def measure_arrow(contour, min_area=100):
    """Return an ArrowMeasure for an arrow-shaped contour, else None.

    The two notches between body and head give the line the arrow is
    symmetric about. The half that tapers to a point is the head.
    """
    area = cv2.contourArea(contour)
    if area < min_area:
        return None
    notches = _notches(contour)
    if notches is None:
        return None
    a, b, depth_1, depth_2 = notches
    if depth_1 < MIN_NOTCH_DEPTH * np.sqrt(area) \
            or depth_2 < MIN_NOTCH_SYMMETRY * depth_1:
        return None

    across_axis = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    axis = np.array([-across_axis[1], across_axis[0]])
    origin = (a + b) / 2
    pts = contour.reshape(-1, 2).astype(np.float64) - origin
    along, across = pts @ axis, pts @ across_axis

    head_width = float(np.ptp(across))
    if not BODY_HEAD_RATIO[0] <= np.linalg.norm(b - a) / head_width \
            <= BODY_HEAD_RATIO[1]:
        return None

    extent_pos, width_pos = _half_width(along, across, 1)
    extent_neg, width_neg = _half_width(along, across, -1)
    sign = 1 if width_pos < width_neg else -1  # the head tapers
    head_end, tail_end = ((extent_pos, extent_neg) if sign > 0
                          else (extent_neg, extent_pos))
    if min(width_pos, width_neg) > MAX_TIP_WIDTH * head_width:
        return None

    return ArrowMeasure(
        tail=origin - sign * tail_end * axis,
        tip=origin + sign * head_end * axis,
        length=head_end + tail_end,
        head_width=head_width)


def length_class(measure):
    """Return (name, confidence) of the body length class of an arrow.

    Compares length / head width with the three real arrows. confidence is 1
    at an exact match and 0 half way to the next class.
    """
    ratio = measure.length / measure.head_width
    nominal = {name: (body + TIP_LENGTH) / HEAD_WIDTH
               for name, body in BODY_LENGTHS.items()}
    name = min(nominal, key=lambda n: abs(nominal[n] - ratio))
    spacing = min(abs(nominal[name] - other) for other in nominal.values()
                  if other != nominal[name])
    error = abs(nominal[name] - ratio)
    return name, float(max(0.0, 1.0 - error / (spacing / 2)))


def angle_to_dir(angle, tolerance=15):
    """Name the direction within tolerance of angle (degrees), or 'UNK'."""
    for name, center in DIRECTIONS.items():
        if abs((angle - center + 180) % 360 - 180) <= tolerance:
            return name
    return 'UNK'


def distance_from_head(measure, focal_px):
    """Distance along the optical axis, assuming the arrow faces the camera."""
    return focal_px * HEAD_WIDTH / measure.head_width
