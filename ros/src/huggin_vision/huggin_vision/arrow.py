"""Arrow detection on a wall, from its outline in the image.

The arrows are block arrows cut from paper: a body and a triangular head on
the same axis. Body and head widths are the same for every arrow; the three
length classes (close, normal, far) differ in body and head length. Their
real sizes are an ArrowSpec, so they can be changed from the parameters; the
constants below are the defaults (competition arrows, 4/11/2026).

How a blob is read:
1. The two notches where the body meets the head tell it is an arrow and
   give a first guess of its axis.
2. The three real arrows are drawn over the blob, with its length and
   center, pointing both ways. The one that overlaps it best (IoU: shared
   area / total area) gives the direction, the ends and a class from the
   shape alone.
3. When the distance to the wall is known (front ultrasonic), the class
   comes from the real length instead (class_from_length): the camera
   blurs away the sharp corners of the head, so the shape can mislead,
   but the length stays right.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

# Competition arrows (m)
BODY_WIDTH = 0.010
HEAD_WIDTH = 0.028  # at its base, where it meets the body
# (body length, head length) per class
SHAPES = {'close': (0.018, 0.011),
          'normal': (0.045, 0.012),
          'far': (0.075, 0.020)}

# Notches where the body meets the head: depth relative to sqrt(area)
MIN_NOTCH_DEPTH = 0.12
MIN_NOTCH_SYMMETRY = 0.5  # shallower notch / deeper notch

# A blob this elongated (ratio of its two inertia moments) is a normal or
# far arrow: its long axis is a better guess of the arrow axis than the
# notches
ELONGATED = 2.5
ANGLE_STEPS = (-4, 0, 4)  # degrees tried around the first guess
# Light and color thresholds make the blob a little thicker or thinner than
# the arrow all around. How much is measured on the body, on this stretch
# (part of the length from the tail): inside the body of every class
BODY_BAND = (0.15, 0.45)
# Most the outline can be off, as a share of the body width seen: at least
# half of it must be the real body. A wrong class would need more
MAX_GROWTH = 0.25
# Each class is scored by its overlap minus this times the growth it needed
# (share of the body width): when two classes fit about as well, the one
# that needed less made-up thickness wins
GROWTH_COST = 0.15
FIT_SIZE = 48  # the fit shrinks the blob to this many pixels: same result
MIN_IOU = 0.6  # less overlap with every real arrow: not an arrow
# Lead in IoU of the best class over the second one for full confidence
FULL_CONFIDENCE_MARGIN = 0.05

DIRECTIONS = {'right': 0, 'up': 90, 'left': 180, 'down': 270}


@dataclass
class ArrowSpec:
    """Real size of the arrows (meters)."""

    body_width: float = BODY_WIDTH
    head_width: float = HEAD_WIDTH
    # (body length, head length) per length class
    shapes: dict = field(default_factory=lambda: dict(SHAPES))


@dataclass
class ArrowMeasure:
    """Arrow found in the image (pixels, y down)."""

    tail: np.ndarray
    tip: np.ndarray
    head_width: float  # pixels
    length_class: str
    # 1 when the best class leads the next one clearly, near 0 on a tie
    confidence: float

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


def _outline(spec, name):
    """Outline (meters) of one real arrow: tail at x = 0, pointing to +x."""
    body, head = spec.shapes[name]
    bw, hw = spec.body_width / 2, spec.head_width / 2
    return np.array([[0, -bw], [body, -bw], [body, -hw], [body + head, 0],
                     [body, hw], [body, bw], [0, bw]])


def _grow(mask, pixels):
    """Move the outline of a 0/1 mask out (pixels > 0) or in (< 0)."""
    if pixels >= 0:
        outside = cv2.distanceTransform(1 - mask, cv2.DIST_L2, 3)
        return (outside <= pixels).astype(np.uint8)
    return (cv2.distanceTransform(mask, cv2.DIST_L2, 3) > -pixels) \
        .astype(np.uint8)


def _first_axis(contour, moments, notch_a, notch_b):
    """Return a unit vector along the arrow, from the notches or long axis.

    Its sign is not known yet: both ways are tried later.
    """
    across = (notch_b - notch_a) / max(np.linalg.norm(notch_b - notch_a),
                                       1e-9)
    axis = np.array([-across[1], across[0]])
    mu20, mu02, mu11 = moments['mu20'], moments['mu02'], moments['mu11']
    spread = np.hypot((mu20 - mu02) / 2, mu11)
    big, small = (mu20 + mu02) / 2 + spread, (mu20 + mu02) / 2 - spread
    if big > ELONGATED * max(small, 1e-9):
        theta = 0.5 * np.arctan2(2 * mu11, mu20 - mu02)
        axis = np.array([np.cos(theta), np.sin(theta)])
    return axis


def measure_arrow(contour, spec=None, min_area=100):
    """Return an ArrowMeasure for an arrow-shaped contour, else None."""
    spec = spec or ArrowSpec()
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

    moments = cv2.moments(contour)
    axis = _first_axis(contour, moments, a, b)
    base_deg = np.degrees(np.arctan2(axis[1], axis[0]))  # y down

    # Blob as an image, shrunk to FIT_SIZE, with room around it
    scale = min(1.0, FIT_SIZE / max(cv2.boundingRect(contour)[2:]))
    points = contour.reshape(-1, 2) * scale
    corner = points.min(axis=0) - FIT_SIZE / 2
    size = np.ceil(points.max(axis=0) - corner + FIT_SIZE / 2).astype(int)
    points = points - corner
    blob = np.zeros((size[1], size[0]), np.uint8)
    cv2.fillPoly(blob, [np.round(points).astype(np.int32)], 1)
    center = np.array([moments['m10'], moments['m01']]) / moments['m00']
    center = center * scale - corner
    inside = np.argwhere(blob)[:, ::-1].astype(np.float64)  # (x, y)

    best = {}  # class name -> (iou, tail, tip, head width), in blob pixels
    for turn in (0, 180):
        for step in ANGLE_STEPS:
            t = np.radians(base_deg + turn + step)
            along, across = np.array([np.cos(t), np.sin(t)]), \
                np.array([-np.sin(t), np.cos(t)])
            position = inside @ along
            start, end = position.min() - 0.5, position.max() + 0.5
            length = end - start
            # Mean width of the body, on a stretch near the tail
            band = (position >= start + BODY_BAND[0] * length) \
                & (position < start + BODY_BAND[1] * length)
            body = np.count_nonzero(band) \
                / max((BODY_BAND[1] - BODY_BAND[0]) * length, 1e-9)
            middle = center @ across
            for name in spec.shapes:
                poly = _outline(spec, name)
                # The blob is the real arrow, px_per_m pixels per meter,
                # with its outline moved out by grown pixels all around.
                # Its length and body width give both
                px_per_m = (length - body) / (poly[3, 0] - spec.body_width)
                if px_per_m <= 0:
                    continue
                grown = np.clip((body - px_per_m * spec.body_width) / 2,
                                -MAX_GROWTH * body, MAX_GROWTH * body)

                def place(p):
                    return np.outer(start + grown + p[..., 0] * px_per_m,
                                    along) \
                        + np.outer(middle + p[..., 1] * px_per_m, across)
                drawn = np.zeros_like(blob)
                cv2.fillPoly(drawn, [np.round(place(poly) * 8)
                                     .astype(np.int32)], 1, shift=3)
                # The real arrow, grown like the blob, against the blob
                drawn = _grow(drawn, grown)
                iou = np.count_nonzero(drawn & blob) \
                    / max(np.count_nonzero(drawn | blob), 1) \
                    - GROWTH_COST * abs(grown) / max(body, 1e-9)
                if name not in best or iou > best[name][0]:
                    best[name] = (iou, place(np.zeros(2))[0],
                                  place(poly[3])[0],
                                  px_per_m * spec.head_width)
    if not best:
        return None
    fits = [(fit[0], name) + fit[1:] for name, fit in best.items()]
    fits.sort(key=lambda f: -f[0])
    iou, name, tail, tip, head_width = fits[0]
    if iou < MIN_IOU:
        return None
    margin = iou - fits[1][0] if len(fits) > 1 else 1.0
    return ArrowMeasure(
        tail=(tail + corner) / scale, tip=(tip + corner) / scale,
        head_width=head_width / scale, length_class=name,
        confidence=float(min(1.0, margin / FULL_CONFIDENCE_MARGIN)))


def angle_to_dir(angle, tolerance=40):
    """Name the direction within tolerance of angle (degrees), or 'UNK'."""
    for name, center in DIRECTIONS.items():
        if abs((angle - center + 180) % 360 - 180) <= tolerance:
            return name
    return 'UNK'


def distance_from_head(measure, focal_px, head_width=HEAD_WIDTH):
    """Distance along the optical axis, assuming the arrow faces the camera.

    head_width is the real width of the head (meters).
    """
    return focal_px * head_width / measure.head_width


def class_from_length(measure, depth, focal_px, spec=None):
    """Return (name, confidence) of the class whose real length is nearest.

    depth is the distance from the camera to the wall along the optical axis
    (meters), so the length in pixels becomes meters. Lengths are compared
    as ratios: the boundary between two classes is their geometric mean
    (about 41 and 74 mm). confidence is 1 at an exact length and 0 at a
    boundary.
    """
    spec = spec or ArrowSpec()
    length = float(np.linalg.norm(measure.tip - measure.tail)) \
        * depth / focal_px
    logs = {name: np.log(body + head)
            for name, (body, head) in spec.shapes.items()}
    error = {name: abs(np.log(max(length, 1e-6)) - value)
             for name, value in logs.items()}
    name = min(error, key=error.get)
    half_gap = min(abs(logs[name] - other) for other_name, other
                   in logs.items() if other_name != name) / 2
    return name, float(max(0.0, 1.0 - error[name] / half_gap))
