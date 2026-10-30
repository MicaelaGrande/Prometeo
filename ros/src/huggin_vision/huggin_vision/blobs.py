"""Turn the color masks into cubes, arrows and zones.

Every blob of a mask is exactly one of them, tried in this order: a zone if
it is big, then an arrow if it has the arrow's notches, then a cube if it is
a filled square.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

from huggin_vision.arrow import (
    angle_to_dir, distance_from_head, length_class, measure_arrow)
from huggin_vision.cube_fit import fit_cube_silhouette, is_cube_shaped, make_pose
from huggin_vision.detections import ARROW, CUBE, Detection
from huggin_vision.geometry import rotation_to_quaternion
from huggin_vision.zone import ZoneConfig, detect_zone

# An arrow whose length class is this unsure is not reported
MIN_ARROW_CONFIDENCE = 0.1


@dataclass
class DetectorConfig:
    """Sizes and thresholds of the detectors."""

    cube_size: float = 0.02
    min_area_cube: int = 500  # pixels
    min_area_arrow: int = 100
    min_area_zone: int = 4000
    arrow_tolerance_deg: float = 15.0
    zone: ZoneConfig = field(default_factory=ZoneConfig)


def detect_cube(color, contour, camera, config):
    """Fit a cube to a blob; its pose is expressed in the floor frame."""
    position, rotation = fit_cube_silhouette(
        contour, camera.matrix, config.cube_size)
    in_floor = camera.rotation @ rotation
    # The cube is symmetric, so its yaw only makes sense modulo 90 degrees
    yaw = np.arctan2(in_floor[1, 0], in_floor[0, 0])
    yaw = (yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
    return Detection(
        kind=CUBE, color=color, position=camera.to_floor(position),
        orientation=rotation_to_quaternion(in_floor), angle=float(yaw),
        contour=contour, camera_pose=make_pose(position, rotation))


def detect_arrow(color, contour, camera, config):
    """Return the Detection of an arrow blob, or None if it is not one."""
    measure = measure_arrow(contour, config.min_area_arrow)
    if measure is None:
        return None
    name, confidence = length_class(measure)
    if confidence < MIN_ARROW_CONFIDENCE:
        return None

    # The arrow faces the camera, so its width gives its distance
    depth = distance_from_head(measure, camera.matrix[0, 0])
    end_points = [camera.to_floor(camera.pixel_ray(*p) * depth)
                  for p in (measure.tail, measure.tip)]
    angle_deg = measure.angle_deg
    return Detection(
        kind=ARROW, color=color,
        position=camera.to_floor(camera.pixel_ray(*measure.center) * depth),
        angle=float(np.radians(angle_deg)),
        direction=angle_to_dir(angle_deg, config.arrow_tolerance_deg),
        length_class=name, polygon=[tuple(map(float, p)) for p in end_points],
        confidence=confidence, contour=contour,
        axis_px=(measure.tail, measure.tip))


def detect_blobs(color, mask, camera, config):
    """Return the list of Detection found in the binary mask of one color."""
    # Every outline point is kept: the arrow is measured from them
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= config.min_area_zone:
            detection = detect_zone(
                color, contour, camera, mask.shape, config.zone)
        else:
            detection = detect_arrow(color, contour, camera, config)
            if detection is None and is_cube_shaped(
                    contour, config.min_area_cube):
                detection = detect_cube(color, contour, camera, config)
        if detection is not None:
            found.append(detection)
    return found
