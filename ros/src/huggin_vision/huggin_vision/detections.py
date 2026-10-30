"""Plain-Python result of the detectors, independent of the ROS messages."""
from dataclasses import dataclass, field

import numpy as np

CUBE = 'cube'
ARROW = 'arrow'
ZONE = 'zone'


@dataclass
class Detection:
    """A cube, arrow or zone, with its pose in the floor (output) frame.

    angle, polygon and the optional fields mean what they do in
    huggin_vision_msgs/Detection; the image-space ones are for drawing.
    """

    kind: str
    color: str
    position: np.ndarray
    orientation: tuple = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
    angle: float = 0.0
    direction: str = ''
    length_class: str = ''
    polygon: list = field(default_factory=list)  # [(x, y, z), ...]
    confidence: float = 1.0
    partial: bool = False
    contour: np.ndarray = None  # blob outline in the image
    axis_px: tuple = None  # arrow (tail, tip) in the image
    camera_pose: object = None  # cube geometry_msgs/Pose, camera frame
