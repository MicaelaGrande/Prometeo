"""Where the camera is relative to the floor frame, with or without TF.

The floor frame (default base_footprint) has x forward, y left, z up and its
origin on the floor. The camera optical frame has x right, y down, z forward.
"""
from dataclasses import dataclass

import numpy as np

# Rotation from the optical frame to a camera frame looking along +x
OPTICAL_TO_FORWARD = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])


@dataclass
class CameraMount:
    """Where the camera sits on the robot and how its pan servo reads."""

    x: float = 0.09  # m, forward of the floor frame origin
    y: float = 0.0  # m, to the left
    height: float = 0.127  # m, above the floor
    pitch_deg: float = 0.0  # tilt down
    pan_center_deg: float = 90.0  # servo angle that looks straight ahead
    pan_sign: float = 1.0  # +1 if a higher servo angle turns the camera left


@dataclass
class CameraView:
    """Camera intrinsics plus pose in the floor frame for one image."""

    matrix: np.ndarray  # 3x3 intrinsics, for the processed image size
    rotation: np.ndarray  # 3x3, optical frame -> floor frame
    translation: np.ndarray  # optical origin in the floor frame

    def to_floor(self, point):
        """Express a point of the optical frame in the floor frame."""
        return self.rotation @ np.asarray(point, dtype=np.float64) \
            + self.translation

    def direction_to_floor(self, direction):
        """Rotate a direction of the optical frame to the floor frame."""
        return self.rotation @ np.asarray(direction, dtype=np.float64)

    def pixel_ray(self, u, v):
        """Return the optical-frame ray (z = 1) through pixel (u, v)."""
        K = self.matrix
        return np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1],
                         1.0])

    def pixel_to_floor_plane(self, u, v, z=0.0):
        """Return where pixel (u, v) hits the plane height z (floor frame).

        None if the ray is level or points up, so it never reaches it.
        """
        direction = self.direction_to_floor(self.pixel_ray(u, v))
        if direction[2] > -1e-6:
            return None
        scale = (z - self.translation[2]) / direction[2]
        if scale <= 0:
            return None
        return self.translation + scale * direction


def mount_pose(mount, pan_deg):
    """Return (rotation, translation) of the optical frame in the floor frame.

    pan_deg is the servo angle; yaw turns about the vertical, then the fixed
    pitch tilts the camera down.
    """
    yaw = np.radians(mount.pan_sign * (pan_deg - mount.pan_center_deg))
    pitch = np.radians(mount.pitch_deg)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    r_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    r_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation = r_yaw @ r_pitch @ OPTICAL_TO_FORWARD
    return rotation, np.array([mount.x, mount.y, mount.height])


def view_from_mount(matrix, mount, pan_deg):
    """Build the CameraView of a camera on mount, panned to pan_deg."""
    rotation, translation = mount_pose(mount, pan_deg)
    return CameraView(np.asarray(matrix, dtype=np.float64), rotation,
                      translation)
