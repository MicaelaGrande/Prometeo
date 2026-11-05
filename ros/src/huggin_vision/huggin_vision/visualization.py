"""Drawing of detections: annotated image and RViz markers."""
import cv2
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from huggin_vision.detections import ARROW, CUBE, ZONE
from huggin_vision.geometry import (
    CUBE_EDGES, cube_vertices, quaternion_to_rotation)

WHITE = (255, 255, 255)
MARKER_ALPHA = 0.9
ZONE_LINE_WIDTH = 0.01  # m
NO_GO_COLOR = 'red'


def draw_outlined_text(img, text, org, scale=0.45):
    """Draw white text with a black outline so it reads on any background."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, text, org, font, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, font, scale, WHITE, 1, cv2.LINE_AA)


def label_lines(detection):
    """Return the text lines to show next to a detection."""
    name = f'{detection.color} {detection.kind}'
    x, y, z = detection.position
    if detection.kind == CUBE:
        return [f'{name} {np.linalg.norm(detection.position):.2f}m',
                f'x={x:+.2f} y={y:+.2f}']
    if detection.kind == ARROW:
        return [f'{name} {detection.direction} {detection.length_class}',
                f'x={x:+.2f} y={y:+.2f}']
    extra = ' NO-GO' if detection.color == NO_GO_COLOR else ''
    partial = ' (partial)' if detection.partial else ''
    return [f'{name}{extra}{partial}', f'x={x:+.2f} y={y:+.2f}']


def _draw_cube(out, detection, camera_matrix, cube_size, bgr):
    """Draw the fitted cube as a projected wireframe; return its top-left."""
    pose = detection.camera_pose
    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    rvec = cv2.Rodrigues(quaternion_to_rotation(pose.orientation))[0]
    pts = cv2.projectPoints(cube_vertices(cube_size), rvec, pos,
                            camera_matrix, None)[0].reshape(-1, 2).astype(int)
    for a, b in CUBE_EDGES:
        cv2.line(out, tuple(pts[a]), tuple(pts[b]), bgr, 2)
    center = cv2.projectPoints(
        np.zeros((1, 3)), np.zeros(3), pos, camera_matrix, None
    )[0].reshape(2).astype(int)
    cv2.circle(out, tuple(center), 3, WHITE, -1)
    return int(pts[:, 0].min()), int(pts[:, 1].min())


def annotate_frame(frame, detections, camera_matrix, cube_size, colors):
    """Return a copy of frame (BGR) with every detection drawn on it."""
    out = frame.copy()
    for detection in detections:
        bgr = tuple(int(255 * c) for c in colors[detection.color][::-1])
        if detection.kind == CUBE:
            left, top = _draw_cube(out, detection, camera_matrix,
                                   cube_size, bgr)
        else:
            cv2.polylines(out, [detection.contour.reshape(-1, 1, 2)], True,
                          bgr, 2)
            x, y, w, h = cv2.boundingRect(detection.contour)
            left, top = x, y
            if detection.kind == ARROW:
                tail, tip = (tuple(int(c) for c in p)
                             for p in detection.axis_px)
                cv2.arrowedLine(out, tail, tip, WHITE, 2, tipLength=0.3)
        for i, text in enumerate(label_lines(detection)):
            draw_outlined_text(
                out, text, (max(left, 0), max(top - 22 + 14 * i, 12)))
    cv2.putText(out, f'detections: {len(detections)}', (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2, cv2.LINE_AA)
    return out


def _marker(kind, index, marker_type, frame_id, stamp, rgb):
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = kind
    m.id = index
    m.type = marker_type
    m.action = Marker.ADD
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = MARKER_ALPHA
    return m


def _set_position(marker, position):
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
        map(float, position)
    marker.pose.orientation.w = 1.0


def make_markers(detections, frame_id, stamp, cube_size, colors):
    """Return a MarkerArray showing the detections in frame_id.

    Cubes are cubes, zones are their outline on the floor, arrows a sphere
    with a text. A DELETEALL first removes the previous frame's markers.
    """
    markers = MarkerArray()
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    for i, d in enumerate(detections):
        rgb = colors[d.color]
        if d.kind == CUBE:
            m = _marker(d.kind, i, Marker.CUBE, frame_id, stamp, rgb)
            _set_position(m, d.position)
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = d.orientation
            m.scale.x = m.scale.y = m.scale.z = cube_size
        elif d.kind == ZONE:
            m = _marker(d.kind, i, Marker.LINE_STRIP, frame_id, stamp, rgb)
            m.pose.orientation.w = 1.0
            ring = list(d.polygon) + [d.polygon[0]]
            m.points = [Point(x=x, y=y, z=0.005) for x, y, _ in ring]
            m.scale.x = ZONE_LINE_WIDTH * (2 if d.color == NO_GO_COLOR else 1)
        else:
            m = _marker(d.kind, i, Marker.SPHERE, frame_id, stamp, rgb)
            _set_position(m, d.position)
            m.scale.x = m.scale.y = m.scale.z = 0.03
            text = _marker('arrow_label', i, Marker.TEXT_VIEW_FACING,
                           frame_id, stamp, (1.0, 1.0, 1.0))
            _set_position(text, d.position + [0.0, 0.0, 0.05])
            text.scale.z = 0.04
            text.text = f'{d.direction} {d.length_class}'
            markers.markers.append(text)
        markers.markers.append(m)
    return markers
