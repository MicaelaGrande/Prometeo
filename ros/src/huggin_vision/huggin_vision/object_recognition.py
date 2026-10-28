import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (registers PoseStamped transforms)
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

# These are the default values of the vision_node parameters (see my_robot_bringup/config/vision.yaml)
FRAME_SIZE = (640, 480)  # (width, height) the frames are resized to
CUBE_SIZE = 0.02  # meters per face
MIN_AREA_CUBE = 500
MARKER_COLORS = {'red': (1.0, 0.0, 0.0), 'blue': (0.0, 0.0, 1.0),
                 'green': (0.0, 1.0, 0.0)}
MIN_AREA_ARROW = 500
MIN_AREA_ZONE = 500
RED_L  , RED_H   = (0, 120, 80)  , (10, 255, 255)
BLUE_L , BLUE_H  = (100, 120, 80), (130, 255, 255)
GREEN_L, GREEN_H = (40, 80, 80)  , (80, 255, 255)
TOLERANCE = 15 #degrees
HSV_RANGES = {'red': (RED_L, RED_H), 'blue': (BLUE_L, BLUE_H),
              'green': (GREEN_L, GREEN_H)}

def classify_arrow(contour, min_area=200):
    """Return (is_arrow, direction_unit_vector, tip_point, lenght) for a contour."""
    area = cv2.contourArea(contour)
    if area < min_area:
        return False, None, None

    # 1. Shape test: an arrow is concave and has about 7 corners
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    solidity = area / hull_area if hull_area else 1.0
    approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
    if solidity > 0.85 or not (6 <= len(approx) <= 9):
        return False, None, None, None

    # 2. Long axis of the shape via PCA
    pts = contour.reshape(-1, 2).astype(np.float64)
    mean = pts.mean(axis=0)
    axis = np.linalg.svd(pts - mean, full_matrices=False)[2][0]
    perp = np.array([-axis[1], axis[0]])
    along = (pts - mean) @ axis
    across = (pts - mean) @ perp

    # 3. The head is the end that is wider across the axis
    lo, hi = along.min(), along.max()
    span = hi - lo
    width_lo = np.ptp(across[along < lo + 0.3 * span])
    width_hi = np.ptp(across[along > hi - 0.3 * span])
    direction = axis if width_hi > width_lo else -axis

    tip = pts[np.argmax((pts - mean) @ direction)]
    return True, direction, tip, len(approx)

def rotation_to_quaternion(R):
    """Return (x, y, z, w) for a 3x3 rotation matrix."""
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

CUBE_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6),
              (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]

def annotate_frame(frame, cubes, camera_matrix, cube_size=CUBE_SIZE,
                   colors=MARKER_COLORS):
    """Return a copy of frame (BGR) with each (color, Pose) cube drawn as a
    projected wireframe plus a label with its distance and camera-frame position."""
    out = frame.copy()
    h = cube_size / 2
    model = np.array([[x, y, z] for x in (-h, h) for y in (-h, h)
                      for z in (-h, h)], dtype=np.float64)
    for color, pose in cubes:
        bgr = tuple(int(255 * c) for c in colors[color][::-1])
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        rvec = cv2.Rodrigues(quaternion_to_rotation(pose.orientation))[0]
        pts = cv2.projectPoints(model, rvec, pos, camera_matrix, None)[0]
        pts = pts.reshape(-1, 2).astype(int)
        for a, b in CUBE_EDGES:
            cv2.line(out, tuple(pts[a]), tuple(pts[b]), bgr, 2)
        center = cv2.projectPoints(
            np.zeros((1, 3)), np.zeros(3), pos, camera_matrix, None
        )[0].reshape(2).astype(int)
        cv2.circle(out, tuple(center), 3, (255, 255, 255), -1)
        label = f'{color} cube {np.linalg.norm(pos):.2f}m'
        coords = f'x={pos[0]:+.2f} y={pos[1]:+.2f} z={pos[2]:.2f}'
        x0, y0 = int(pts[:, 0].min()), int(pts[:, 1].min())
        for i, text in enumerate((label, coords)):
            org = (max(x0, 0), max(y0 - 22 + 14 * i, 12))
            cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, f'cubes: {len(cubes)}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return out

def _nelder_mead(f, x0, step, iters=60):
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

def _fit_cube_silhouette(contour, camera_matrix, cube_size):
    """Fit a full cube pose to a contour; returns (position, rotation matrix).

    The center is pinned to the contour centroid's viewing ray, so only the
    depth and the rotation are searched, minimizing 1 - IoU between the
    contour and the projected cube silhouette (plus a small prior favoring
    face-on, since a square blob is otherwise ambiguous in depth). Depth is well determined even
    though the orientation of a tilted cube is only loosely constrained.
    """
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    h = cube_size / 2
    verts = np.array([[x, y, z] for x in (-h, h) for y in (-h, h)
                      for z in (-h, h)], dtype=np.float64)

    # Work in a small ROI so each render is cheap
    x0, y0, bw, bh = cv2.boundingRect(contour)
    pad = max(bw, bh)
    x0, y0 = x0 - pad, y0 - pad
    size = (bh + 2 * pad, bw + 2 * pad)
    target = np.zeros(size, np.uint8)
    cv2.drawContours(target, [contour - [x0, y0]], -1, 255, cv2.FILLED)
    target = target > 0

    m = cv2.moments(contour)
    u, v = m['m10'] / m['m00'], m['m01'] / m['m00']
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

    def cost(p):
        z = p[0]
        if z <= 0.05:
            return 1.0
        R = cv2.Rodrigues(p[1:4])[0]
        pts = cv2.projectPoints(verts, cv2.Rodrigues(R)[0], z * ray,
                                camera_matrix, None)[0].reshape(-1, 2)
        hull = cv2.convexHull((pts - [x0, y0]).astype(np.float32))
        mask = np.zeros(size, np.uint8)
        cv2.fillConvexPoly(mask, np.round(hull).astype(np.int32), 255)
        mask = mask > 0
        union = np.count_nonzero(mask | target)
        if not union:
            return 1.0
        # Small prior toward face-on: a square blob fits many tilt/depth pairs
        return (1.0 - np.count_nonzero(mask & target) / union
                + 0.03 * np.dot(p[1:4], p[1:4]))

    z0 = fx * cube_size / np.sqrt(cv2.contourArea(contour)) * 1.2
    starts = [(0, 0, 0), (0.6, 0, 0), (0, 0.6, 0), (0.5, 0.5, 0.4),
              (-0.5, 0.4, 0.2), (0, 0, 0.78)]
    best, best_cost = None, np.inf
    for r in starts:
        # The cost is sharp in depth, so scan it before refining
        z = min(np.linspace(0.6 * z0, 1.5 * z0, 25),
                key=lambda z: cost(np.array([z, *r])))
        x, c = _nelder_mead(cost, [z, *r], [0.03 * z, 0.2, 0.2, 0.2], 80)
        if c < best_cost:
            best, best_cost = x, c
    return best[0] * ray, cv2.Rodrigues(best[1:4])[0]

def estimate_cube_poses(mask, camera_matrix, cube_size=CUBE_SIZE,
                        min_area=MIN_AREA_CUBE):
    """Return a list of geometry_msgs/Pose, one per cube found in a binary mask.

    Poses are the cube center expressed in the camera optical frame
    (x right, y down, z forward), in meters. camera_matrix must match the
    mask resolution. The position comes from fitting the cube's projected
    silhouette to the blob; the orientation is only a best fit and is loosely
    constrained (cube symmetry, and a silhouette says little about tilt).
    """
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    poses = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        # Cubes look roughly square: reject elongated or sparse blobs
        w, hh = cv2.minAreaRect(contour)[1]
        if min(w, hh) == 0 or min(w, hh) / max(w, hh) < 0.6 \
                or area / (w * hh) < 0.7:
            continue

        pos, R = _fit_cube_silhouette(contour, camera_matrix, cube_size)
        quat = rotation_to_quaternion(R)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, pos)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = map(float, quat)
        poses.append(pose)
    return poses

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.K = None
        self.declare_params()
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_cb, 10)
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        self.debug_pub = self.create_publisher(Image, '/vision/annotated', 10)
        # Markers are drawn in a fixed frame so they stay put as the robot moves
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/vision/cube_markers', 10)
        # One PoseArray per cube color, in the camera optical frame
        self.cube_pubs = {
            color: self.create_publisher(PoseArray, f'/vision/cubes/{color}', 10)
            for color in ('red', 'blue', 'green')}

    def declare_params(self):
        """Declare the tunables; the module constants are the defaults."""
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('frame_width', FRAME_SIZE[0])
        self.declare_parameter('frame_height', FRAME_SIZE[1])
        self.declare_parameter('cube_size', float(CUBE_SIZE))
        self.declare_parameter('arrow_tolerance_deg', float(TOLERANCE))
        self.declare_parameter('min_area.cube', MIN_AREA_CUBE)
        self.declare_parameter('min_area.arrow', MIN_AREA_ARROW)
        self.declare_parameter('min_area.zone', MIN_AREA_ZONE)
        for color, (lower, upper) in HSV_RANGES.items():
            self.declare_parameter(f'hsv.{color}.lower', list(lower))
            self.declare_parameter(f'hsv.{color}.upper', list(upper))
            self.declare_parameter(f'marker_color.{color}',
                                   [float(c) for c in MARKER_COLORS[color]])

    def param(self, name):
        """Current value of a parameter (read each time so it can be retuned live)."""
        return self.get_parameter(name).value

    def info_cb(self, msg):
        """Keep intrinsics scaled to the resized frame used for detection."""
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        K[0] *= self.param('frame_width') / msg.width
        K[1] *= self.param('frame_height') / msg.height
        self.K = K

    def angle_to_dir(self, angle, tolerance=None):
        if tolerance is None:
            tolerance = self.param('arrow_tolerance_deg')
        cases = {
            'right': 0,
            'up': 90,
            'left': 180,
            'down': 270
        }
        for key, val in cases.items():
            if val-tolerance <= angle < val+tolerance:
                return key

        return "UNK"

    def get_hsv(self):
        """ Compatibility, turn msg (sensor_msg/image_raw) into hsv format (Hue Saturation Value)"""
        self.frame = self.bridge.imgmsg_to_cv2(self.msg, 'bgr8')
        self.frame = cv2.resize(
            self.frame, (self.param('frame_width'), self.param('frame_height')))
        blurred = cv2.GaussianBlur(self.frame, (5, 5), 0)
        return cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    def color_mask(self):
        hsv = self.get_hsv()
        self.masks = {
            color: cv2.inRange(hsv,
                               tuple(self.param(f'hsv.{color}.lower')),
                               tuple(self.param(f'hsv.{color}.upper')))
            for color in HSV_RANGES
        }

    def find_contours(self):
        self.contours = {
            'red': [],
            'blue': [],
            'green': [],
        }
        for label, mask in self.masks.items():
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
            contours_aux, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.contours[label] = contours_aux

    def is_arrow(self, contour, min_area=None):
        if min_area is None:
            min_area = self.param('min_area.arrow')
        is_arrow = cv2.contourArea(contour) < min_area
        return False

    def is_cube(self, contour, min_area=None):
        if min_area is None:
            min_area = self.param('min_area.cube')
        return True

    def is_zone(self, contour, min_area=None):
        if min_area is None:
            min_area = self.param('min_area.zone')
        return False

    def clear_detections(self):
        self.detections = {
            'arrow': [],
            'cube': [],
            'zone': []
        }

    def send_msgs(self):
        """Publish detected cubes, one PoseArray per color (empty if none)."""
        for color, pub in self.cube_pubs.items():
            out = PoseArray()
            out.header = self.msg.header
            out.poses = [p for c, p in self.detections['cube'] if c == color]
            pub.publish(out)
        self.publish_markers()
        self.publish_annotated()

    def publish_annotated(self):
        """Publish the frame with the detections drawn on it."""
        if self.K is None:
            annotated = self.frame
        else:
            annotated = annotate_frame(
                self.frame, self.detections['cube'], self.K,
                self.param('cube_size'), self.marker_colors())
        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = self.msg.header
        self.debug_pub.publish(out)

    def marker_colors(self):
        return {c: self.param(f'marker_color.{c}') for c in HSV_RANGES}

    def to_world(self, pose, world):
        """Express a camera-frame pose in the fixed frame, or None if no TF."""
        stamped = PoseStamped()
        stamped.header.frame_id = self.msg.header.frame_id
        stamped.pose = pose
        for stamp in (self.msg.header.stamp, Time().to_msg()):  # then latest
            stamped.header.stamp = stamp
            try:
                return self.tf_buffer.transform(
                    stamped, world, timeout=Duration(seconds=0.05)).pose
            except tf2_ros.TransformException as e:
                err = e
        self.get_logger().warn(
            f'No TF to {world}: {err}', throttle_duration_sec=5.0)
        return None

    def publish_markers(self):
        """Publish a cube model (RViz Marker) at each detected pose."""
        world = self.param('world_frame')
        colors = self.marker_colors()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = world
        clear.header.stamp = self.msg.header.stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (color, pose) in enumerate(self.detections['cube']):
            world_pose = self.to_world(pose, world)
            if world_pose is None:
                continue
            m = Marker()
            m.header.frame_id = world
            m.header.stamp = self.msg.header.stamp
            m.ns = 'cubes'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = world_pose
            m.scale.x = m.scale.y = m.scale.z = self.param('cube_size')
            m.color.r, m.color.g, m.color.b = colors[color]
            m.color.a = 0.9
            markers.markers.append(m)
        self.marker_pub.publish(markers)

    def image_cb(self, msg):
        self.msg = msg
        self.clear_detections()
        # Apply the color masks
        self.color_mask()
        # Find all the contours for each mask
        self.find_contours()
        # Cube poses (camera optical frame), as (color, Pose)
        if self.K is not None:
            for label, mask in self.masks.items():
                self.detections['cube'] += [
                    (label, p) for p in estimate_cube_poses(
                        mask, self.K, self.param('cube_size'),
                        self.param('min_area.cube'))]
        # Find all elements of interest
        # for label, cs in self.contours:
        #     for c in cs:
        #         # arrow
        #         if self.is_arrow(c):
        #             self.detections['arrow'] += [c]
        #         # zone
        #         if self.is_zone(c):
        #             self.detections['zone'] += [c]
        # Comunicate to ROS Network
        self.send_msgs()



def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
