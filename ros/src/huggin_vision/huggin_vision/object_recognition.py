"""ROS node detecting colored cubes, arrows and zones in the camera image.

Pipeline per image: preprocess -> color masks -> detections -> publish.
Detections are expressed in a floor frame, so the robot logic does not need
TF: the camera pose comes from the mount parameters and the pan servo angle,
or from TF when camera.use_tf is set (simulation).
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
from huggin_vision_msgs.msg import Detection, DetectionArray

from huggin_vision.blobs import DetectorConfig, detect_blobs
from huggin_vision.camera_model import (
    CameraMount, CameraView, view_from_mount)
from huggin_vision.detections import CUBE
from huggin_vision.geometry import quaternion_to_rotation
from huggin_vision.segmentation import (
    HSV_RANGES, MARKER_COLORS, color_masks, preprocess)
from huggin_vision.visualization import annotate_frame, make_markers
from huggin_vision.zone import ZoneConfig

# Defaults of the vision_node parameters (see my_robot_bringup/config/vision.yaml)
FRAME_SIZE = (640, 480)  # (width, height) the frames are resized to
OUTPUT_FRAME = 'base_footprint'  # on the floor, x forward, y left, z up
MAX_FPS = 15.0  # 0 = process every image

# Latest image wins: drop old ones instead of queueing when we are slow
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.camera_matrix = None  # scaled to the resized frame
        self.pan_deg = None  # last servo command
        self.last_image_time = 0.0
        self._declare_params()

        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_cb, IMAGE_QOS)
        self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, IMAGE_QOS)
        self.create_subscription(
            Float32, self._param('camera.pan_topic'), self.pan_cb, 10)

        self.detections_pub = self.create_publisher(
            DetectionArray, '/vision/detections', 10)
        self.annotated_pub = self.create_publisher(
            Image, '/vision/annotated', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/vision/markers', 10)
        # Cubes in the camera optical frame, one PoseArray per color
        self.cube_pubs = {
            color: self.create_publisher(
                PoseArray, f'/vision/cubes/{color}', 10)
            for color in HSV_RANGES}

        self.tf_buffer = None
        if self._param('camera.use_tf'):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _declare_params(self):
        """Declare the tunables; the module constants are the defaults."""
        mount, config = CameraMount(), DetectorConfig()
        self.declare_parameter('output_frame', OUTPUT_FRAME)
        self.declare_parameter('frame_width', FRAME_SIZE[0])
        self.declare_parameter('frame_height', FRAME_SIZE[1])
        self.declare_parameter('max_fps', MAX_FPS)

        self.declare_parameter('camera.use_tf', False)
        self.declare_parameter('camera.pan_topic', 'camara/cmd')
        self.declare_parameter('camera.x', mount.x)
        self.declare_parameter('camera.y', mount.y)
        self.declare_parameter('camera.height', mount.height)
        self.declare_parameter('camera.pitch_deg', mount.pitch_deg)
        self.declare_parameter('camera.pan_center_deg', mount.pan_center_deg)
        self.declare_parameter('camera.pan_sign', mount.pan_sign)

        self.declare_parameter('cube_size', config.cube_size)
        self.declare_parameter('zone_size', config.zone.size)
        self.declare_parameter('zone_tolerance', config.zone.tolerance)
        self.declare_parameter('arrow_tolerance_deg',
                               config.arrow_tolerance_deg)
        self.declare_parameter('min_area.cube', config.min_area_cube)
        self.declare_parameter('min_area.arrow', config.min_area_arrow)
        self.declare_parameter('min_area.zone', config.min_area_zone)
        for color, (lower, upper) in HSV_RANGES.items():
            self.declare_parameter(f'hsv.{color}.lower', list(lower))
            self.declare_parameter(f'hsv.{color}.upper', list(upper))
            self.declare_parameter(f'marker_color.{color}',
                                   list(MARKER_COLORS[color]))

    def _param(self, name):
        """Current value of a parameter (read each time so it can be
        retuned live)."""
        return self.get_parameter(name).value

    def _frame_size(self):
        return self._param('frame_width'), self._param('frame_height')

    def _hsv_ranges(self):
        return {color: (self._param(f'hsv.{color}.lower'),
                        self._param(f'hsv.{color}.upper'))
                for color in HSV_RANGES}

    def _marker_colors(self):
        return {color: self._param(f'marker_color.{color}')
                for color in HSV_RANGES}

    def _detector_config(self):
        return DetectorConfig(
            cube_size=self._param('cube_size'),
            min_area_cube=self._param('min_area.cube'),
            min_area_arrow=self._param('min_area.arrow'),
            min_area_zone=self._param('min_area.zone'),
            arrow_tolerance_deg=self._param('arrow_tolerance_deg'),
            zone=ZoneConfig(self._param('zone_size'),
                            self._param('zone_tolerance')))

    def _mount(self):
        return CameraMount(
            x=self._param('camera.x'), y=self._param('camera.y'),
            height=self._param('camera.height'),
            pitch_deg=self._param('camera.pitch_deg'),
            pan_center_deg=self._param('camera.pan_center_deg'),
            pan_sign=self._param('camera.pan_sign'))

    def info_cb(self, msg):
        """Keep intrinsics scaled to the resized frame used for detection."""
        width, height = self._frame_size()
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        K[0] *= width / msg.width
        K[1] *= height / msg.height
        self.camera_matrix = K

    def pan_cb(self, msg):
        self.pan_deg = msg.data

    def _camera_view(self, header):
        """Camera pose in the output frame for this image, or None."""
        if self.tf_buffer is None:
            mount = self._mount()
            pan = mount.pan_center_deg if self.pan_deg is None else self.pan_deg
            return view_from_mount(self.camera_matrix, mount, pan)
        try:
            tf = self.tf_buffer.lookup_transform(
                self._param('output_frame'), header.frame_id, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'No TF to {self._param("output_frame")}: {e}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return CameraView(self.camera_matrix,
                          quaternion_to_rotation(tf.transform.rotation),
                          np.array([t.x, t.y, t.z]))

    def _too_soon(self):
        """True if the previous image was processed less than 1/max_fps ago."""
        max_fps = self._param('max_fps')
        now = time.monotonic()
        if max_fps > 0 and now - self.last_image_time < 1.0 / max_fps:
            return True
        self.last_image_time = now
        return False

    def image_cb(self, msg):
        if self.camera_matrix is None or self._too_soon():
            return
        camera = self._camera_view(msg.header)
        if camera is None:
            return

        started = time.perf_counter()
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame, hsv = preprocess(bgr, self._frame_size())
        masks = color_masks(hsv, self._hsv_ranges())
        masked = time.perf_counter()

        config = self._detector_config()
        detections = []
        for color, mask in masks.items():
            detections += detect_blobs(color, mask, camera, config)
        detected = time.perf_counter()

        self._publish(msg.header, frame, detections)
        self.get_logger().debug(
            f'preprocess {1e3 * (masked - started):.0f} ms, '
            f'detect {1e3 * (detected - masked):.0f} ms, '
            f'publish {1e3 * (time.perf_counter() - detected):.0f} ms, '
            f'{len(detections)} detections')

    def _publish(self, header, frame, detections):
        frame_id = self._param('output_frame')
        out = DetectionArray()
        out.header.stamp = header.stamp
        out.header.frame_id = frame_id
        out.detections = [self._to_msg(d) for d in detections]
        self.detections_pub.publish(out)

        for color, pub in self.cube_pubs.items():
            cubes = PoseArray()
            cubes.header = header
            cubes.poses = [d.camera_pose for d in detections
                           if d.kind == CUBE and d.color == color]
            pub.publish(cubes)

        colors = self._marker_colors()
        cube_size = self._param('cube_size')
        self.marker_pub.publish(make_markers(
            detections, frame_id, header.stamp, cube_size, colors))
        annotated = self.bridge.cv2_to_imgmsg(
            annotate_frame(frame, detections, self.camera_matrix, cube_size,
                           colors), 'bgr8')
        annotated.header = header
        self.annotated_pub.publish(annotated)

    @staticmethod
    def _to_msg(d):
        msg = Detection()
        msg.kind, msg.color = d.kind, d.color
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
            map(float, d.position)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = d.orientation
        msg.angle = d.angle
        msg.direction, msg.length_class = d.direction, d.length_class
        msg.polygon = [Point(x=x, y=y, z=z) for x, y, z in d.polygon]
        msg.confidence, msg.partial = float(d.confidence), d.partial
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
