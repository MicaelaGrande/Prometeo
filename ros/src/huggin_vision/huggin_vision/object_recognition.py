import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MIN_AREA = 500
RED_L  , RED_H   = (0, 120, 80)  , (10, 255, 255)
BLUE_L , BLUE_H  = (100, 120, 80), (130, 255, 255)
GREEN_L, GREEN_H = (40, 80, 80)  , (80, 255, 255)
TOLERANCE = 15 #degrees

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

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        self.debug_pub = self.create_publisher(Image, '/vision/annotated', 10)

    def angle_to_dir(self, angle, tolerance):
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


    def find_arrows(self, frame):
        """ Formato de salida color, tamaño, angulo (0 izq, 180 der)"""
        arrow_list = []

        frame = cv2.resize(frame, (640, 480))
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        masks = {
            'red': cv2.inRange(hsv, RED_L, RED_H),
            'blue': cv2.inRange(hsv, BLUE_L, BLUE_H),
            'green': cv2.inRange(hsv, GREEN_L, GREEN_H),
        }

        annotated = frame.copy()
        for label, mask in masks.items():
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
            mask = cv2.morphologyEx(
                mask, cv2.MORPH_CLOSE,
                cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                  area = cv2.contourArea(c)
                  if area < MIN_AREA:
                      continue
                  is_arrow, direction, tip, size = classify_arrow(c)
                  if not is_arrow:
                      continue

                  cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)

                  angle = np.degrees(np.arctan2(-direction[1], direction[0]))  # minus because image y points down

                  dir = self.angle_to_dir(angle, tolerance=15)

                  cv2.putText(annotated, f'{label}, {size}, {dir}', np.add(tuple(tip.astype(int)), (4, -4)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)

                  cv2.circle(annotated, center=tuple(tip.astype(int)), radius=2, color=(0, 255, 0), thickness=2)

        debug_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        debug_msg.header = self.msg.header
        self.debug_pub.publish(debug_msg)
        return arrow_list

    def image_cb(self, msg):
        self.msg = msg
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        arrows = self.find_arrows(frame)



def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
