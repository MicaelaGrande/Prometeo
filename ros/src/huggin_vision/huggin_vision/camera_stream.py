#!/usr/bin/env python3
"""
ROS 2 node: subscribe to a sensor_msgs/Image topic, convert each message to an
OpenCV image, and push it live to a local web page.

Requirements (ROS 2 sourced, e.g. `source /opt/ros/<distro>/setup.bash`):
    sudo apt install ros-<distro>-cv-bridge
    pip install opencv-python        # or: sudo apt install python3-opencv

Run:
    python3 ros_camera_stream.py
    python3 ros_camera_stream.py --ros-args -p topic:=robot/camera -p port:=8000

Then open http://localhost:8000

Parameters:
    topic    (string)  image topic to subscribe to      default: robot/camera
    host     (string)  address the web server binds to  default: 0.0.0.0
    port     (int)     web server port                  default: 8000
    quality  (int)     JPEG quality 1-100               default: 80
"""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TOPIC__</title>
<style>
  :root { color-scheme: light dark; }
  body {
    margin: 0;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 12px;
    font-family: system-ui, sans-serif;
    background: Canvas;
    color: CanvasText;
  }
  img {
    max-width: 96vw;
    max-height: 88vh;
    border-radius: 8px;
    background: #000;
  }
  p { margin: 0; opacity: .7; font-size: .9rem; }
</style>
</head>
<body>
  <img src="/stream" alt="Live camera feed">
  <p>Live from <code>__TOPIC__</code></p>
</body>
</html>
"""


class FrameBuffer:
    """Holds the latest JPEG and wakes up any waiting viewers when a new one arrives."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._frame_id = 0

    def put(self, jpeg_bytes):
        with self._cond:
            self._jpeg = jpeg_bytes
            self._frame_id += 1
            self._cond.notify_all()

    def wait_for_frame(self, last_id, timeout=5):
        with self._cond:
            self._cond.wait_for(lambda: self._frame_id != last_id, timeout)
            return self._jpeg, self._frame_id


def make_handler(buffer, topic):
    page = PAGE.replace("__TOPIC__", topic).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path == "/stream":
                self.stream()
            else:
                self.send_error(404)

        def stream(self):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            last_id = -1
            try:
                while True:
                    jpeg, last_id = buffer.wait_for_frame(last_id)
                    if jpeg is None:
                        continue  # no frame received yet
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # viewer closed the tab

        def log_message(self, *args):
            pass

    return Handler


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__("camera_web_stream")

        self.declare_parameter("topic", "camera/image_raw")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("quality", 80)

        topic = self.get_parameter("topic").value
        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        self.quality = int(self.get_parameter("quality").value)

        self.bridge = CvBridge()
        self.buffer = FrameBuffer()
        self._warned_encoding = False

        # Sensor-data QoS (best effort) connects to both best-effort and reliable publishers.
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

        self.server = ThreadingHTTPServer((host, port), make_handler(self.buffer, topic))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

        self.get_logger().info(f"Subscribed to '{topic}'")
        self.get_logger().info(f"Streaming at http://localhost:{port}")

    def to_bgr8(self, msg):
        """Convert a ROS Image to an 8-bit BGR OpenCV image."""
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            pass

        # Fallback for encodings cv_bridge won't convert to bgr8 (e.g. 16UC1 / 32FC1 depth).
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if not self._warned_encoding:
            self.get_logger().warn(
                f"Encoding '{msg.encoding}' is not directly displayable; normalizing to 8-bit."
            )
            self._warned_encoding = True

        img = np.nan_to_num(img.astype(np.float32))
        img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    def on_image(self, msg):
        try:
            frame = self.to_bgr8(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Could not convert image: {e}", throttle_duration_sec=5.0)
            return

        # --- Place any OpenCV processing on `frame` here (overlays, resize, etc.) ---

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            self.buffer.put(buf.tobytes())

    def shutdown(self):
        self.server.shutdown()
        self.server.server_close()


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
