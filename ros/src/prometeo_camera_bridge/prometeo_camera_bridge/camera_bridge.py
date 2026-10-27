#!/usr/bin/env python3
"""Bridge ESP32-CAM HTTP/MJPEG to ROS 2 sensor_msgs/msg/Image (bgr8).

Run with:
    ros2 run prometeo_camera_bridge camera_bridge --ros-args \
      -p stream_url:=http://192.168.1.20/stream
"""

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        self.declare_parameter('stream_url', 'http://192.168.1.20/stream')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('retry_seconds', 2.0)

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.bridge = CvBridge()
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._receive_loop, daemon=True)
        self.worker.start()
        self.get_logger().info('Publicando sensor_msgs/msg/Image en /camera/image_raw')

    def _receive_loop(self):
        url = str(self.get_parameter('stream_url').value)
        frame_id = str(self.get_parameter('frame_id').value)
        max_fps = max(0.1, float(self.get_parameter('max_fps').value))
        retry_seconds = max(0.1, float(self.get_parameter('retry_seconds').value))
        min_interval = 1.0 / max_fps
        last_published = 0.0

        while not self.stop_event.is_set():
            self.get_logger().info(f'Conectando a {url}')
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.get_logger().warning('No se pudo abrir el stream. Reintentando...')
                cap.release()
                self.stop_event.wait(retry_seconds)
                continue

            self.get_logger().info('Stream conectado')
            try:
                while not self.stop_event.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        self.get_logger().warning('Se perdio el stream; reconectando...')
                        break

                    now = time.monotonic()
                    if now - last_published < min_interval:
                        continue
                    last_published = now

                    if frame.ndim != 3 or frame.shape[2] != 3:
                        self.get_logger().warning('Frame con formato inesperado; omitido')
                        continue

                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    # Hora de recepcion en Ubuntu, no la hora exacta de captura en la ESP32.
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = frame_id
                    self.publisher.publish(msg)
            except Exception as exc:
                self.get_logger().error(f'Error recibiendo/publicando: {exc}')
            finally:
                cap.release()
            self.stop_event.wait(retry_seconds)

    def stop(self):
        self.stop_event.set()
        self.worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
