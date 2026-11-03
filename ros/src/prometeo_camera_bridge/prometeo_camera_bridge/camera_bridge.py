
#!/usr/bin/env python3
"""Puente ESP32-CAM HTTP/MJPEG a ROS 2.

Publica:
    /camera/image_raw   (sensor_msgs/msg/Image)
    /camera/camera_info (sensor_msgs/msg/CameraInfo, si hay calibracion)

La imagen se publica sin rectificar.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        self.declare_parameter(
            'stream_url', 'http://172.16.0.50/stream'
        )
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('retry_seconds', 2.0)
        self.declare_parameter('calibration_file', '')

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            Image, '/camera/image_raw', qos
        )

        self.info_publisher = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos
        )

        self.bridge = CvBridge()
        self.calibration = None

        calibration_file = str(
            self.get_parameter('calibration_file').value
        )

        if calibration_file:
            with np.load(calibration_file) as data:
                self.calibration = {
                    'K': data['camera_matrix'].reshape(3, 3).copy(),
                    'D': data['dist_coeffs'].reshape(-1).copy(),
                    'width': int(data['image_width']),
                    'height': int(data['image_height']),
                }

            self.get_logger().info(
                f'Calibracion cargada: {calibration_file}'
            )
        else:
            self.get_logger().warning(
                'No se configuro calibration_file; '
                'CameraInfo no se publicara'
            )

        self.stop_event = threading.Event()
        self.worker = threading.Thread(
            target=self._receive_loop,
            daemon=True
        )
        self.worker.start()

        self.get_logger().info(
            'Publicando imagenes en /camera/image_raw'
        )

    def _receive_loop(self):
        url = str(self.get_parameter('stream_url').value)
        frame_id = str(self.get_parameter('frame_id').value)
        max_fps = max(
            0.1, float(self.get_parameter('max_fps').value)
        )
        retry_seconds = max(
            0.1, float(self.get_parameter('retry_seconds').value)
        )

        min_interval = 1.0 / max_fps
        last_published = 0.0

        while not self.stop_event.is_set():
            self.get_logger().info(f'Conectando a {url}')

            cap = cv2.VideoCapture(
                url,
                cv2.CAP_FFMPEG,
                [
                    cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                    cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000,
                ]
            )

            if not cap.isOpened():
                self.get_logger().warning(
                    'No se pudo abrir el stream. Reintentando...'
                )
                cap.release()
                self.stop_event.wait(retry_seconds)
                continue

            self.get_logger().info('Stream conectado')

            try:
                while not self.stop_event.is_set():
                    ok, frame = cap.read()

                    if self.stop_event.is_set():
                        break

                    if not ok or frame is None:
                        self.get_logger().warning(
                            'Se perdio el stream; reconectando...'
                        )
                        break

                    now = time.monotonic()

                    if now - last_published < min_interval:
                        continue

                    if frame.ndim != 3 or frame.shape[2] != 3:
                        self.get_logger().warning(
                            'Frame con formato inesperado; omitido'
                        )
                        continue

                    height, width = frame.shape[:2]

                    if self.calibration is not None:
                        cal = self.calibration
                        if (
                            width != cal['width']
                            or height != cal['height']
                        ):
                            self.get_logger().warning(
                                'Resolucion distinta de la calibrada; '
                                'CameraInfo omitido'
                            )
                            continue

                    msg = self.bridge.cv2_to_imgmsg(
                        frame, encoding='bgr8'
                    )

                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = frame_id

                    if self.stop_event.is_set() or not rclpy.ok():
                        break

                    self.publisher.publish(msg)
                    last_published = now

                    if self.calibration is not None:
                        cal = self.calibration
                        K = cal['K']

                        info = CameraInfo()
                        info.header = msg.header
                        info.width = width
                        info.height = height
                        info.distortion_model = 'plumb_bob'
                        info.d = cal['D'].tolist()
                        info.k = K.reshape(-1).tolist()

                        info.r = [
                            1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0,
                        ]

                        info.p = [
                            float(K[0, 0]), 0.0,
                            float(K[0, 2]), 0.0,
                            0.0, float(K[1, 1]),
                            float(K[1, 2]), 0.0,
                            0.0, 0.0, 1.0, 0.0,
                        ]

                        if self.stop_event.is_set() or not rclpy.ok():
                            break

                        self.info_publisher.publish(info)

            except Exception as exc:
                self.get_logger().error(
                    f'Error recibiendo/publicando: {exc}'
                )
            finally:
                cap.release()

            self.stop_event.wait(retry_seconds)

    def stop(self):
        self.stop_event.set()
        self.worker.join()


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
