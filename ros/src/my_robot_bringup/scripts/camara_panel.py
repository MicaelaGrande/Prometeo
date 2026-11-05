#!/usr/bin/env python3
"""Le pasa la imagen de la camara al panel de navegacion (T11). Funciona SIN INTERNET.

Escucha la imagen de vision, que ya trae dibujado lo que detecta (flechas, cubos,
zonas), y si vision no esta corriendo usa la imagen cruda de la camara. La achica y la
sirve como video MJPEG (una seguidilla de fotos JPEG) en:

    http://<ip-de-esta-compu>:8083/stream

El panel la muestra con un <img>, sin rosbridge: mandar fotos por rosbridge seria
muy pesado.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CamaraPanel(Node):

    def __init__(self):
        super().__init__('camara_panel')
        p = {nombre: self.declare_parameter(nombre, valor).value for nombre, valor in [
            ('topic_vision', '/vision/annotated'), ('topic_camara', '/camera/image_raw'),
            ('puerto', 8083), ('ancho', 480), ('fps', 6.0), ('calidad_jpeg', 70)]}
        self.p = p
        self.bridge = CvBridge()
        self.candado = threading.Lock()   # la imagen la escribe ROS y la lee el servidor web
        self.jpeg = None
        self.hora_vision = 0.0
        self.hora_ultima = 0.0

        # Las dos con "sensor data" (best effort) y con la normal: si no, segun quien
        # publique, alguna puede no llegar. Se procesan como mucho fps veces por segundo.
        for qos in (qos_profile_sensor_data, 10):
            self.create_subscription(Image, p['topic_vision'], self.imagen_vision, qos)
            self.create_subscription(Image, p['topic_camara'], self.imagen_camara, qos)

        servidor = ThreadingHTTPServer(('0.0.0.0', p['puerto']), self.manejador())
        servidor.daemon_threads = True
        threading.Thread(target=servidor.serve_forever, daemon=True).start()
        self.get_logger().info(f'Camara del panel en http://localhost:{p["puerto"]}/stream')

    def imagen_vision(self, mensaje):
        self.hora_vision = time.monotonic()
        self.guardar(mensaje)

    def imagen_camara(self, mensaje):
        if time.monotonic() - self.hora_vision < 1.0:
            return  # vision esta andando: mostrar la suya, que tiene los recuadros
        self.guardar(mensaje)

    def guardar(self, mensaje):
        ahora = time.monotonic()
        if ahora - self.hora_ultima < 1.0 / self.p['fps']:
            return
        self.hora_ultima = ahora
        try:
            foto = self.bridge.imgmsg_to_cv2(mensaje, desired_encoding='bgr8')
        except Exception as error:  # noqa: BLE001 (una imagen rara no tiene que tirar el nodo)
            self.get_logger().warn(f'Imagen que no se pudo leer: {error}', throttle_duration_sec=5.0)
            return
        alto = int(foto.shape[0] * self.p['ancho'] / foto.shape[1])
        foto = cv2.resize(foto, (self.p['ancho'], alto))
        ok, jpeg = cv2.imencode('.jpg', foto, [cv2.IMWRITE_JPEG_QUALITY, self.p['calidad_jpeg']])
        if ok:
            with self.candado:
                self.jpeg = jpeg.tobytes()

    def manejador(self):
        nodo = self

        class Manejador(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # sin una linea por cada foto

            def do_GET(self):
                if not self.path.startswith('/stream'):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=foto')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                enviado = None
                try:
                    while True:
                        with nodo.candado:
                            jpeg = nodo.jpeg
                        if jpeg is not None and jpeg is not enviado:
                            self.wfile.write(b'--foto\r\nContent-Type: image/jpeg\r\n')
                            self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                            self.wfile.write(jpeg + b'\r\n')
                            enviado = jpeg
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # se cerro la pagina

        return Manejador


def main():
    rclpy.init()
    nodo = CamaraPanel()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
