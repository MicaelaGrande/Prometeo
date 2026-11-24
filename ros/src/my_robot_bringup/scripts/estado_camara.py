#!/usr/bin/env python3
"""Le pregunta a la ESP32-CAM como esta y se lo pasa al panel.

La ESP32-CAM es el hotspot (la red WiFi a la que se conectan la compu y la ESP32 del
cuerpo). Cada periodo_s lee su pagina de estado (firmware/esp32_cam, stream_server.c):

    http://192.168.4.1:81/status
    {"uptime_s":120,"ultimo_reinicio":"POWERON","camara":"ok",
     "camara_inicios_fallidos":0,"camara_fallos_seguidos":0,"clientes_wifi":2,"heap_libre":81234}

y lo publica en /camara/estado (String con JSON), agregando:
    "responde": true/false   si contesto (false = el hotspot no esta o esta colgado)
    "error": "..."           por que no contesto (solo si responde es false)
"""

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class EstadoCamara(Node):

    def __init__(self):
        super().__init__('estado_camara')
        self.url = self.declare_parameter('url', 'http://192.168.4.1:81/status').value
        self.espera_s = self.declare_parameter('espera_s', 1.5).value
        periodo_s = self.declare_parameter('periodo_s', 2.0).value
        self.pub = self.create_publisher(String, '/camara/estado', 10)
        self.respondia = None
        self.create_timer(periodo_s, self.preguntar)

    def preguntar(self):
        try:
            with urllib.request.urlopen(self.url, timeout=self.espera_s) as respuesta:
                estado = json.loads(respuesta.read().decode())
            estado['responde'] = True
        except (urllib.error.URLError, OSError, ValueError) as error:
            estado = {'responde': False, 'error': str(error)}
        if estado['responde'] != self.respondia:   # avisar solo cuando cambia
            if estado['responde']:
                self.get_logger().info(f'La ESP32-CAM responde en {self.url}')
            else:
                self.get_logger().warning(f'La ESP32-CAM no responde ({estado["error"]})')
            self.respondia = estado['responde']
        self.pub.publish(String(data=json.dumps(estado)))


def main():
    rclpy.init()
    nodo = EstadoCamara()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
