# Nodo mapeo (T3): llena la libreta del recorrido con los avisos del detector.
# La cuenta esta en libreta.py (Libreta); este nodo solo la conecta con ROS.
#
#   /navegacion/lugar (detector)        ──► Libreta.anotar()        ──┐
#   /navegacion/salida_tomada (/mover)  ──► Libreta.tomar_salida()  ──┼──► mapeo/libreta (JSON)
#   modo_actual (selector): en manual   ──► Libreta.pasar_a_manual() ─┘     (lo dibuja el panel)
#
# mapeo/libreta es un String con JSON, el formato que espera el panel
# (TICKETS_NAVEGACION.md, T11): {"lugares": [...], "actual": 1, "camino": []}.
# Se publica guardado (transient_local): quien se conecta tarde recibe la ultima.
# "camino" queda vacio hasta que exista /volver (Dijkstra).
#
# La orientacion para tomar_salida es la del aviso del lugar, no la de ese momento:
# /mover publica la salida antes de girar, pero asi no depende del orden.

import json
from dataclasses import asdict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.libreta import Libreta
from prometeo_navigation.modos import MANUAL


def libreta_a_json(libreta):
    """La libreta como texto JSON, en el formato que dibuja el panel."""
    return json.dumps({
        'lugares': [asdict(hoja) for hoja in libreta.lugares],
        'actual': libreta.actual.numero if libreta.actual is not None else None,
        'camino': [],
    })


class NodoMapeo(Node):

    def __init__(self):
        super().__init__('mapeo')
        margen_m = self.declare_parameter('margen_m', 0.30).value
        self.libreta = Libreta(margen_m)
        self.orientacion = None   # la del ultimo aviso
        self.en_manual = False

        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(AvisoLugar, '/navegacion/lugar', self.al_llegar, 10)
        self.create_subscription(String, '/navegacion/salida_tomada', self.al_salir, 10)
        self.create_subscription(String, 'modo_actual', self.al_cambiar_modo, guardado)
        self.pub_libreta = self.create_publisher(String, 'mapeo/libreta', guardado)
        self.publicar()
        self.get_logger().info(f'Mapeo listo (margen {margen_m} m)')

    def al_llegar(self, aviso):
        if self.en_manual:
            return
        self.orientacion = aviso.orientacion
        hoja, es_nuevo = self.libreta.anotar(
            aviso.tipo, aviso.x, aviso.y, aviso.orientacion, list(aviso.salidas))
        salidas = ', '.join(f'{s.direccion} {s.estado}' for s in hoja.salidas)
        self.get_logger().info(
            f'{"Lugar NUEVO" if es_nuevo else "Ya estuve en el"} {hoja.numero}: '
            f'{hoja.tipo} ({hoja.x:.2f}, {hoja.y:.2f}) [{salidas}]')
        self.publicar()

    def al_salir(self, mensaje):
        if self.orientacion is None:
            return
        camino = self.libreta.tomar_salida(self.orientacion, mensaje.data)
        if camino is None:
            self.get_logger().warning(f'Salida "{mensaje.data}" sin lugar en la libreta')
            return
        self.get_logger().info(
            f'Sale del {self.libreta.actual.numero} hacia el {camino.direccion}')
        self.publicar()

    def al_cambiar_modo(self, mensaje):
        en_manual = mensaje.data == MANUAL
        if en_manual and not self.en_manual:
            self.libreta.pasar_a_manual()
            self.get_logger().info('Manual: no anoto el tramo a mano')
            self.publicar()
        self.en_manual = en_manual

    def publicar(self):
        self.pub_libreta.publish(String(data=libreta_a_json(self.libreta)))


def main():
    rclpy.init()
    nodo = NodoMapeo()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
