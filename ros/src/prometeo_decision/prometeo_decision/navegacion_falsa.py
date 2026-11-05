# Navegacion "de mentira", para probar Decision sin el robot.
#
# Atiende las acciones /mover y /volver como lo haria Navegacion, muestra
# lo que le pidieron y contesta siempre "terminada" (despues de una espera
# chiquita, como si se hubiera movido).
#
# Se corre con:
#   ros2 run prometeo_decision navegacion_falsa

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from prometeo_interfaces.action import Mover, Volver


class NavegacionFalsa(Node):

    def __init__(self):
        super().__init__('navegacion_falsa')
        # Cuanto "tarda" en moverse, en segundos.
        self.demora = self.declare_parameter('demora', 1.0).value
        ActionServer(self, Mover, '/mover', self.atender_mover)
        ActionServer(self, Volver, '/volver', self.atender_volver)
        self.get_logger().info('Navegacion falsa lista: atiendo /mover y /volver')

    def atender_mover(self, pedido):
        p = pedido.request
        self.get_logger().info(
            f'/mover me pidieron: accion={p.accion} salida={p.salida} largo={p.length_class}')
        time.sleep(self.demora)
        pedido.succeed()
        self.get_logger().info('/mover: terminada')
        return Mover.Result(resultado='terminada')

    def atender_volver(self, pedido):
        color = pedido.request.color or '(sin cubo)'
        self.get_logger().info(f'/volver me pidieron volver con: {color}')
        time.sleep(self.demora)
        pedido.succeed()
        self.get_logger().info('/volver: terminada')
        return Volver.Result(resultado='terminada')


def main():
    rclpy.init()
    nodo = NavegacionFalsa()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
