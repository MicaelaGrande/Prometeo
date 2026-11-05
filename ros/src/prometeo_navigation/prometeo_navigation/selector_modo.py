# Nodo selector de modo (T11): elige si a la ESP32 le llega la orden del automatico o
# la del manual.
#
#   Navegacion alta (automatico) ──► /cmd_vel_auto   ─┐
#                                                      ├─► este nodo ──► /cmd_vel ──► ESP32
#   Teclado o joystick (manual)  ──► /cmd_vel_manual ─┘        ▲
#                                                               │
#                                              /modo: "auto" o "manual"
#
# Publica /cmd_vel siempre, varias veces por segundo, aunque la orden no cambie.
# La ESP32 frena sola si pasan 300 ms sin ordenes (TIMEOUT_MS en
# micro_ros/drivers/motores/motores.c), y el teclado solo manda cuando se aprieta
# una tecla: sin este repetidor, el robot avanzaria a los tirones.
#
# Avisa el modo actual en /modo_actual. Lo guarda para quien se conecte tarde
# (durabilidad transient_local), asi Navegacion alta sabe el modo aunque arranque despues.
#
# Los topics no llevan "/" adelante a proposito, asi toman el namespace con que se corra:
#   - Gazebo:      ros2 run prometeo_navigation selector_modo
#                  -> /cmd_vel, /cmd_vel_manual, /modo
#   - Robot real con el firmware nuevo (nodo "body" en el namespace "prometeo"):
#                  ros2 run prometeo_navigation selector_modo --ros-args -r __ns:=/prometeo
#                  -> /prometeo/cmd_vel, /prometeo/cmd_vel_manual, /prometeo/modo
#
# Atiende un mensaje por vez (executor por defecto): no hay condiciones de carrera.

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from prometeo_navigation.modos import AUTO, MANUAL, Selector


class SelectorModo(Node):

    def __init__(self):
        super().__init__('selector_modo')

        # Cuanto puede tener una orden antes de considerarla vieja y parar.
        # Mas de 0,5 s porque al mantener una tecla apretada la compu tarda ese
        # tiempo en empezar a repetirla.
        edad_maxima_s = self.declare_parameter('edad_maxima_s', 0.6).value
        frecuencia_hz = self.declare_parameter('frecuencia_hz', 20.0).value
        self.selector = Selector(edad_maxima_s)

        self.create_subscription(Twist, 'cmd_vel_auto', self.orden_auto, 10)
        self.create_subscription(Twist, 'cmd_vel_manual', self.orden_manual, 10)
        self.create_subscription(String, 'modo', self.pedido_de_modo, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_modo = self.create_publisher(String, 'modo_actual', guardado)

        self.create_timer(1.0 / frecuencia_hz, self.publicar)
        self.avisar_modo()
        self.get_logger().info(
            f'Selector listo en modo {self.selector.modo}. '
            'Cambiar con: ros2 topic pub --once /modo std_msgs/msg/String "{data: auto}"')

    def ahora_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def orden_auto(self, mensaje):
        self.selector.guardar_orden(AUTO, mensaje, self.ahora_s())

    def orden_manual(self, mensaje):
        self.selector.guardar_orden(MANUAL, mensaje, self.ahora_s())

    def pedido_de_modo(self, mensaje):
        if self.selector.cambiar_modo(mensaje.data):
            self.get_logger().warn(f'MODO {self.selector.modo.upper()}')
            self.pub_cmd_vel.publish(Twist())  # parar en el momento del cambio
            self.avisar_modo()
        elif mensaje.data.strip().lower() not in (AUTO, MANUAL):
            self.get_logger().error(f'Modo desconocido: "{mensaje.data}". Usar auto o manual.')

    def avisar_modo(self):
        self.pub_modo.publish(String(data=self.selector.modo))

    def publicar(self):
        orden = self.selector.orden_a_publicar(self.ahora_s())
        self.pub_cmd_vel.publish(orden if orden is not None else Twist())  # Twist() = parar


def main():
    rclpy.init()
    nodo = SelectorModo()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Al cerrar con Ctrl+C, ROS ya se apago y no se puede publicar. No es un
        # problema: la ESP32 frena sola a los 300 ms sin ordenes.
        if rclpy.ok():
            nodo.pub_cmd_vel.publish(Twist())
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
