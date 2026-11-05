# Nodo selector de modo (T11): elige que orden de movimiento le llega a la ESP32.
#
#   Navegacion alta (automatico) ──► cmd_vel_auto   ─┐
#   Panel o teclado (compu)      ──► cmd_vel_manual ─┼─► este nodo ──► cmd_vel ──► ESP32
#   Joystick                     ──► cmd_vel_joy    ─┘        ▲
#                                                              │
#                         modo: "auto", "manual", "parar" (freno) o "soltar"
#
# El freno PARAR queda puesto hasta que una persona lo suelta, y no cambia el modo.
#
# Publica cmd_vel siempre, varias veces por segundo, aunque la orden no cambie.
# La ESP32 frena sola si pasan 300 ms sin ordenes (TIMEOUT_MS en
# micro_ros/drivers/motores/motores.c), y el teclado solo manda cuando se aprieta
# una tecla: sin este repetidor, el robot avanzaria a los tirones.
#
# Lo que avisa, para los demas nodos y el panel. Los dos primeros se guardan para quien
# se conecte tarde (durabilidad transient_local) y se repiten una vez por segundo:
#   modo_actual      (String) "auto" o "manual"   <- la garra, Decision y T2 escuchan esto
#   freno_operador   (Bool)   True si el freno PARAR esta puesto
#   estado_selector  (String) JSON {"modo", "freno", "fuente"} para el panel
#   orden_enviada    (Twist)  copia de lo que sale por cmd_vel, para el panel
#
# Los topics no llevan "/" adelante a proposito, asi toman el namespace con que se corra:
#   - Gazebo:      ros2 run prometeo_navigation selector_modo
#                  -> /cmd_vel, /cmd_vel_manual, /modo
#   - Robot real con el firmware nuevo (nodo "body" en el namespace "prometeo"):
#                  ros2 run prometeo_navigation selector_modo --ros-args -r __ns:=/prometeo
#                  -> /prometeo/cmd_vel, /prometeo/cmd_vel_manual, /prometeo/modo
#
# Atiende un mensaje por vez (executor por defecto): no hay condiciones de carrera.

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from prometeo_navigation.modos import AUTO, COMPU, JOYSTICK, MODOS, PARAR, SOLTAR, Selector


class SelectorModo(Node):

    def __init__(self):
        super().__init__('selector_modo')

        # Cuanto puede tener una orden antes de considerarla vieja y parar.
        # Mas de 0,5 s porque al mantener una tecla apretada la compu tarda ese
        # tiempo en empezar a repetirla.
        edad_maxima_s = self.declare_parameter('edad_maxima_s', 0.6).value
        frecuencia_hz = self.declare_parameter('frecuencia_hz', 20.0).value
        self.selector = Selector(edad_maxima_s)
        self.fuente_avisada = None

        self.create_subscription(Twist, 'cmd_vel_auto', self.orden_de(AUTO), 10)
        self.create_subscription(Twist, 'cmd_vel_manual', self.orden_de(COMPU), 10)
        self.create_subscription(Twist, 'cmd_vel_joy', self.orden_de(JOYSTICK), 10)
        self.create_subscription(String, 'modo', self.pedido, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # Copia de lo que sale por cmd_vel, para el panel. El panel no puede tener
        # permiso sobre cmd_vel: en rosbridge leer y escribir van juntos, y podria
        # mandarle ordenes a la ESP32 sin pasar por el selector.
        self.pub_copia = self.create_publisher(Twist, 'orden_enviada', 10)
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_modo = self.create_publisher(String, 'modo_actual', guardado)
        self.pub_freno = self.create_publisher(Bool, 'freno_operador', guardado)
        self.pub_estado = self.create_publisher(String, 'estado_selector', 10)

        self.create_timer(1.0 / frecuencia_hz, self.publicar)
        self.create_timer(1.0, self.avisar)  # una vez por segundo, para quien llegue tarde
        self.avisar()
        self.get_logger().info(
            f'Selector listo en modo {self.selector.modo}. '
            'Cambiar con: ros2 topic pub --once modo std_msgs/msg/String "{data: auto}"')

    def ahora_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def orden_de(self, origen):
        """Devuelve la funcion que guarda las ordenes de ese origen."""
        def guardar(mensaje):
            self.selector.guardar_orden(origen, mensaje, self.ahora_s())
        return guardar

    def pedido(self, mensaje):
        texto = mensaje.data.strip().lower()
        if self.selector.pedir(texto):
            if texto == PARAR:
                self.get_logger().warn('FRENO PUESTO')
            elif texto == SOLTAR:
                self.get_logger().warn(f'Freno suelto, sigue en {self.selector.modo.upper()}')
            else:
                self.get_logger().warn(f'MODO {self.selector.modo.upper()}')
            self.pub_cmd_vel.publish(Twist())  # parar en el momento del cambio
            self.avisar()
        elif texto not in MODOS + (PARAR, SOLTAR):
            self.get_logger().error(
                f'Pedido desconocido: "{mensaje.data}". Usar auto, manual, parar o soltar.')

    def avisar(self):
        fuente = self.selector.fuente(self.ahora_s())
        self.fuente_avisada = fuente
        self.pub_modo.publish(String(data=self.selector.modo))
        self.pub_freno.publish(Bool(data=self.selector.freno_puesto))
        self.pub_estado.publish(String(data=json.dumps({
            'modo': self.selector.modo,
            'freno': self.selector.freno_puesto,
            'fuente': fuente,
        })))

    def publicar(self):
        ahora_s = self.ahora_s()
        orden = self.selector.orden_a_publicar(ahora_s)
        orden = orden if orden is not None else Twist()  # Twist() = parar
        self.pub_cmd_vel.publish(orden)
        self.pub_copia.publish(orden)
        if self.selector.fuente(ahora_s) != self.fuente_avisada:
            self.avisar()  # cambio quien maneja: avisar enseguida, no esperar al segundo


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
