# Nodo que usa los botones del joystick para hablarle a la llave de modo.
# Manejar con el joystick lo hace teleop_twist_joy (ver panel.launch.py); este nodo
# solo mira dos botones y le manda los pedidos a la llave por "modo".
#
# Botones por defecto, joystick Xbox leido con game_controller_node (numeracion estandar):
#   B (1)                        PARAR: termina la corrida (para volver a empezar, relanzar)
#   Menu / Start (6), mantener 1 s  pasar a automatico (una sola vez, no tiene vuelta)
# Si un boton no responde, mirar los numeros con:  ros2 topic echo /prometeo/joy
# y cambiarlos en my_robot_bringup/config/joystick_xbox.yaml.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from prometeo_navigation.botones_joystick import BotonesJoystick
from prometeo_navigation.modos import PEDIR_AUTO, PEDIR_PARAR


class JoystickModo(Node):

    def __init__(self):
        super().__init__('joystick_modo')
        p = {nombre: self.declare_parameter(nombre, valor).value for nombre, valor in [
            ('boton_parar', 1), ('boton_auto', 6), ('tiempo_auto_s', 1.0)]}
        self.botones = BotonesJoystick(
            {p['boton_parar']: PEDIR_PARAR},
            p['boton_auto'], PEDIR_AUTO, p['tiempo_auto_s'])
        self.pub = self.create_publisher(String, 'modo', 10)
        self.create_subscription(Joy, 'joy', self.leer, 10)
        self.get_logger().info(
            f'Joystick: B({p["boton_parar"]}) PARAR, mantener {p["boton_auto"]} '
            f'{p["tiempo_auto_s"]} s para pasar a automatico')

    def leer(self, mensaje):
        ahora_s = self.get_clock().now().nanoseconds / 1e9
        for pedido in self.botones.actualizar(list(mensaje.buttons), ahora_s):
            self.get_logger().warn(f'Joystick pide: {pedido}')
            self.pub.publish(String(data=pedido))


def main():
    rclpy.init()
    nodo = JoystickModo()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
