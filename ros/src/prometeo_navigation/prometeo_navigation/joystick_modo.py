# Nodo que usa los botones del joystick para cambiar de modo y poner el freno (T11).
# Manejar con el joystick lo hace teleop_twist_joy (ver panel.launch.py); este nodo
# solo mira los botones de modo y le manda los pedidos al selector por "modo".
#
# Botones por defecto, joystick Xbox leido con game_controller_node (numeracion estandar):
#   B (1)               PARAR: poner el freno
#   Y (3), mantener 1 s soltar el freno
#   View / Back (4)     modo manual
#   Menu / Start (6)    modo automatico
# Si un boton no responde, mirar los numeros con:  ros2 topic echo /prometeo/joy
# y cambiarlos en my_robot_bringup/config/joystick_xbox.yaml.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from prometeo_navigation.botones_joystick import BotonesJoystick


class JoystickModo(Node):

    def __init__(self):
        super().__init__('joystick_modo')
        p = {nombre: self.declare_parameter(nombre, valor).value for nombre, valor in [
            ('boton_parar', 1), ('boton_soltar', 3), ('boton_manual', 4), ('boton_auto', 6),
            ('tiempo_soltar_s', 1.0)]}
        self.botones = BotonesJoystick(
            {p['boton_parar']: 'parar', p['boton_manual']: 'manual', p['boton_auto']: 'auto'},
            p['boton_soltar'], p['tiempo_soltar_s'])
        self.pub = self.create_publisher(String, 'modo', 10)
        self.create_subscription(Joy, 'joy', self.leer, 10)
        self.get_logger().info(
            f'Joystick: B({p["boton_parar"]}) parar, Y({p["boton_soltar"]}) mantener para '
            f'soltar, {p["boton_manual"]} manual, {p["boton_auto"]} auto')

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
