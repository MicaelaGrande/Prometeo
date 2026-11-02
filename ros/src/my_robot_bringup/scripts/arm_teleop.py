#!/usr/bin/env python3
"""Maneja el robot, el brazo y la camara con un joystick (topic /joy).

El boton Start cambia entre dos modos. Arranca en el modo manejo.

  Modo manejo
    Stick izquierdo           mueve el robot (adelante y atras, giro)
    D-pad izquierda y derecha gira la camara

  Modo brazo (el robot queda frenado)
    D-pad izquierda y derecha gira la base del brazo
    Stick izquierdo           sube y baja el hombro
    Stick derecho             sube y baja el codo
    Boton B                   abre la pinza
    Boton Y                   cierra la pinza

El nodo suma un poco al angulo de cada articulacion en cada ciclo, mientras se mantenga el
control apretado, y publica el angulo en radianes. Las articulaciones pasivas se calculan
aca para que el antebrazo y la pinza se mantengan paralelos al piso.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64

LIMITE_RAD = math.pi / 2  # mismo limite que el URDF
LIMITE_PINZA_RAD = 0.6    # apertura maxima del dedo izquierdo, igual que el URDF


def avanzar(angulo, entrada, velocidad, dt, limite=LIMITE_RAD, minimo=None):
    """Nuevo angulo despues de dt segundos, sin salir del rango."""
    minimo = -limite if minimo is None else minimo
    return max(minimo, min(limite, angulo + entrada * velocidad * dt))


class ArmTeleop(Node):

    def __init__(self):
        super().__init__('arm_teleop')

        # Numeros de eje y boton de un control Xbox con el driver de Linux
        self.eje_izq_x = self.declare_parameter('eje_izq_x', 0).value
        self.eje_izq_y = self.declare_parameter('eje_izq_y', 1).value
        self.eje_der_y = self.declare_parameter('eje_der_y', 4).value
        self.eje_dpad_x = self.declare_parameter('eje_dpad_x', 6).value
        self.boton_modo = self.declare_parameter('boton_modo', 7).value      # Start
        self.boton_abrir = self.declare_parameter('boton_abrir', 1).value    # B
        self.boton_cerrar = self.declare_parameter('boton_cerrar', 3).value  # Y

        # Velocidades con el control a fondo, en rad/s y en m/s
        self.vel_camara = self.declare_parameter('vel_camara', 1.5).value
        self.vel_base = self.declare_parameter('vel_base', 1.0).value
        self.vel_hombro = self.declare_parameter('vel_hombro', 0.8).value
        self.vel_codo = self.declare_parameter('vel_codo', 0.8).value
        self.vel_pinza = self.declare_parameter('vel_pinza', 1.0).value
        self.vel_lineal = self.declare_parameter('vel_lineal', 0.5).value
        self.vel_angular = self.declare_parameter('vel_angular', 0.5).value

        self.periodo = 0.02
        self.modo_brazo = False
        self.boton_modo_antes = False
        self.angulos = dict(camara=0.0, base=0.0, hombro=0.0, codo=0.0, pinza=0.0)
        self.entrada = dict(camara=0.0, base=0.0, hombro=0.0, codo=0.0, pinza=0.0,
                            lineal=0.0, angular=0.0)

        nombres = ('camera/pan', 'arm/base_yaw', 'arm/shoulder', 'arm/elbow',
                   'arm/elbow_compensation', 'arm/wrist', 'arm/gripper', 'arm/gripper_right')
        self.pubs = {n: self.create_publisher(Float64, f'/{n}/cmd_pos', 10) for n in nombres}
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.medidos = {}
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_timer(self.periodo, self.on_timer)
        self.get_logger().info('Modo manejo. Start cambia al modo brazo.')

    def on_joy(self, msg):
        def eje(i):
            return msg.axes[i] if 0 <= i < len(msg.axes) else 0.0

        def boton(i):
            return 1.0 if 0 <= i < len(msg.buttons) and msg.buttons[i] else 0.0

        # Cada pulsacion de Start cambia el modo
        apretado = bool(boton(self.boton_modo))
        if apretado and not self.boton_modo_antes:
            self.modo_brazo = not self.modo_brazo
            self.get_logger().info('Modo brazo.' if self.modo_brazo else 'Modo manejo.')
        self.boton_modo_antes = apretado

        # En /joy, izquierda y arriba son positivos, igual que el giro antihorario de la base
        # y de la camara. El hombro positivo inclina el brazo hacia adelante, y el codo
        # positivo baja el antebrazo, por eso el stick derecho va con el signo cambiado.
        dpad = eje(self.eje_dpad_x)
        self.entrada = dict(
            camara=0.0 if self.modo_brazo else dpad,
            base=dpad if self.modo_brazo else 0.0,
            hombro=eje(self.eje_izq_y) if self.modo_brazo else 0.0,
            codo=-eje(self.eje_der_y) if self.modo_brazo else 0.0,
            pinza=(boton(self.boton_abrir) - boton(self.boton_cerrar)) if self.modo_brazo else 0.0,
            lineal=0.0 if self.modo_brazo else eje(self.eje_izq_y),
            angular=0.0 if self.modo_brazo else eje(self.eje_izq_x),
        )

    def on_joint_states(self, msg):
        self.medidos = dict(zip(msg.name, msg.position))

    def pasivas(self):
        """Angulos de la compensacion y la muneca, calculados con los angulos medidos.

        El antebrazo queda paralelo al piso si hombro + compensacion + codo es igual al codo
        pedido, y la pinza si ademas la muneca cancela todo. Usar los angulos medidos, y no
        los pedidos, corrige el error de seguimiento del hombro y del codo.
        Mientras no llegan mediciones se usan los angulos pedidos."""
        a = self.angulos
        hombro = self.medidos.get('arm_shoulder_joint', a['hombro'])
        compensacion = -hombro
        medida = self.medidos.get('arm_elbow_compensation_joint', compensacion)
        codo = self.medidos.get('arm_elbow_joint', a['codo'])
        return compensacion, -(hombro + medida + codo)

    def on_timer(self):
        dt = self.periodo
        a, e = self.angulos, self.entrada
        a['camara'] = avanzar(a['camara'], e['camara'], self.vel_camara, dt)
        a['base'] = avanzar(a['base'], e['base'], self.vel_base, dt)
        a['hombro'] = avanzar(a['hombro'], e['hombro'], self.vel_hombro, dt)
        a['codo'] = avanzar(a['codo'], e['codo'], self.vel_codo, dt)
        a['pinza'] = avanzar(a['pinza'], e['pinza'], self.vel_pinza, dt,
                             limite=LIMITE_PINZA_RAD, minimo=0.0)

        compensacion, muneca = self.pasivas()
        comandos = {
            'camera/pan': a['camara'],
            'arm/base_yaw': a['base'],
            'arm/shoulder': a['hombro'],
            'arm/elbow': a['codo'],
            # Pasivas: mantienen el antebrazo y la pinza paralelos al piso
            'arm/elbow_compensation': compensacion,
            'arm/wrist': muneca,
            # El dedo derecho copia al izquierdo, al reves
            'arm/gripper': a['pinza'],
            'arm/gripper_right': -a['pinza'],
        }
        for nombre, valor in comandos.items():
            self.pubs[nombre].publish(Float64(data=valor))

        # En modo brazo igual se publica, con ceros, para que el robot frene
        vel = Twist()
        vel.linear.x = e['lineal'] * self.vel_lineal
        vel.angular.z = e['angular'] * self.vel_angular
        self.pub_vel.publish(vel)


def main():
    rclpy.init()
    nodo = ArmTeleop()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
