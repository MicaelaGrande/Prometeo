#!/usr/bin/env python3
"""Reemplaza al ESP32: convierte los rayos de Gazebo en el topic /distancias.

Mide por turnos, igual que sensores_task() en sensores.c:
    Turno 1: izquierda y derecha juntos.
    Turno 2: frente, 60 ms despues.

Cada dos turnos publica [izq, der, frente] en cm, con -1 si un sensor no tiene dato
y 400 si no ve nada. Quien escucha /distancias recibe lo mismo que del robot real.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

import modelo_hcsr04 as modelo

SENSORES = ('izq', 'der', 'frente')
TURNOS = (('izq', 'der'), ('frente',))
PERIODO_TURNO_S = 0.06  # HCSR04_MIN_CYCLE_US en hcsr04.c


class Hcsr04Sim(Node):

    def __init__(self):
        super().__init__('hcsr04_sim')

        semilla = self.declare_parameter('semilla', 42).value
        self.generador = np.random.default_rng(semilla)

        comunes = {
            'angulo_max_rebote_deg': self.declare_parameter('angulo_max_rebote_deg', 20.0).value,
            'ruido_base_cm': self.declare_parameter('ruido_base_cm', 0.3).value,
            'ruido_proporcional': self.declare_parameter('ruido_proporcional', 0.002).value,
            'prob_eco_perdido': self.declare_parameter('prob_eco_perdido', 0.01).value,
            'temperatura_c': self.declare_parameter('temperatura_c', 20.0).value,
        }
        self.parametros = {
            nombre: modelo.Parametros(
                sesgo_cm=self.declare_parameter(f'sesgo_{nombre}_cm', 0.0).value, **comunes)
            for nombre in SENSORES
        }

        self.rayos = {nombre: None for nombre in SENSORES}
        self.medianas = {nombre: modelo.Mediana() for nombre in SENSORES}
        self.distancias = {nombre: modelo.DIST_SIN_DATO for nombre in SENSORES}
        self.turno = 0

        for nombre in SENSORES:
            self.create_subscription(
                LaserScan, f'/ultrasonido/{nombre}/rayos',
                lambda mensaje, nombre=nombre: self.guardar_rayos(nombre, mensaje),
                qos_profile_sensor_data)

        # Best effort, como en el ESP32: si se pierde un mensaje, el siguiente ya trae
        # el dato nuevo. Quien escuche /distancias tiene que usar el mismo QoS.
        self.publicador = self.create_publisher(
            Float32MultiArray, '/distancias', qos_profile_sensor_data)

        # Este temporizador sigue el reloj de Gazebo si use_sim_time esta activo
        self.create_timer(PERIODO_TURNO_S, self.medir_turno)

    def guardar_rayos(self, nombre, mensaje):
        self.rayos[nombre] = mensaje

    def medir_turno(self):
        for nombre in TURNOS[self.turno]:
            self.medir(nombre)

        self.turno = (self.turno + 1) % len(TURNOS)
        if self.turno == 0:
            mensaje = Float32MultiArray()
            mensaje.data = [float(self.distancias[nombre]) for nombre in SENSORES]
            self.publicador.publish(mensaje)

    def medir(self, nombre):
        mensaje = self.rayos[nombre]
        if mensaje is None:
            return

        angulos = mensaje.angle_min + np.arange(len(mensaje.ranges)) * mensaje.angle_increment
        cm = modelo.lectura_cm(mensaje.ranges, angulos, self.parametros[nombre], self.generador)
        self.distancias[nombre] = self.medianas[nombre].agregar(cm)


def main():
    rclpy.init()
    nodo = Hcsr04Sim()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
