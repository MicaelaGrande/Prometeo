#!/usr/bin/env python3
"""Prueba de motores: adelante, atrás, izquierda y derecha con velocidad creciente.

El tiempo total (max_time) se reparte en partes iguales entre las 4 direcciones.
En cada una la velocidad sube linealmente de 0 a max_speed, y al final hay una
pausa corta en 0 antes de pasar a la siguiente dirección.

Publica geometry_msgs/Twist en /prometeo/cmd_vel en unidades reales (como Gazebo):
  adelante/atrás -> linear.x en m/s (+ adelante)
  izquierda/derecha (giro en el lugar) -> angular.z en rad/s (+ izquierda)
max_speed es una FRACCIÓN (0..1) de la velocidad máxima: 1.0 = v_max m/s en línea
recta, o el giro en el lugar que deja un lado a +v_max y el otro a -v_max.

Se publica a 20 Hz sin parar: si dejara de publicar, el failsafe del ESP32 frena.
Ctrl-C o cualquier error termina enviando 0.

Uso:
  ros2 run my_robot_bringup motor_test.py --ros-args -p max_speed:=0.6 -p max_time:=20.0
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (nombre, componente, signo)
FASES = [
    ("ADELANTE", "linear_x", +1.0),
    ("ATRAS", "linear_x", -1.0),
    ("IZQUIERDA", "angular_z", +1.0),
    ("DERECHA", "angular_z", -1.0),
]
PAUSA_S = 0.5          # parada entre fases (cuenta dentro del tiempo de cada fase)
RATE_HZ = 20.0


class MotorTest(Node):
    def __init__(self):
        super().__init__("motor_test")
        self.declare_parameter("max_speed", 0.5)     # fracción 0..1 de la velocidad máxima
        # Deben coincidir con menuconfig: Prometeo - Cinematica
        self.declare_parameter("v_max", 0.70)             # m/s con motores al 100%
        self.declare_parameter("wheel_separation", 0.161) # m
        self.declare_parameter("turn_factor", 1.0)        # PROMETEO_TURN_FACTOR_PCT / 100
        self.declare_parameter("max_time", 20.0)     # segundos, total de la rutina
        self.declare_parameter("topic", "/prometeo/cmd_vel")

        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_time = float(self.get_parameter("max_time").value)
        v_max = float(self.get_parameter("v_max").value)
        ancho = float(self.get_parameter("wheel_separation").value) * float(self.get_parameter("turn_factor").value)
        # Valor de cada componente a fracción 1.0
        self.escala = {"linear_x": v_max, "angular_z": 2.0 * v_max / ancho}
        topic = self.get_parameter("topic").value

        if not 0.0 < self.max_speed <= 1.0:
            raise ValueError(f"max_speed debe estar en (0, 1], llegó {self.max_speed}")
        if self.max_time < len(FASES) * 2 * PAUSA_S:
            raise ValueError(f"max_time demasiado corto (mínimo {len(FASES) * 2 * PAUSA_S:.1f} s)")

        self.pub = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(
            f"Publicando en {topic}: max_speed={self.max_speed}, max_time={self.max_time}s "
            f"({self.max_time / len(FASES):.1f}s por dirección)")

    def enviar(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

    def parar(self):
        for _ in range(5):   # varios por si se pierde un paquete (best effort)
            self.enviar()
            time.sleep(0.02)

    def run(self):
        t_fase = self.max_time / len(FASES)
        t_rampa = t_fase - PAUSA_S
        periodo = 1.0 / RATE_HZ

        # Espera a que el ESP32 se suscriba, si no los primeros comandos se pierden
        t0 = time.monotonic()
        while rclpy.ok() and self.pub.get_subscription_count() == 0:
            if time.monotonic() - t0 > 10.0:
                self.get_logger().warn("Nadie suscrito a cmd_vel (¿agente/ESP32 conectado?). Sigo igual.")
                break
            time.sleep(0.1)

        for nombre, campo, signo in FASES:
            self.get_logger().info(f"{nombre}: 0 -> {self.max_speed:.2f}")
            inicio = time.monotonic()
            while rclpy.ok():
                t = time.monotonic() - inicio
                if t >= t_fase:
                    break
                frac = self.max_speed * min(t / t_rampa, 1.0) if t < t_rampa else 0.0
                self.enviar(**{campo: signo * frac * self.escala[campo]})
                time.sleep(periodo)
            self.enviar()
        self.parar()
        self.get_logger().info("Prueba terminada")


def main():
    rclpy.init()
    nodo = None
    try:
        nodo = MotorTest()
        nodo.run()
    except KeyboardInterrupt:
        pass
    except ValueError as e:
        print(f"Parámetros inválidos: {e}")
    finally:
        if nodo is not None:
            nodo.parar()
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
