"""Convierte /distancias en un topic sensor_msgs/Range por cada ultrasonido.

/distancias trae los 3 sensores juntos en cm: [izq, der, frente]. Nav2 y otras
herramientas de ROS esperan Range: un mensaje por sensor, en metros, que dice ademas
donde esta montado y que tan ancho es su cono.

Funciona igual con el robot real y con la simulacion, porque los dos publican
/distancias con el mismo formato.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

# En el mismo orden que /distancias
SENSORES = ('izq', 'der', 'frente')

DIST_LEJOS_CM = 400.0   # ver micro_ros/main/sensores.h
CONO_RAD = math.radians(30.0)
RANGO_MIN_M = 0.02
RANGO_MAX_M = 4.0


def cm_a_rango(cm):
    """Devuelve el valor en metros, o None si el sensor no tiene dato.

    Por convencion de ROS (REP 117), +inf significa "no hay nada dentro del rango".
    """
    if cm < 0.0:
        return None
    if cm >= DIST_LEJOS_CM:
        return math.inf
    return cm / 100.0


class DistanciasARange(Node):

    def __init__(self):
        super().__init__('distancias_a_range')

        self.publicadores = {
            nombre: self.create_publisher(
                Range, f'/ultrasonido/{nombre}', qos_profile_sensor_data)
            for nombre in SENSORES
        }

        # Best effort: si no, no llega nada desde el ESP32
        self.create_subscription(
            Float32MultiArray, '/distancias', self.convertir, qos_profile_sensor_data)

    def convertir(self, mensaje):
        if len(mensaje.data) != len(SENSORES):
            self.get_logger().warning(
                f'/distancias: llegaron {len(mensaje.data)} valores', throttle_duration_sec=5.0)
            return

        ahora = self.get_clock().now().to_msg()
        for nombre, cm in zip(SENSORES, mensaje.data):
            rango = cm_a_rango(cm)
            if rango is None:
                continue

            salida = Range()
            salida.header.stamp = ahora
            salida.header.frame_id = f'ultrasonido_{nombre}_link'  # como en el URDF
            salida.radiation_type = Range.ULTRASOUND
            salida.field_of_view = CONO_RAD
            salida.min_range = RANGO_MIN_M
            salida.max_range = RANGO_MAX_M
            salida.range = rango
            self.publicadores[nombre].publish(salida)


def main():
    rclpy.init()
    nodo = DistanciasARange()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
