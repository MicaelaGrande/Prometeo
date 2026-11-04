"""PROVISORIO: imita en Gazebo lo que va a hacer la ESP32 del robot real (T9).

Borrar cuando la ESP32 se centre sola. Ver LEEME_BORRAR.md.

    T2 / teclado ──/cmd_vel──► este nodo ──/cmd_vel_corregido──► Gazebo

Hace dos cosas con la orden que llega:
- Freno: si hay una pared adelante, no deja avanzar. Girar y retroceder siempre pasan.
  Es una COPIA de micro_ros/main/seguridad.c.
- Centrado: si el robot avanza derecho, le suma un giro para quedar en el medio del pasillo.
  Si quien manda esta girando, no toca el giro: este nodo no decide hacia donde doblar.

Las cuentas estan en calculos.py; aca solo esta la parte de ROS.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from provisorio_bajo_nivel_sim.calculos import (
    FrenoEmergencia,
    debe_centrarse,
    giro_para_centrarse,
    umbral_freno_cm,
)

SENSORES = ('izq', 'der', 'frente')

# Valores por defecto. Los que valen son los de config/centrado_simulado.yaml.
PARAMETROS = {
    't_reaccion_s': 0.14,
    't_frenado_s': 0.08,
    'margen_cm': 1.0,
    'umbral_min_cm': 3.0,
    'histeresis_cm': 2.0,
    'lecturas_para_liberar': 3,
    'ganancia': 2.0,
    'giro_max_rad_s': 0.5,
    'giro_minimo_rad_s': 0.05,
    'distancia_salida_cm': 50.0,
    'tiempo_sin_orden_s': 0.5,
    'tiempo_sensor_viejo_s': 0.5,
    'frecuencia_hz': 20.0,
}


class CentradoSimulado(Node):

    def __init__(self):
        super().__init__('centrado_simulado')

        # Lee cada parametro del yaml. Si falta, usa el valor de PARAMETROS.
        self.p = {nombre: self.declare_parameter(nombre, defecto).value
                  for nombre, defecto in PARAMETROS.items()}

        self.freno = FrenoEmergencia(self.p['histeresis_cm'], self.p['lecturas_para_liberar'])
        self.freno_publicado = None  # para publicar /freno_emergencia solo cuando cambia

        self.orden = Twist()     # la ultima orden de /cmd_vel
        self.hora_orden = None   # cuando llego (None = nunca)
        self.lecturas = {nombre: (None, None) for nombre in SENSORES}  # (cm, hora)

        # ----- Lo que escucha -----
        self.create_subscription(Twist, '/cmd_vel', self.guardar_orden, 10)
        for nombre in SENSORES:
            # lambda: una funcion chiquita que le agrega el nombre del sensor
            self.create_subscription(
                Range, f'/ultrasonido/{nombre}',
                lambda mensaje, nombre=nombre: self.guardar_lectura(nombre, mensaje),
                qos_profile_sensor_data)
        self.create_subscription(Bool, '/seguridad_habilitada', self.habilitar_freno, 10)

        # ----- Lo que publica -----
        self.pub_orden = self.create_publisher(Twist, '/cmd_vel_corregido', 10)
        self.pub_freno = self.create_publisher(Bool, '/freno_emergencia', 10)

        self.create_timer(1.0 / self.p['frecuencia_hz'], self.publicar)
        self.get_logger().warning('PROVISORIO: imitando el centrado y el freno de la ESP32')

    # ===== Lo que llega =====

    def guardar_orden(self, mensaje):
        self.orden = mensaje
        self.hora_orden = self.ahora()

    def guardar_lectura(self, nombre, mensaje):
        # Range viene en metros; +inf significa "no hay nada en el rango" (lejos)
        cm = mensaje.range * 100.0 if math.isfinite(mensaje.range) else math.inf
        self.lecturas[nombre] = (cm, self.ahora())

        if nombre == 'frente':
            # Como en la ESP32: el freno se actualiza con cada medicion nueva del frente
            avance = self.avance_pedido()
            self.freno.actualizar(avance, cm, self.umbral(avance))
            self.publicar_freno_si_cambio()

    def habilitar_freno(self, mensaje):
        # False = modo recoger: deja acercarse a un objeto
        self.freno.habilitado = mensaje.data
        if not mensaje.data:
            self.freno.activo = False
        self.publicar_freno_si_cambio()

    # ===== Lo que sale =====

    def publicar(self):
        salida = Twist()  # arranca en cero: si no hay orden, el robot se para

        if self.orden_vigente():
            avance = self.orden.linear.x
            giro = self.orden.angular.z
            frente = self.lectura('frente')

            # Si el freno estaba suelto, revisa con la orden nueva (seguridad_filtrar en la ESP32)
            if not self.freno.activo:
                self.freno.actualizar(avance, frente, self.umbral(avance))
                self.publicar_freno_si_cambio()

            # El freno solo bloquea el avance: girar y retroceder siempre pasan
            if self.freno.activo and avance > 0.0:
                avance = 0.0

            if debe_centrarse(avance, giro, self.p['giro_minimo_rad_s']):
                giro += giro_para_centrarse(
                    self.lectura('izq'), self.lectura('der'), self.p['ganancia'],
                    self.p['giro_max_rad_s'], self.p['distancia_salida_cm'])

            salida.linear.x = avance
            salida.angular.z = giro

        self.pub_orden.publish(salida)

    def publicar_freno_si_cambio(self):
        if self.freno.activo != self.freno_publicado:
            self.pub_freno.publish(Bool(data=self.freno.activo))
            self.freno_publicado = self.freno.activo

    # ===== Ayudas =====

    def ahora(self):
        return self.get_clock().now().nanoseconds / 1e9  # en segundos

    def orden_vigente(self):
        return (self.hora_orden is not None
                and self.ahora() - self.hora_orden < self.p['tiempo_sin_orden_s'])

    def avance_pedido(self):
        return self.orden.linear.x if self.orden_vigente() else 0.0

    def lectura(self, nombre):
        """Ultima lectura en cm, o None si no hay o es muy vieja."""
        cm, hora = self.lecturas[nombre]
        if hora is None or self.ahora() - hora > self.p['tiempo_sensor_viejo_s']:
            return None
        return cm

    def umbral(self, avance):
        return umbral_freno_cm(avance, self.p['t_reaccion_s'], self.p['t_frenado_s'],
                               self.p['margen_cm'], self.p['umbral_min_cm'])


def main():
    rclpy.init()
    nodo = CentradoSimulado()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
