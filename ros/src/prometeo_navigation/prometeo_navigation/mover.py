# Nodo mover (T2): atiende la accion /mover y mueve el robot por el laberinto.
# Solo EJECUTA: quien decide hacia donde ir es Decision (con las flechas) o
# Exploracion (cuando no hay flecha). Las cuentas estan en maniobras.py.
#
#   Detector ──/navegacion/lugar──► Decision / Exploracion ──/mover──► este nodo ──► cmd_vel_auto
#      ▲                                                                   │
#      └──────────── /navegacion/reiniciar_detector ("ya sali") ──────────┘
#   /navegacion/salida_tomada (String): por que salida se fue, para Mapeo.
#
# Etapas:
#   AVANZANDO       va derecho por el pasillo (el centrado lo hace la asistencia / Bajo nivel)
#   ESPERANDO       el detector aviso un lugar: frena y espera una orden
#   RETROCEDIENDO   en una puerta: vuelve hasta el medio antes de girar
#   GIRANDO         gira hasta la orientacion de la salida, mirando /odom
# Al terminar de girar avisa "ya sali" al detector, contesta TERMINADA y vuelve a AVANZANDO.
#
# Publica en cmd_vel_auto: el selector solo lo deja pasar en automatico. En manual no
# publica, y si habia una orden en curso la cancela.
# En Gazebo hace falta el selector (panel.launch.py ns:=''), que es quien pasa
# cmd_vel_auto a cmd_vel.
#
# PROVISORIO: ACERCARSE (al cubo) todavia no esta hecho: contesta BLOQUEADA.
# Falta usar length_class para la velocidad (T2, "largo de la flecha").

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from std_msgs.msg import Empty, Float32MultiArray, String

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar
from prometeo_navigation.detector_lugares import orientacion_de
from prometeo_navigation.lugares import distancia_a_lectura
from prometeo_navigation.maniobras import (
    angulo_objetivo, es_puerta, frente_en_el_medio_de_la_puerta, normalizar_angulo,
    velocidad_de_giro)
from prometeo_navigation.modos import MANUAL

AVANZANDO = 'avanzando'
ESPERANDO = 'esperando'
RETROCEDIENDO = 'retrocediendo'
GIRANDO = 'girando'


class NodoMover(Node):

    def __init__(self):
        super().__init__('mover')
        p = {nombre: self.declare_parameter(nombre, valor).value for nombre, valor in [
            ('velocidad_avance', 0.12),      # m/s por el pasillo
            ('velocidad_retroceso', 0.08),   # m/s al volver al medio de una puerta
            ('distancia_parar_cm', 8.0),     # frente mas cerca que esto: no avanza
            ('ganancia_giro', 2.0),          # rad/s por radian que falta girar
            ('giro_min', 0.35),              # rad/s: menos que esto no mueve las ruedas
            ('giro_max', 0.9),               # rad/s
            ('tolerancia_giro_grados', 3.0),
            ('tiempo_max_giro_s', 10.0),     # si no llega, BLOQUEADA
            ('tiempo_max_retroceso_s', 6.0),
            ('frecuencia_hz', 20.0),
        ]}
        self.p = p
        self.tolerancia = math.radians(p['tolerancia_giro_grados'])

        self.etapa = AVANZANDO
        self.en_manual = False
        self.orientacion = None      # de /odom (None = todavia no llego)
        self.frente_cm = None        # de /distancias
        self.ultimo_lugar = None     # el ultimo aviso del detector
        self.objetivo = None         # GIRANDO: hacia donde tiene que quedar mirando
        self.frente_objetivo = None  # RETROCEDIENDO: lo que mide el frente en el medio
        self.desde = None            # cuando empezo a girar o retroceder (para el limite)
        self.pedido = None           # la orden en curso (goal handle), o None
        self.fin = None              # Future que se completa con el resultado

        self.create_subscription(Odometry, 'odom', self.al_moverse, 10)
        self.create_subscription(
            Float32MultiArray, 'distancias', self.al_medir, qos_profile_sensor_data)
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'modo_actual', self.al_cambiar_modo, guardado)
        self.create_subscription(Lugar, '/navegacion/lugar', self.al_llegar_a_lugar, 10)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.pub_reiniciar = self.create_publisher(Empty, '/navegacion/reiniciar_detector', 10)
        # Para Mapeo: por que salida se fue del lugar (la anota PROBADA en la libreta)
        self.pub_salida = self.create_publisher(String, '/navegacion/salida_tomada', 10)

        ActionServer(self, Mover, '/mover', self.ejecutar,
                     goal_callback=self.al_recibir_pedido,
                     cancel_callback=lambda _pedido: CancelResponse.ACCEPT)
        self.create_timer(1.0 / p['frecuencia_hz'], self.paso)
        self.get_logger().info(f'Mover listo (avanza a {p["velocidad_avance"]} m/s)')

    # ----------------------------- Lo que escucha -----------------------------

    def al_moverse(self, mensaje):
        self.orientacion = orientacion_de(mensaje.pose.pose.orientation)

    def al_medir(self, mensaje):
        if len(mensaje.data) == 3:
            self.frente_cm = distancia_a_lectura(mensaje.data[2])

    def al_cambiar_modo(self, mensaje):
        en_manual = mensaje.data == MANUAL
        if en_manual == self.en_manual:
            return
        self.en_manual = en_manual
        if en_manual and self.pedido is not None:
            self.terminar(Mover.Result.CANCELADA)
        # Al volver a automatico avanza; si quedo en un lugar, el detector (mirando
        # alrededor) lo avisa y frena.
        self.cambiar_etapa(AVANZANDO)

    def al_llegar_a_lugar(self, lugar):
        self.ultimo_lugar = lugar
        if self.etapa == AVANZANDO:
            self.cambiar_etapa(ESPERANDO)

    # ------------------------------ La accion ------------------------------

    def al_recibir_pedido(self, _pedido):
        if self.pedido is not None or self.en_manual:
            self.get_logger().warning('Pedido rechazado: ya hay una orden en curso o es manual')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def ejecutar(self, pedido):
        """Corre mientras dura la orden. paso() la va moviendo y llama a terminar()."""
        self.pedido = pedido
        self.fin = Future()
        self.empezar(pedido.request)
        resultado = await self.fin
        self.pedido = None
        if resultado == Mover.Result.TERMINADA:
            pedido.succeed()
        elif resultado == Mover.Result.CANCELADA and pedido.is_cancel_requested:
            pedido.canceled()
        else:
            pedido.abort()
        return Mover.Result(resultado=resultado)

    def empezar(self, orden):
        self.get_logger().info(f'Orden: {orden.accion} {orden.salida} {orden.length_class}')
        if orden.accion != Mover.Goal.IR_A_SALIDA:
            self.get_logger().warning(f'PROVISORIO: "{orden.accion}" todavia no esta hecho')
            self.terminar(Mover.Result.BLOQUEADA)
            return
        if self.orientacion is None:
            self.get_logger().error('Sin /odom: no puedo girar')
            self.terminar(Mover.Result.BLOQUEADA)
            return
        objetivo = angulo_objetivo(self.orientacion, orden.salida)
        if objetivo is None:
            self.get_logger().error(f'Salida desconocida: "{orden.salida}"')
            self.terminar(Mover.Result.BLOQUEADA)
            return
        self.pub_salida.publish(String(data=orden.salida))
        if orden.salida == Lugar.ADELANTE:
            self.salir_del_lugar()       # no hay que girar
            return
        self.objetivo = objetivo
        medio = frente_en_el_medio_de_la_puerta(
            self.ultimo_lugar.frente_inicial, self.ultimo_lugar.frente_final
        ) if es_puerta(self.ultimo_lugar) else None
        if medio is not None:
            self.frente_objetivo = medio
            self.cambiar_etapa(RETROCEDIENDO)
        else:
            self.cambiar_etapa(GIRANDO)

    def salir_del_lugar(self):
        """Termino la maniobra: avisa al detector, contesta TERMINADA y sigue avanzando."""
        self.pub_reiniciar.publish(Empty())
        self.ultimo_lugar = None
        self.cambiar_etapa(AVANZANDO)
        self.terminar(Mover.Result.TERMINADA)

    def terminar(self, resultado):
        if resultado != Mover.Result.TERMINADA:
            self.get_logger().warning(f'Orden terminada: {resultado}')
            self.cambiar_etapa(ESPERANDO)
        if self.fin is not None and not self.fin.done():
            self.fin.set_result(resultado)

    # --------------------------- Cada 1/20 de segundo ---------------------------

    def paso(self):
        if self.en_manual:
            return
        if self.pedido is not None and self.pedido.is_cancel_requested:
            self.terminar(Mover.Result.CANCELADA)

        orden = Twist()
        if self.etapa == AVANZANDO:
            cerca = self.frente_cm is not None and self.frente_cm <= self.p['distancia_parar_cm']
            orden.linear.x = 0.0 if cerca else self.p['velocidad_avance']
        elif self.etapa == RETROCEDIENDO:
            orden.linear.x = self.retroceder()
        elif self.etapa == GIRANDO:
            orden.angular.z = self.girar()
        self.pub_cmd.publish(orden)   # ESPERANDO: todo en 0

    def retroceder(self):
        if self.paso_el_tiempo(self.p['tiempo_max_retroceso_s']):
            self.get_logger().warning('No llegue al medio de la puerta: giro desde aca')
            self.cambiar_etapa(GIRANDO)
            return 0.0
        if self.frente_cm is None:
            return 0.0                    # sin dato: espera quieto
        if self.frente_cm >= self.frente_objetivo:
            self.cambiar_etapa(GIRANDO)   # llego al medio de la puerta
            return 0.0
        return -self.p['velocidad_retroceso']

    def girar(self):
        if self.paso_el_tiempo(self.p['tiempo_max_giro_s']):
            self.get_logger().error('El giro no termino a tiempo')
            self.terminar(Mover.Result.BLOQUEADA)
            return 0.0
        error = normalizar_angulo(self.objetivo - self.orientacion)
        velocidad = velocidad_de_giro(error, self.p['ganancia_giro'], self.p['giro_min'],
                                      self.p['giro_max'], self.tolerancia)
        if velocidad == 0.0:
            self.salir_del_lugar()
        return velocidad

    # ------------------------------- Ayudas -------------------------------

    def cambiar_etapa(self, etapa):
        if etapa != self.etapa:
            self.get_logger().info(f'Etapa: {etapa}')
        self.etapa = etapa
        self.desde = self.get_clock().now()

    def paso_el_tiempo(self, segundos):
        return (self.get_clock().now() - self.desde).nanoseconds / 1e9 > segundos


def main():
    rclpy.init()
    nodo = NodoMover()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
