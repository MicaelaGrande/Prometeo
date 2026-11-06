# Nodo mover (T2): atiende la accion /mover y mueve el robot por el laberinto.
# Solo EJECUTA: quien decide hacia donde ir es Decision (con las flechas) o
# Exploracion (cuando no hay flecha). Las cuentas estan en maniobras.py.
#
#   Detector ──/navegacion/lugar──► Decision / Exploracion ──/mover──► este nodo ──► cmd_vel_auto
#      ▲                                                                   │
#      └──────────── /navegacion/reiniciar_detector ("ya sali") ──────────┘
#   /navegacion/salida_tomada (String): por que salida se fue, para Mapeo.
#   odom_laberinto (Odometry): la posicion de laberinto (maniobras.PosicionLaberinto),
#       con los giros confirmados y el tiempo avanzando. El detector la usa en vez de
#       /odom (autonomia.launch.py lo remapea), porque las ruedas patinan.
#   /navegacion/poner_posicion (Odometry): la corrige (Mapeo, cuando el operador dice
#       en que lugar dejo el robot despues de manejarlo a mano).
#
# Etapas:
#   AVANZANDO       va derecho por el pasillo (el centrado lo hace la asistencia / Bajo nivel)
#   ESPERANDO       el detector aviso un lugar: frena y espera una orden
#   RETROCEDIENDO   en una puerta: vuelve hasta el medio antes de girar
#   GIRANDO         gira hacia la salida y lo confirma con los ultrasonidos (quieto)
# Al terminar de girar avisa "ya sali" al detector, contesta TERMINADA y vuelve a AVANZANDO.
#
# El giro (las ruedas patinan mucho y los encoders no son confiables):
#   1. Gira lo que cree que es el angulo pedido. Cuanto giro lo saca de /odom si llega;
#      si no, de la velocidad que mando por el tiempo (por factor_giro_tiempo).
#   2. FRENA y mira con el robot quieto (girando, una pared en diagonal no devuelve
#      bien el eco y el sensor marca cualquier cosa). Necesita lecturas_confirmar
#      lecturas seguidas que den bien (maniobras.giro_confirmado):
#        - esquina: la pared que tenia adelante ahora mide lo mismo desde el costado
#          (dobla a la izquierda: el sensor derecho), +- tolerancia_costado_cm;
#        - media vuelta: adelante tiene que estar el pasillo (frente_abierto_cm);
#        - puerta: antes no habia pared adelante, no hay con que comparar: confia en 1.
#   3. Si no da bien (giro de menos, lo tipico al patinar), gira paso_grados mas y
#      vuelve a mirar, hasta giro_extra_max_grados.
#   4. Si nunca da bien, el robot no esta girando: BLOQUEADA y pide AYUDA en el panel
#      (navegacion/aviso).
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
from std_msgs.msg import Bool, Empty, Float32MultiArray, String

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar
from prometeo_navigation.detector_lugares import orientacion_de
from prometeo_navigation.lugares import distancia_a_lectura
from prometeo_navigation.maniobras import (
    SIN_CONFIRMAR, PosicionLaberinto, como_confirmar, es_puerta, frente_en_el_medio_de_la_puerta,
    giro_confirmado, giro_de_salida, normalizar_angulo, velocidad_de_giro)
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
            # Giro confirmado con el ultrasonido de adelante
            ('frente_abierto_cm', 40.0),     # adelante mide esto o mas = pasillo
            ('tolerancia_costado_cm', 8.0),  # esquina: costado vs. lo que media el frente
            ('lecturas_confirmar', 3),       # lecturas quieto que tienen que dar bien
            ('espera_quieto_s', 0.3),        # despues de frenar, antes de mirar
            ('paso_grados', 10.0),           # cuanto mas gira si no confirmo
            ('giro_extra_max_grados', 45.0),  # el maximo de pasos de mas
            # Sin /odom: cuanto gira de verdad por cada rad/s que se manda (calibrar:
            # mandar un giro de 360 y ver cuanto giro). 1 = sin patinar.
            ('factor_giro_tiempo', 1.0),
            ('edad_max_odom_s', 0.5),        # /odom mas viejo que esto = no hay
            ('tiempo_max_retroceso_s', 6.0),
            ('frecuencia_hz', 20.0),
        ]}
        self.p = p
        self.tolerancia = math.radians(p['tolerancia_giro_grados'])

        self.etapa = AVANZANDO
        self.en_manual = False
        self.orientacion = None      # de /odom (None = todavia no llego)
        self.hora_odom = None        # cuando llego el ultimo /odom
        self.frente_cm = None        # de /distancias
        self.izquierda_cm = None
        self.derecha_cm = None
        self.ultimo_lugar = None     # el ultimo aviso del detector
        self.giro_pedido = 0.0       # GIRANDO: cuanto tiene que girar (rad, + izquierda)
        self.girado = 0.0            # cuanto lleva girado
        self.mirando = False         # quieto, juntando lecturas para confirmar
        self.mirar_desde = None
        self.lecturas_bien = 0
        self.como = None             # como se confirma este giro (maniobras.como_confirmar)
        self.frente_antes = None     # lo que media el frente antes de girar
        self.salida = None
        self.ultima_orientacion = None  # para sumar lo girado con /odom
        self.ultimo_giro = 0.0       # el ultimo rad/s mandado (para sumar sin /odom)
        self.frente_objetivo = None  # RETROCEDIENDO: lo que mide el frente en el medio
        self.desde = None            # cuando empezo a girar o retroceder (para el limite)
        self.pedido = None           # la orden en curso (goal handle), o None
        self.fin = None              # Future que se completa con el resultado

        self.create_subscription(Odometry, 'odom', self.al_moverse, 10)
        self.create_subscription(
            Float32MultiArray, 'distancias', self.al_medir, qos_profile_sensor_data)
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'modo_actual', self.al_cambiar_modo, guardado)
        # Con PARAR puesto el selector no deja pasar nada: no sumar lo que "avanzaria"
        self.frenado = False
        self.create_subscription(Bool, 'freno_operador',
                                 lambda m: setattr(self, 'frenado', m.data), guardado)
        self.create_subscription(Lugar, '/navegacion/lugar', self.al_llegar_a_lugar, 10)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.pub_reiniciar = self.create_publisher(Empty, '/navegacion/reiniciar_detector', 10)
        # Para Mapeo: por que salida se fue del lugar (la anota PROBADA en la libreta)
        self.pub_salida = self.create_publisher(String, '/navegacion/salida_tomada', 10)
        # El panel muestra en rojo los avisos que empiezan con AYUDA
        self.pub_aviso = self.create_publisher(String, 'navegacion/aviso', 10)
        self.posicion = PosicionLaberinto()
        self.pub_posicion = self.create_publisher(Odometry, 'odom_laberinto', 10)
        self.create_subscription(
            Odometry, '/navegacion/poner_posicion', self.al_poner_posicion, 10)

        ActionServer(self, Mover, '/mover', self.ejecutar,
                     goal_callback=self.al_recibir_pedido,
                     cancel_callback=lambda _pedido: CancelResponse.ACCEPT)
        self.create_timer(1.0 / p['frecuencia_hz'], self.paso)
        self.get_logger().info(f'Mover listo (avanza a {p["velocidad_avance"]} m/s)')

    # ----------------------------- Lo que escucha -----------------------------

    def al_moverse(self, mensaje):
        self.orientacion = orientacion_de(mensaje.pose.pose.orientation)
        self.hora_odom = self.get_clock().now()

    def al_poner_posicion(self, mensaje):
        pose = mensaje.pose.pose
        self.posicion.poner(pose.position.x, pose.position.y, orientacion_de(pose.orientation))
        self.get_logger().info(f'Posicion corregida: ({pose.position.x:.2f}, '
                               f'{pose.position.y:.2f})')

    def al_medir(self, mensaje):
        if len(mensaje.data) == 3:
            self.izquierda_cm, self.derecha_cm, self.frente_cm = (
                distancia_a_lectura(cm) for cm in mensaje.data)
            if self.etapa == GIRANDO and self.mirando:
                self.mirar_lectura()

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
        giro = giro_de_salida(orden.salida)
        if giro is None:
            self.get_logger().error(f'Salida desconocida: "{orden.salida}"')
            self.terminar(Mover.Result.BLOQUEADA)
            return
        self.pub_salida.publish(String(data=orden.salida))
        if orden.salida == Lugar.ADELANTE:
            self.salir_del_lugar()       # no hay que girar
            return
        self.giro_pedido = giro
        self.salida = orden.salida
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
        # Posicion de laberinto: suma lo que avanzo (o retrocedio) en este paso
        if not self.frenado:
            self.posicion.avanzar(orden.linear.x / self.p['frecuencia_hz'])
        self.publicar_posicion()

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

    def empezar_a_girar(self):
        self.girado = 0.0
        self.mirando = False
        self.ultimo_giro = 0.0
        self.ultima_orientacion = self.orientacion if self.hay_odom() else None
        self.objetivo_estimado = self.giro_pedido
        self.frente_antes = self.frente_cm
        self.como = como_confirmar(self.salida, self.frente_antes, self.p['frente_abierto_cm'])
        self.get_logger().info(
            'Giro con ' + ('/odom' if self.hay_odom() else 'el tiempo (no hay /odom)') +
            f', confirma {self.como} (frente antes: {self.frente_antes} cm)')

    def girar(self):
        if self.paso_el_tiempo(self.p['tiempo_max_giro_s']):
            self.pedir_ayuda('el giro no termino a tiempo')
            return 0.0
        self.girado += self.giro_desde_el_paso_anterior()
        if self.mirando:
            return self.mandar_giro(0.0)      # quieto: mirar_lectura() decide

        # 1. Girar hasta lo estimado
        velocidad = velocidad_de_giro(
            self.objetivo_estimado - self.girado, self.p['ganancia_giro'],
            self.p['giro_min'], self.p['giro_max'], self.tolerancia)
        if velocidad != 0.0:
            return self.mandar_giro(velocidad)
        if self.como == SIN_CONFIRMAR:
            self.terminar_giro()   # puerta: confia en lo estimado
            return self.mandar_giro(0.0)
        # 2. Frenar y mirar
        self.mirando = True
        self.mirar_desde = self.get_clock().now()
        self.lecturas_bien = 0
        return self.mandar_giro(0.0)

    def mirar_lectura(self):
        """Con el robot quieto, cada lectura nueva de /distancias suma o corta la cuenta."""
        quieto = (self.get_clock().now() - self.mirar_desde).nanoseconds / 1e9
        if quieto < self.p['espera_quieto_s']:
            return                            # todavia frenando: la lectura no sirve
        bien = giro_confirmado(
            self.como, self.salida, self.izquierda_cm, self.derecha_cm, self.frente_cm,
            self.frente_antes, self.p['tolerancia_costado_cm'], self.p['frente_abierto_cm'])
        if bien is None:
            return                            # sin dato: no cuenta
        if bien:
            self.lecturas_bien += 1
            if self.lecturas_bien >= self.p['lecturas_confirmar']:
                self.terminar_giro()
            return
        # 3. Dio mal: giro de menos. Un paso mas, con limite
        extra = abs(self.girado) - abs(self.giro_pedido)
        if extra >= math.radians(self.p['giro_extra_max_grados']):
            # 4. Nunca confirmo: el robot no esta girando (o no es la salida que creia)
            self.pedir_ayuda(
                f'gire {math.degrees(self.girado):.0f} grados y no confirmo: '
                f'izq {self.izquierda_cm}, der {self.derecha_cm}, frente {self.frente_cm} cm')
            return
        self.objetivo_estimado = self.girado + math.copysign(
            math.radians(self.p['paso_grados']), self.giro_pedido)
        self.mirando = False

    def publicar_posicion(self):
        mensaje = Odometry()
        mensaje.header.stamp = self.get_clock().now().to_msg()
        mensaje.header.frame_id = 'odom'
        mensaje.pose.pose.position.x = self.posicion.x
        mensaje.pose.pose.position.y = self.posicion.y
        angulo = self.posicion.orientacion()
        mensaje.pose.pose.orientation.z = math.sin(angulo / 2)
        mensaje.pose.pose.orientation.w = math.cos(angulo / 2)
        self.pub_posicion.publish(mensaje)

    def terminar_giro(self):
        self.get_logger().info(
            f'Giro confirmado ({self.como}): {math.degrees(self.girado):.0f} grados, '
            f'izq {self.izquierda_cm}, der {self.derecha_cm}, frente {self.frente_cm} cm')
        self.mirando = False
        self.posicion.girar(self.giro_pedido)   # un giro confirmado: cuartos de vuelta justos
        self.salir_del_lugar()

    def giro_desde_el_paso_anterior(self):
        """Cuanto giro desde el paso anterior (rad): con /odom si hay, si no por el tiempo."""
        if self.hay_odom():
            anterior = self.ultima_orientacion
            self.ultima_orientacion = self.orientacion
            if anterior is None:
                return 0.0
            return normalizar_angulo(self.orientacion - anterior)
        self.ultima_orientacion = None
        return self.ultimo_giro * self.p['factor_giro_tiempo'] / self.p['frecuencia_hz']

    def mandar_giro(self, velocidad):
        self.ultimo_giro = velocidad
        return velocidad

    def pedir_ayuda(self, motivo):
        texto = f'AYUDA: no pude girar ({motivo}). Pasa a manual y giralo a mano'
        self.get_logger().error(texto)
        self.pub_aviso.publish(String(data=texto))
        self.terminar(Mover.Result.BLOQUEADA)

    def hay_odom(self):
        if self.hora_odom is None:
            return False
        edad = (self.get_clock().now() - self.hora_odom).nanoseconds / 1e9
        return edad <= self.p['edad_max_odom_s']

    # ------------------------------- Ayudas -------------------------------

    def cambiar_etapa(self, etapa):
        if etapa != self.etapa:
            self.get_logger().info(f'Etapa: {etapa}')
        self.etapa = etapa
        self.desde = self.get_clock().now()
        if etapa == GIRANDO:
            self.empezar_a_girar()

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
