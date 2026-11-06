# Nodo mapeo (T3) y exploracion (T4): llena la libreta del recorrido con los avisos
# del detector, y cuando Decision no sabe por donde ir, elige con la libreta.
# Las cuentas estan en libreta.py (Libreta) y exploracion.py (elegir_salida); este
# nodo solo las conecta con ROS.
#
# Exploracion va en este mismo nodo porque necesita la libreta al dia: si fuera otro
# nodo, podria elegir antes de que se anote el lugar donde esta.
#   /decision/explorar (Empty, de Decision) ──► elegir_salida() ──► /mover
#
# Y la vuelta al inicio (T8): atiende la accion /volver (la pide Decision cuando
# agarro el cubo). Tambien necesita la libreta al dia, por eso va aca.
#   1. Si esta en un pasillo (salio de un lugar y todavia no llego al siguiente, por
#      ejemplo despues de agarrar el cubo), da media vuelta y vuelve a ese lugar.
#   2. En cada lugar calcula el camino mas rapido a la largada (vuelta.py, Dijkstra)
#      y le pide a /mover la primera salida. Recalcula en cada lugar.
#   3. En el primer lugar sale hacia la largada y avanza lo que tardo al principio de
#      la ronda. Ahi pide modo MANUAL (topic modo): el robot frena y el operador deja
#      el cubo a mano en el contenedor. Contesta TERMINADA.
#   4. Al volver a automatico despues de dejar el cubo, la posicion de laberinto
#      vuelve a (0, 0) mirando al norte: el operador tiene que dejar el robot como
#      arranco la ronda.
# Si en el medio pasan a manual, la vuelta queda en pausa: sigue en el proximo lugar
# que reconozca. Si no hay camino (la libreta quedo partida), pide AYUDA.
#
#   /navegacion/lugar (detector)        ──► Libreta.anotar()        ──┐
#   /navegacion/salida_tomada (/mover)  ──► Libreta.tomar_salida()  ──┼──► mapeo/libreta (JSON)
#   modo_actual (selector): en manual   ──► Libreta.pasar_a_manual() ─┘     (lo dibuja el panel)
#
# mapeo/libreta es un String con JSON, el formato que espera el panel
# (TICKETS_NAVEGACION.md, T11): {"lugares": [...], "actual": 1, "camino": []}.
# Se publica guardado (transient_local): quien se conecta tarde recibe la ultima.
# "camino" queda vacio hasta que exista /volver (Dijkstra).
#
# La orientacion para tomar_salida es la del aviso del lugar, no la de ese momento:
# /mover publica la salida antes de girar, pero asi no depende del orden.

import json
from dataclasses import asdict

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import Empty, String

from prometeo_interfaces.action import Mover, Volver
from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.exploracion import elegir_salida
from prometeo_navigation.libreta import LARGADA, Libreta, salida_hacia
from prometeo_navigation.modos import MANUAL
from prometeo_navigation.vuelta import camino_a_la_largada

# Si no se sabe cuanto tardo el primer tramo (la largada), avanza esto antes de frenar
SEGUNDOS_LARGADA_SI_NO_SE_SABE = 3.0


def libreta_a_json(libreta, camino=()):
    """La libreta como texto JSON, en el formato que dibuja el panel.

    camino: los numeros de lugar de la vuelta (el panel la dibuja en verde).
    """
    return json.dumps({
        'lugares': [asdict(hoja) for hoja in libreta.lugares],
        'actual': libreta.actual.numero if libreta.actual is not None else None,
        'camino': list(camino),
    })


class NodoMapeo(Node):

    def __init__(self):
        super().__init__('mapeo')
        margen_m = self.declare_parameter('margen_m', 0.30).value
        self.libreta = Libreta(margen_m)
        self.orientacion = None   # la del ultimo aviso
        self.en_manual = False
        self.hora_inicio = None   # cuando arranco la ronda en automatico (para la largada)
        # La vuelta (/volver)
        self.volviendo = None     # el pedido en curso, o None
        self.fin_volver = None
        self.camino = []          # para el panel
        self.ignorar_salida = False   # la media vuelta en el pasillo no es salir de un lugar
        self.segundos_final = None    # cuanto avanzar desde el primer lugar hasta la largada
        self.temporizador = None
        self.en_largada = False   # llego y paso a manual para dejar el cubo

        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(AvisoLugar, '/navegacion/lugar', self.al_llegar, 10)
        self.create_subscription(String, '/navegacion/salida_tomada', self.al_salir, 10)
        self.create_subscription(String, 'modo_actual', self.al_cambiar_modo, guardado)
        self.create_subscription(Empty, '/decision/explorar', self.al_pedir_explorar, 10)
        self.pub_libreta = self.create_publisher(String, 'mapeo/libreta', guardado)
        self.mover = ActionClient(self, Mover, '/mover')
        self.pub_modo = self.create_publisher(String, 'modo', 10)
        self.pub_aviso = self.create_publisher(String, 'navegacion/aviso', 10)
        self.pub_posicion = self.create_publisher(Odometry, '/navegacion/poner_posicion', 10)
        ActionServer(self, Volver, '/volver', self.ejecutar_volver,
                     cancel_callback=self.al_cancelar_volver)
        self.publicar()
        self.get_logger().info(f'Mapeo listo (margen {margen_m} m)')

    def al_llegar(self, aviso):
        if self.en_manual:
            return
        self.orientacion = aviso.orientacion
        primero = not self.libreta.lugares
        hoja, es_nuevo = self.libreta.anotar(
            aviso.tipo, aviso.x, aviso.y, aviso.orientacion, list(aviso.salidas), self.ahora())
        if primero and self.hora_inicio is not None:
            # El primer tramo: desde que arranco la ronda hasta el primer lugar
            for camino in hoja.salidas:
                if camino.lleva_a == LARGADA:
                    camino.segundos = self.ahora() - self.hora_inicio
        salidas = ', '.join(f'{s.direccion} {s.estado}' for s in hoja.salidas)
        self.get_logger().info(
            f'{"Lugar NUEVO" if es_nuevo else "Ya estuve en el"} {hoja.numero}: '
            f'{hoja.tipo} ({hoja.x:.2f}, {hoja.y:.2f}) [{salidas}]')
        self.publicar()
        if self.volviendo is not None:
            self.dar_paso(hoja, aviso.orientacion)

    def al_salir(self, mensaje):
        if self.ignorar_salida:
            self.ignorar_salida = False
            return
        if self.orientacion is None:
            return
        camino = self.libreta.tomar_salida(self.orientacion, mensaje.data, self.ahora())
        if camino is None:
            self.get_logger().warning(f'Salida "{mensaje.data}" sin lugar en la libreta')
            return
        self.get_logger().info(
            f'Sale del {self.libreta.actual.numero} hacia el {camino.direccion}')
        self.publicar()

    def al_cambiar_modo(self, mensaje):
        en_manual = mensaje.data == MANUAL
        if en_manual and not self.en_manual:
            self.libreta.pasar_a_manual()
            self.get_logger().info('Manual: no anoto el tramo a mano' +
                                   (' (la vuelta queda en pausa)' if self.volviendo else ''))
            self.publicar()
        if not en_manual:
            if self.hora_inicio is None:
                self.hora_inicio = self.ahora()
            if self.en_largada:
                # Dejaron el cubo y el robot esta otra vez como arranco la ronda
                self.en_largada = False
                self.poner_posicion_en_la_largada()
        self.en_manual = en_manual

    def al_pedir_explorar(self, _mensaje):
        if self.en_manual or self.orientacion is None:
            return
        salida = elegir_salida(self.libreta, self.orientacion)
        if salida is None:
            self.get_logger().warning('Explorar: no hay lugar actual en la libreta')
            return
        self.get_logger().info(f'Explorar: elijo {salida}')
        if not self.mover.server_is_ready():
            self.get_logger().warning('/mover todavia no responde, mando igual')
        self.mover.send_goal_async(Mover.Goal(accion=Mover.Goal.IR_A_SALIDA, salida=salida))

    # ------------------------------ /volver ------------------------------

    async def ejecutar_volver(self, pedido):
        self.volviendo = pedido
        self.fin_volver = Future()
        self.get_logger().info(f'Volver a la largada (cubo: "{pedido.request.color}")')
        if self.libreta.saliendo_por is not None or not self.libreta.lugares:
            # 1. Esta en un pasillo: media vuelta y vuelve al lugar de donde salio.
            #    No se une nada: va a llegar otra vez al mismo lugar.
            self.libreta.saliendo_por = None
            self.ignorar_salida = True
            if not self.libreta.lugares:
                # Todavia no paso por ningun lugar: vuelve lo que avanzo desde el inicio
                self.segundos_final = self.ahora() - (self.hora_inicio or self.ahora())
            self.pedir_mover(Mover.Goal.VUELTA, ultimo=not self.libreta.lugares)
        elif self.libreta.actual is not None and self.orientacion is not None:
            self.dar_paso(self.libreta.actual, self.orientacion)
        # Si no: espera a que el detector avise un lugar (por ejemplo, volvio de manual)
        resultado = await self.fin_volver
        self.volviendo = None
        self.camino = []
        self.publicar()
        if resultado == Volver.Result.TERMINADA:
            pedido.succeed()
        elif resultado == Volver.Result.CANCELADA:
            pedido.canceled()
        else:
            pedido.abort()
        return Volver.Result(resultado=resultado)

    def al_cancelar_volver(self, _pedido):
        if self.fin_volver is not None and not self.fin_volver.done():
            self.fin_volver.set_result(Volver.Result.CANCELADA)
        return CancelResponse.ACCEPT

    def dar_paso(self, hoja, orientacion):
        """2. En un lugar: pide la primera salida del camino mas rapido a la largada."""
        if self.en_manual:
            return
        pasos = camino_a_la_largada(self.libreta, hoja)
        if pasos is None:
            self.avisar(f'AYUDA: no se volver desde el lugar {hoja.numero} (la libreta quedo '
                        f'partida). Llevalo a mano hasta un lugar conocido')
            return
        self.camino = [numero for numero, _ in pasos]
        self.publicar()
        _, direccion = pasos[0]
        ultimo = len(pasos) == 1
        if ultimo:
            largada = [s for s in hoja.salidas if s.lleva_a == LARGADA]
            self.segundos_final = largada[0].segundos if largada else None
        self.get_logger().info(
            f'Vuelta: {" -> ".join(map(str, self.camino))} -> largada. Salgo al {direccion}')
        self.pedir_mover(salida_hacia(orientacion, direccion), ultimo)

    def pedir_mover(self, salida, ultimo):
        def al_terminar(futuro):
            resultado = futuro.result().result.resultado
            if resultado == Mover.Result.TERMINADA and ultimo:
                self.esperar_la_largada()
            # BLOQUEADA: /mover ya pidio AYUDA; sigue cuando el operador lo destrabe

        def al_aceptar(futuro):
            if futuro.result().accepted:
                futuro.result().get_result_async().add_done_callback(al_terminar)
            else:
                self.get_logger().warning('/mover rechazo el pedido de la vuelta')

        self.mover.send_goal_async(
            Mover.Goal(accion=Mover.Goal.IR_A_SALIDA, salida=salida)
        ).add_done_callback(al_aceptar)

    def esperar_la_largada(self):
        """3. Ya salio hacia la largada: avanza lo que tardo al principio y frena."""
        segundos = self.segundos_final or SEGUNDOS_LARGADA_SI_NO_SE_SABE
        self.get_logger().info(f'Hacia la largada: freno en {segundos:.1f} s')

        def llegar():
            self.temporizador.cancel()
            self.pub_modo.publish(String(data=MANUAL))   # frena: el cubo se deja a mano
            self.en_largada = True
            self.avisar('Llegue a la largada: deja el cubo a mano. Despues ponelo como '
                        'arranco la ronda y volve a automatico')
            if self.fin_volver is not None and not self.fin_volver.done():
                self.fin_volver.set_result(Volver.Result.TERMINADA)

        self.temporizador = self.create_timer(segundos, llegar)

    def poner_posicion_en_la_largada(self):
        """4. La posicion de laberinto vuelve a (0, 0) mirando al norte, como al arrancar."""
        mensaje = Odometry()
        mensaje.pose.pose.orientation = Quaternion(w=1.0)
        self.pub_posicion.publish(mensaje)
        self.get_logger().info('En la largada: posicion (0, 0) mirando al norte')

    # ------------------------------ Ayudas ------------------------------

    def avisar(self, texto):
        self.get_logger().warning(texto)
        self.pub_aviso.publish(String(data=texto))

    def ahora(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publicar(self):
        self.pub_libreta.publish(String(data=libreta_a_json(self.libreta, self.camino)))


def main():
    rclpy.init()
    nodo = NodoMapeo()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
