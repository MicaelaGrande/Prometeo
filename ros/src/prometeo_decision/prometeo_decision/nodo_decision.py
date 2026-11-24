# Nodo de ROS de Decision: el "cuerpo" de logica.py.
#
# Escucha lo que pasa en el robot, se lo pasa a la logica, y manda las
# ordenes que la logica devuelve. La logica no sabe nada de ROS: este
# archivo es el traductor entre los dos.
#
# Se corre con:
#   ros2 run prometeo_decision nodo_decision
#
# Escucha:
#   /vision/detections   lo que ve la camara             -> logica.ver()
#   /navegacion/lugar    llego a curva/bifurcacion/...   -> logica.llegar_a_lugar()
#   /prometeo/modo_actual el modo de la llave (prometeo_interfaces/msg/Modo.msg). Es de
#                        una sola via: MANUAL -> AUTO -> TERMINADO.
#
# Solo decide en AUTO. En MANUAL lo esta manejando una persona (lo trae a la
# largada), y en TERMINADO ya termino: en los dos ignora la camara y los lugares.
#
# Manda:
#   /mover               accion: ir a una salida o acercarse al cubo
#                        (si acercarse no termina bien, o no contesta en
#                        tiempo_max_acercando_s, es falsa alarma -> sigue por el pasillo)
#   /volver              accion: volver al inicio (con o sin cubo)
#                        con cubo, al terminar -> logica.llegue_al_deposito() -> soltar
#   /brazo               accion: agarrar o soltar el cubo (Brazo.action). Contesta
#                        listo o fallo. Si no contesta en tiempo_max_brazo_s, es fallo.
#                        Con sin_brazo:=true (por defecto) no se usa: hace de cuenta
#                        que agarra y suelta enseguida.
#   /decision/explorar   timbre para que Exploracion elija por donde ir
#   /decision/retomar    timbre: abandono el color, Navegacion vuelve al ultimo lugar
#   /prometeo/seguridad_habilitada  false = modo recoger (sin freno) para acercarse al cubo
#   /decision/estado     en que anda Decision, para mirar desde la terminal
#   /prometeo/modo       cuando la logica TERMINA (no queda ningun color), le pide
#                        PEDIR_PARAR a la llave: el robot queda quieto, fin de la ronda

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String

from huggin_vision_msgs.msg import DetectionArray
from prometeo_interfaces.action import Brazo, Mover, Volver
from prometeo_interfaces.msg import Lugar, Modo

from prometeo_decision.logica import (
    ACERCARSE, AGARRAR, DISTANCIA_MAX_CUBO, DISTANCIA_MIN_CUBO, Decision, EXPLORAR,
    IMAGENES_CUBO, IR_A_SALIDA, MINIMO_CUBO, RETOMAR,
    SEGUIR, SOLTAR, TERMINADO, VOLVER, VOLVIENDO,
)


class NodoDecision(Node):

    def __init__(self):
        super().__init__('decision')

        # Mientras no este el brazo, hace de cuenta que agarra y suelta el cubo
        # enseguida. Con el brazo (o brazo_falso): --ros-args -p sin_brazo:=false
        self.sin_brazo = self.declare_parameter('sin_brazo', True).value
        confianza = self.declare_parameter('confianza_minima', 0.6).value
        # Filtro del cubo: lo tiene que ver en minimo_cubo de las ultimas
        # imagenes_cubo imagenes (por defecto 4 de 5).
        imagenes_cubo = self.declare_parameter('imagenes_cubo', IMAGENES_CUBO).value
        minimo_cubo = self.declare_parameter('minimo_cubo', MINIMO_CUBO).value
        # Un cubo mas cerca o mas lejos que esto (metros) se descarta: es una
        # mancha que no es un cubo (por ejemplo el fondo azul).
        distancia_min_cubo = self.declare_parameter(
            'distancia_min_cubo', DISTANCIA_MIN_CUBO).value
        distancia_max_cubo = self.declare_parameter(
            'distancia_max_cubo', DISTANCIA_MAX_CUBO).value

        # Donde escucha la ESP32 el freno. Con el firmware nuevo lleva /prometeo
        # adelante. Con el viejo: -p topic_seguridad:=/seguridad_habilitada
        topic_seguridad = self.declare_parameter(
            'topic_seguridad', '/prometeo/seguridad_habilitada').value

        # Si es false, decide siempre, sin esperar a que la llave pase a automatico.
        # Sirve para probar a mano con navegacion_falsa: -p usar_selector:=false
        self.usar_selector = self.declare_parameter('usar_selector', True).value

        # Reloj de seguridad: si /mover no contesta en este tiempo mientras se acerca
        # al cubo, cuenta como que no llego (falsa alarma). Mas que los 25 s de mover.
        self.tiempo_max_acercando = self.declare_parameter(
            'tiempo_max_acercando_s', 40.0).value
        self.reloj_acercando = None
        # Numero del ultimo pedido de acercarse: una respuesta vieja que llega tarde
        # (de un acercarse anterior) no se confunde con la del actual.
        self.numero_acercarse = 0

        # Reloj de seguridad del brazo: si no contesta en este tiempo, cuenta como
        # que fallo (asi Decision nunca queda esperando para siempre).
        self.tiempo_max_brazo = self.declare_parameter('tiempo_max_brazo_s', 20.0).value
        self.reloj_brazo = None
        self.numero_brazo = 0   # igual que numero_acercarse, para el brazo

        # Hasta que la llave avise, supone lo seguro: manual (no decide nada).
        self.modo = Modo.MANUAL
        # Si ya le pidio a la llave que pare (para pedirlo una sola vez)
        self.parada_pedida = False

        self.logica = Decision(confianza_minima=confianza, imagenes_cubo=imagenes_cubo,
                               minimo_cubo=minimo_cubo,
                               distancia_min_cubo=distancia_min_cubo,
                               distancia_max_cubo=distancia_max_cubo)

        # Lo que escucha
        self.create_subscription(DetectionArray, '/vision/detections', self.al_ver, 10)
        self.create_subscription(Lugar, '/navegacion/lugar', self.al_llegar_a_lugar, 10)

        # La llave repite el modo y lo guarda para el que se conecta tarde
        # (TRANSIENT_LOCAL). Hay que escucharlo igual.
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/prometeo/modo_actual', self.cambio_de_modo, guardado)

        # Lo que manda
        self.pub_explorar = self.create_publisher(Empty, '/decision/explorar', 10)
        # Abandono el color: Navegacion vuelve al ultimo lugar y se decide de nuevo
        self.pub_retomar = self.create_publisher(Empty, '/decision/retomar', 10)
        self.pub_seguridad = self.create_publisher(Bool, topic_seguridad, 10)
        self.pub_estado = self.create_publisher(String, '/decision/estado', 10)
        # Para pedirle a la llave que termine la ronda (PEDIR_PARAR)
        self.pub_modo = self.create_publisher(String, '/prometeo/modo', 10)
        self.mover = ActionClient(self, Mover, '/mover')
        self.volver = ActionClient(self, Volver, '/volver')
        self.brazo = ActionClient(self, Brazo, '/brazo')

        # Publica el estado una vez por segundo, para verlo con ros2 topic echo.
        self.create_timer(1.0, self.publicar_estado)
        self.get_logger().info(
            f'Decision lista (sin_brazo={self.sin_brazo}, usar_selector={self.usar_selector}, '
            f'freno en {topic_seguridad})')

    # -----------------------------------------------------------------------
    # Lo que llega
    # -----------------------------------------------------------------------

    def puede_decidir(self):
        """True si la llave esta en automatico."""
        if not self.usar_selector:
            return True
        return self.modo == Modo.AUTO

    def cambio_de_modo(self, msg):
        if msg.data == self.modo:
            return   # la llave repite el modo una vez por segundo
        self.get_logger().info(f'Modo: {msg.data}')
        self.modo = msg.data

    def al_ver(self, msg):
        if not self.puede_decidir():
            return
        self.hacer(self.logica.ver(msg.detections))

    def al_llegar_a_lugar(self, msg):
        if not self.puede_decidir():
            return
        self.get_logger().info(f'Lugar: {msg.tipo} con salidas {list(msg.salidas)}')
        self.hacer(self.logica.llegar_a_lugar(msg.tipo, list(msg.salidas)))

    # -----------------------------------------------------------------------
    # Lo que manda: traduce cada Orden de la logica a un mensaje de ROS
    # -----------------------------------------------------------------------

    def hacer(self, orden):
        """Ejecuta la Orden que devolvio la logica. Si es None, no hace nada."""
        if orden is None:
            return
        self.get_logger().info(f'Orden: {orden}  (estado: {self.logica.estado})')

        if orden.accion == IR_A_SALIDA:
            self.pedir_mover(Mover.Goal.IR_A_SALIDA, orden.salida, orden.length_class)

        elif orden.accion == ACERCARSE:
            self.pub_seguridad.publish(Bool(data=False))  # modo recoger: sin freno
            self.pedir_acercarse()

        elif orden.accion == SEGUIR:
            # Falsa alarma: vuelve el freno y sigue por el pasillo.
            # PENDIENTE confirmar en Gazebo que 'adelante' retoma bien el pasillo.
            self.pub_seguridad.publish(Bool(data=True))
            self.pedir_mover(Mover.Goal.IR_A_SALIDA, Lugar.ADELANTE)

        elif orden.accion == EXPLORAR:
            self.pub_explorar.publish(Empty())

        elif orden.accion == AGARRAR:
            if self.sin_brazo:
                self.hacer(self.logica.cubo_agarrado())  # hace de cuenta que lo agarro
            else:
                self.pedir_brazo(Brazo.Goal.AGARRAR)

        elif orden.accion == SOLTAR:
            if self.sin_brazo:
                self.logica.cubo_soltado()               # hace de cuenta que lo solto
            else:
                self.pedir_brazo(Brazo.Goal.SOLTAR)

        elif orden.accion == VOLVER:
            self.pub_seguridad.publish(Bool(data=True))  # vuelve el freno
            self.pedir_volver()

        elif orden.accion == RETOMAR:
            self.pub_seguridad.publish(Bool(data=True))  # vuelve el freno
            self.pub_retomar.publish(Empty())

        self.publicar_estado()

    def pedir_mover(self, accion, salida='', length_class=''):
        pedido = Mover.Goal()
        pedido.accion = accion
        pedido.salida = salida or ''
        pedido.length_class = length_class or ''
        self.enviar(self.mover, pedido,
                    lambda resultado: self.al_terminar_mover(accion, resultado))

    def pedir_acercarse(self):
        """Pide /mover acercarse y arranca el reloj de seguridad."""
        self.numero_acercarse += 1
        numero = self.numero_acercarse
        pedido = Mover.Goal()
        pedido.accion = Mover.Goal.ACERCARSE
        self.parar_reloj_acercando()
        self.reloj_acercando = self.create_timer(
            self.tiempo_max_acercando, self.se_acabo_el_tiempo_acercando)
        self.enviar(self.mover, pedido,
                    lambda resultado: self.al_terminar_acercarse(numero, resultado),
                    al_rechazar=lambda: self.al_terminar_acercarse(
                        numero, Mover.Result.BLOQUEADA))

    def parar_reloj_acercando(self):
        if self.reloj_acercando is not None:
            self.destroy_timer(self.reloj_acercando)
            self.reloj_acercando = None

    def se_acabo_el_tiempo_acercando(self):
        self.parar_reloj_acercando()
        self.get_logger().warn(
            f'/mover no contesto en {self.tiempo_max_acercando} s: no llegue al cubo')
        self.hacer(self.logica.no_llegue_al_cubo())

    def pedir_brazo(self, accion):
        """Pide /brazo agarrar o soltar, y arranca el reloj de seguridad del brazo."""
        self.numero_brazo += 1
        numero = self.numero_brazo
        pedido = Brazo.Goal()
        pedido.accion = accion
        pedido.color = self.logica.color_actual or ''
        self.parar_reloj_brazo()
        self.reloj_brazo = self.create_timer(
            self.tiempo_max_brazo, lambda: self.se_acabo_el_tiempo_brazo(numero, accion))
        self.enviar(self.brazo, pedido,
                    lambda resultado: self.al_responder_el_brazo(numero, accion, resultado),
                    al_rechazar=lambda: self.al_responder_el_brazo(
                        numero, accion, Brazo.Result.FALLO))

    def parar_reloj_brazo(self):
        if self.reloj_brazo is not None:
            self.destroy_timer(self.reloj_brazo)
            self.reloj_brazo = None

    def se_acabo_el_tiempo_brazo(self, numero, accion):
        self.get_logger().warn(f'/brazo no contesto en {self.tiempo_max_brazo} s ({accion})')
        self.al_responder_el_brazo(numero, accion, Brazo.Result.FALLO)

    def pedir_volver(self):
        pedido = Volver.Goal()
        # Con cubo: el color, para ir a su contenedor. Sin cubo: vacio.
        pedido.color = self.logica.color_actual or ''
        self.enviar(self.volver, pedido, self.al_terminar_volver)

    def enviar(self, cliente, pedido, al_terminar, al_rechazar=None):
        """Manda un pedido a una accion y llama a al_terminar con la respuesta.

        Si la accion rechaza el pedido, llama a al_rechazar (si se lo pasaron).
        """
        if not cliente.server_is_ready():
            self.get_logger().warn(f'{cliente._action_name} todavia no responde, mando igual')

        def aceptado(futuro):
            handle = futuro.result()
            if not handle.accepted:
                self.get_logger().warn(f'{cliente._action_name} rechazo el pedido')
                if al_rechazar is not None:
                    al_rechazar()
                return
            handle.get_result_async().add_done_callback(
                lambda f: al_terminar(f.result().result.resultado))

        cliente.send_goal_async(pedido).add_done_callback(aceptado)

    # -----------------------------------------------------------------------
    # Las respuestas de las acciones
    # -----------------------------------------------------------------------

    def al_terminar_mover(self, accion, resultado):
        # Si termina un ir_a_salida, sigue andando y no hay nada que decidir.
        self.get_logger().info(f'/mover ({accion}) contesto: {resultado}')

    def al_terminar_acercarse(self, numero, resultado):
        self.get_logger().info(f'/mover (acercarse) contesto: {resultado}')
        if numero != self.numero_acercarse:
            return  # respuesta de un acercarse viejo: ya no importa
        self.parar_reloj_acercando()
        if resultado == Mover.Result.TERMINADA:
            self.hacer(self.logica.llegue_al_cubo())      # llego al cubo
        else:
            self.hacer(self.logica.no_llegue_al_cubo())   # bloqueada o cancelada

    def al_terminar_volver(self, resultado):
        self.get_logger().info(f'/volver contesto: {resultado}')
        if resultado != Volver.Result.TERMINADA:
            return
        if self.logica.estado == VOLVIENDO:
            self.hacer(self.logica.llegue_al_deposito())   # volvio con el cubo: soltarlo
        else:
            self.logica.llegue_al_inicio()                 # volvio sin cubo
        self.publicar_estado()

    def al_responder_el_brazo(self, numero, accion, resultado):
        self.get_logger().info(f'/brazo ({accion}) contesto: {resultado}')
        if numero != self.numero_brazo:
            return  # respuesta de un pedido viejo (o ya se acabo su tiempo)
        self.parar_reloj_brazo()
        self.numero_brazo += 1  # este pedido ya tuvo respuesta: las siguientes no cuentan
        listo = resultado == Brazo.Result.LISTO
        if accion == Brazo.Goal.AGARRAR:
            self.hacer(self.logica.cubo_agarrado() if listo else self.logica.cubo_no_agarrado())
        else:
            if listo:
                self.logica.cubo_soltado()
                self.publicar_estado()
            else:
                self.hacer(self.logica.cubo_no_soltado())

    def terminar_si_no_queda_nada(self):
        """Si la logica ya no tiene ningun color por buscar, le pide a la llave que pare.

        Es el fin de la ronda: la llave pasa a TERMINADO y el robot queda quieto. Se
        pide una sola vez (se revisa cada vez que se publica el estado).
        """
        if self.logica.estado != TERMINADO or self.parada_pedida:
            return
        self.parada_pedida = True
        self.get_logger().warn('No queda ningun color: termino la ronda')
        self.pub_modo.publish(String(data=Modo.PEDIR_PARAR))

    def publicar_estado(self):
        self.terminar_si_no_queda_nada()
        l = self.logica
        decide = 'si' if self.puede_decidir() else f'no (modo {self.modo})'
        texto = (f'estado={l.estado} color={l.color_actual} '
                 f'faltan={sorted(l.cubos_faltantes)} decide={decide} '
                 f'cubos_descartados={l.cubos_descartados}')
        self.pub_estado.publish(String(data=texto))


def main():
    rclpy.init()
    nodo = NodoDecision()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
