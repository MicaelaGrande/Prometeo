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
#   /brazo/agarrado      el brazo agarro (true) o no     -> logica.cubo_agarrado() / cubo_no_agarrado()
#   /prometeo/modo_actual     'auto' o 'manual', del selector del panel
#   /prometeo/freno_operador  true si apretaron PARAR
#
# Solo decide en modo 'auto' y sin PARAR. Mientras tanto ignora la camara y los
# lugares, porque el robot lo esta manejando una persona.
#
# Manda:
#   /mover               accion: ir a una salida o acercarse al cubo
#   /volver              accion: volver al inicio (con o sin cubo)
#   /decision/explorar   timbre para que Exploracion elija por donde ir
#   /decision/agarrar    timbre para que el brazo agarre el cubo
#   /prometeo/seguridad_habilitada  false = modo recoger (sin freno) para acercarse al cubo
#   /decision/estado     en que anda Decision, para mirar desde la terminal

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String

from huggin_vision_msgs.msg import DetectionArray
from prometeo_interfaces.action import Mover, Volver
from prometeo_interfaces.msg import Lugar

from prometeo_decision.logica import (
    ACERCARSE, AGARRAR, Decision, EXPLORAR, IR_A_SALIDA, VOLVER, VOLVIENDO,
)


class NodoDecision(Node):

    def __init__(self):
        super().__init__('decision')

        # Mientras no este el brazo, hace de cuenta que agarro el cubo enseguida.
        # Cuando el brazo exista: ros2 run ... --ros-args -p sin_brazo:=false
        self.sin_brazo = self.declare_parameter('sin_brazo', True).value
        confianza = self.declare_parameter('confianza_minima', 0.6).value

        # Donde escucha la ESP32 el freno. Con el firmware nuevo lleva /prometeo
        # adelante. Con el viejo: -p topic_seguridad:=/seguridad_habilitada
        topic_seguridad = self.declare_parameter(
            'topic_seguridad', '/prometeo/seguridad_habilitada').value

        # Si es false, decide siempre, sin esperar al selector de modo del panel.
        # Sirve para probar a mano con navegacion_falsa: -p usar_selector:=false
        self.usar_selector = self.declare_parameter('usar_selector', True).value

        # Hasta que el selector avise, supone lo seguro: manual y frenado.
        self.modo = 'manual'
        self.freno = True

        self.logica = Decision(confianza_minima=confianza)

        # Lo que escucha
        self.create_subscription(DetectionArray, '/vision/detections', self.al_ver, 10)
        self.create_subscription(Lugar, '/navegacion/lugar', self.al_llegar_a_lugar, 10)
        self.create_subscription(Bool, '/brazo/agarrado', self.al_responder_el_brazo, 10)

        # El selector repite el modo y el freno, y los guarda para el que se
        # conecta tarde (TRANSIENT_LOCAL). Hay que escucharlos igual.
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/prometeo/modo_actual', self.cambio_de_modo, guardado)
        self.create_subscription(Bool, '/prometeo/freno_operador', self.cambio_de_freno, guardado)

        # Lo que manda
        self.pub_explorar = self.create_publisher(Empty, '/decision/explorar', 10)
        self.pub_agarrar = self.create_publisher(Empty, '/decision/agarrar', 10)
        self.pub_seguridad = self.create_publisher(Bool, topic_seguridad, 10)
        self.pub_estado = self.create_publisher(String, '/decision/estado', 10)
        self.mover = ActionClient(self, Mover, '/mover')
        self.volver = ActionClient(self, Volver, '/volver')

        # Publica el estado una vez por segundo, para verlo con ros2 topic echo.
        self.create_timer(1.0, self.publicar_estado)
        self.get_logger().info(
            f'Decision lista (sin_brazo={self.sin_brazo}, usar_selector={self.usar_selector}, '
            f'freno en {topic_seguridad})')

    # -----------------------------------------------------------------------
    # Lo que llega
    # -----------------------------------------------------------------------

    def puede_decidir(self):
        """True si esta en automatico y nadie apreto PARAR."""
        if not self.usar_selector:
            return True
        return self.modo == 'auto' and not self.freno

    def cambio_de_modo(self, msg):
        if msg.data == self.modo:
            return
        self.get_logger().info(f'Modo: {msg.data}')
        self.modo = msg.data
        if msg.data != 'auto':
            self.olvidar_lo_visto()

    def cambio_de_freno(self, msg):
        if msg.data == self.freno:
            return
        self.get_logger().info(f'PARAR: {"puesto" if msg.data else "suelto"}')
        self.freno = msg.data
        if msg.data:
            self.olvidar_lo_visto()

    def olvidar_lo_visto(self):
        """Cuando lo manejan a mano, el robot se mueve y lo que vio ya no sirve.

        Se olvida de la flecha guardada y de las ultimas imagenes, para no usar
        una flecha vieja en una bifurcacion equivocada. El color que sigue y los
        cubos que ya deposito no se tocan.
        """
        self.logica.flecha_pendiente = None
        self.logica.ultimas.clear()

    def al_ver(self, msg):
        if not self.puede_decidir():
            return
        self.hacer(self.logica.ver(msg.detections))

    def al_llegar_a_lugar(self, msg):
        if not self.puede_decidir():
            return
        self.get_logger().info(f'Lugar: {msg.tipo} con salidas {list(msg.salidas)}')
        self.hacer(self.logica.llegar_a_lugar(msg.tipo, list(msg.salidas)))

    def al_responder_el_brazo(self, msg):
        if not self.puede_decidir():
            return
        if msg.data:
            self.hacer(self.logica.cubo_agarrado())
        else:
            self.hacer(self.logica.cubo_no_agarrado())

    # -----------------------------------------------------------------------
    # Lo que manda: traduce cada Orden de la logica a un mensaje de ROS
    # -----------------------------------------------------------------------

    def hacer(self, orden):
        """Ejecuta la Orden que devolvio la logica. Si es None, no hace nada."""
        if orden is None:
            return
        self.get_logger().info(f'Orden: {orden}  (estado: {self.logica.estado})')

        if orden.accion == IR_A_SALIDA:
            self.pedir_mover('ir_a_salida', orden.salida, orden.length_class)

        elif orden.accion == ACERCARSE:
            self.pub_seguridad.publish(Bool(data=False))  # modo recoger: sin freno
            self.pedir_mover('acercarse')

        elif orden.accion == EXPLORAR:
            self.pub_explorar.publish(Empty())

        elif orden.accion == AGARRAR:
            if self.sin_brazo:
                self.hacer(self.logica.cubo_agarrado())  # hace de cuenta que lo agarro
            else:
                self.pub_agarrar.publish(Empty())

        elif orden.accion == VOLVER:
            self.pub_seguridad.publish(Bool(data=True))  # vuelve el freno
            self.pedir_volver()

        self.publicar_estado()

    def pedir_mover(self, accion, salida='', length_class=''):
        pedido = Mover.Goal()
        pedido.accion = accion
        pedido.salida = salida or ''
        pedido.length_class = length_class or ''
        self.enviar(self.mover, pedido,
                    lambda resultado: self.al_terminar_mover(accion, resultado))

    def pedir_volver(self):
        pedido = Volver.Goal()
        # Con cubo: el color, para ir a su contenedor. Sin cubo: vacio.
        pedido.color = self.logica.color_actual or ''
        self.enviar(self.volver, pedido, self.al_terminar_volver)

    def enviar(self, cliente, pedido, al_terminar):
        """Manda un pedido a una accion y llama a al_terminar con la respuesta."""
        if not cliente.server_is_ready():
            self.get_logger().warn(f'{cliente._action_name} todavia no responde, mando igual')

        def aceptado(futuro):
            handle = futuro.result()
            if not handle.accepted:
                self.get_logger().warn(f'{cliente._action_name} rechazo el pedido')
                return
            handle.get_result_async().add_done_callback(
                lambda f: al_terminar(f.result().result.resultado))

        cliente.send_goal_async(pedido).add_done_callback(aceptado)

    # -----------------------------------------------------------------------
    # Las respuestas de las acciones
    # -----------------------------------------------------------------------

    def al_terminar_mover(self, accion, resultado):
        self.get_logger().info(f'/mover ({accion}) contesto: {resultado}')
        # Solo importa cuando termina de acercarse: ahi llego al cubo.
        # (Si termina un ir_a_salida, sigue andando y no hay nada que decidir.)
        if accion == 'acercarse' and resultado == 'terminada':
            self.hacer(self.logica.llegue_al_cubo())

    def al_terminar_volver(self, resultado):
        self.get_logger().info(f'/volver contesto: {resultado}')
        if resultado != 'terminada':
            return
        if self.logica.estado == VOLVIENDO:
            self.logica.cubo_depositado()   # volvio con el cubo
        else:
            self.logica.llegue_al_inicio()  # volvio sin cubo
        self.publicar_estado()

    def publicar_estado(self):
        l = self.logica
        decide = 'si' if self.puede_decidir() else 'no (manual o PARAR)'
        texto = (f'estado={l.estado} color={l.color_actual} '
                 f'faltan={sorted(l.cubos_faltantes)} decide={decide}')
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
