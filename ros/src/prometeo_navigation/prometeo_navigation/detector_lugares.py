# Nodo detector de lugares (T2): avisa cuando el robot llega a una curva, una
# bifurcacion o un callejon. La cuenta esta en lugares.py (DetectorDeLugares);
# este nodo solo la conecta con ROS.
#
#   distancias (ultrasonidos) ──► DetectorDeLugares ──► /navegacion/lugar ──► Decision
#   odom (posicion)          ──┘   (lugares.py)                           └─► Mapeo (T3)
#
#   /navegacion/reiniciar_detector (Empty): el robot termino de salir del lugar
#       (termino de girar o de pasar de largo). Lo va a mandar quien atienda /mover;
#       mientras tanto se puede mandar a mano para probar.
#   modo_actual (String, del selector): en "manual" no avisa nada, porque los giros a
#       mano darian saltos falsos (T2, "En manual"). Al volver a "auto" mira alrededor.
#       Si nadie publica el modo (Gazebo sin selector), funciona como en automatico.
#       Con el parametro modo_testeo:=true SI detecta en manual (para grabar un rosbag
#       manejando a mano, o reproducirlo despues): avisa los lugares igual, y como nadie
#       atiende /mover para mandar el reinicio, se reinicia solo (mirar_alrededor)
#       reinicio_testeo_s segundos despues de cada aviso.
#
# Lee /distancias y no los tres /ultrasonido/*: ahi llegan los tres sensores JUNTOS
# en un solo mensaje, asi que cada mensaje es una lectura nueva de los tres. Con
# tres topics separados, el detector contaria dos veces la misma lectura (y
# confirmaria con menos lecturas reales), o mezclaria lecturas viejas.
#
# Topics sin "/" adelante (distancias, odom, modo_actual) toman el namespace:
#   - Gazebo:      ros2 run prometeo_navigation detector_lugares
#   - Robot real:  ... --ros-args -r __ns:=/prometeo   (-> /prometeo/distancias)
# /navegacion/lugar y /navegacion/reiniciar_detector son fijos: es lo que escucha Decision.
#
# Atiende un mensaje por vez (executor por defecto): el detector no se cuenta dos
# veces a la vez (T2, cuidado 5).

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Empty, Float32MultiArray, String

from prometeo_interfaces.msg import Lugar
from prometeo_navigation.lugares import DetectorDeLugares, distancia_a_lectura
from prometeo_navigation.modos import MANUAL


def orientacion_de(cuaternion):
    """El angulo hacia donde mira el robot (radianes), a partir del cuaternion de /odom."""
    q = cuaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NodoDetectorLugares(Node):

    def __init__(self):
        super().__init__('detector_lugares')
        p = {nombre: self.declare_parameter(nombre, valor).value for nombre, valor in [
            ('salto_cm', 15.0),             # cuanto tiene que saltar un costado para abrirse
            ('lecturas_necesarias', 3),     # lecturas seguidas para confirmar
            ('distancia_frente_cm', 15.0),  # frente "muy cerca" (callejon, confirmar ya)
            ('ancho_pasillo_cm', 40.0),     # al arrancar: un costado mas lejos = salida
            # Esquina = el frente llega a F0 * esto. 0.5 era F0 / 2, pero el robot, ya con
            # mas velocidad al avanzar, no alcanzaba a frenar y chocaba. Subirlo reconoce la
            # esquina antes; bajarlo la reconoce mas cerca de la pared.
            ('fraccion_esquina', 0.65),
            # Testeo: detectar tambien en manual (manejando a mano o con un rosbag)
            ('modo_testeo', False),
            # Solo en testeo y manual: segundos entre un aviso y el reinicio automatico
            # (a 0.12 m/s, 2 s son ~24 cm: tiene que salir del lugar antes de buscar otro)
            ('reinicio_testeo_s', 2.0),
        ]}
        self.detector = DetectorDeLugares(
            p['salto_cm'], p['lecturas_necesarias'], p['distancia_frente_cm'],
            p['ancho_pasillo_cm'], p['fraccion_esquina'])
        self.modo_testeo = p['modo_testeo']
        self.reinicio_testeo_s = p['reinicio_testeo_s']
        self.timer_reinicio = None
        # Al arrancar no hay historia: puede estar parado en el medio de un lugar
        self.detector.mirar_alrededor()
        self.en_manual = False
        self.odom = None   # la ultima posicion (None = todavia no llego /odom)
        self.etapa_avisada = None

        self.create_subscription(
            Float32MultiArray, 'distancias', self.al_medir, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.al_moverse, 10)
        guardado = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'modo_actual', self.al_cambiar_modo, guardado)
        self.create_subscription(Empty, '/navegacion/reiniciar_detector', self.al_reiniciar, 10)

        self.pub_lugar = self.create_publisher(Lugar, '/navegacion/lugar', 10)
        self.get_logger().info(
            f'Detector de lugares listo (salto {p["salto_cm"]} cm, '
            f'{p["lecturas_necesarias"]} lecturas, frente {p["distancia_frente_cm"]} cm'
            f'{", MODO TESTEO: detecta tambien en manual" if self.modo_testeo else ""})')

    def al_moverse(self, mensaje):
        self.odom = mensaje.pose.pose

    def al_cambiar_modo(self, mensaje):
        en_manual = mensaje.data == MANUAL
        if en_manual == self.en_manual:
            return
        self.en_manual = en_manual
        if not en_manual:
            # Volvio a automatico: lo que sabia de antes ya no sirve (lo movieron a mano),
            # y puede haber quedado en el medio de un lugar.
            self.detector.mirar_alrededor()
        if en_manual and self.modo_testeo:
            self.get_logger().info('Modo manual (testeo): sigo avisando lugares')
        else:
            self.get_logger().info('Modo manual: no aviso lugares' if en_manual
                                   else 'Modo automatico: vuelvo a buscar lugares')

    def al_reiniciar(self, _mensaje):
        self.detector.reiniciar()
        self.get_logger().info('Reiniciado: busco el lugar siguiente')

    def reiniciar_por_testeo(self):
        """Testeo en manual: pasado el tiempo, busca el lugar siguiente (como /mover)."""
        self.timer_reinicio.cancel()
        self.destroy_timer(self.timer_reinicio)
        self.timer_reinicio = None
        self.detector.mirar_alrededor()
        self.get_logger().info('Testeo: busco el lugar siguiente')

    def al_medir(self, mensaje):
        if self.en_manual and not self.modo_testeo:
            return
        if len(mensaje.data) != 3:
            self.get_logger().warning(
                f'distancias: llegaron {len(mensaje.data)} valores (espero 3)',
                throttle_duration_sec=5.0)
            return
        izquierda, derecha, frente = (distancia_a_lectura(cm) for cm in mensaje.data)
        lugar = self.detector.actualizar(izquierda, derecha, frente)

        if self.detector.etapa != self.etapa_avisada:
            self.etapa_avisada = self.detector.etapa
            self.get_logger().info(f'Etapa: {self.detector.etapa}')
        if lugar is not None:
            self.avisar(*lugar)

    def avisar(self, tipo, salidas):
        aviso = Lugar(tipo=tipo, salidas=salidas)
        aviso.frente_inicial = numero_o_menos_uno(self.detector.frente_inicial_cm)
        aviso.frente_final = numero_o_menos_uno(self.detector.frente_final_cm)
        if self.odom is not None:
            aviso.x = self.odom.position.x
            aviso.y = self.odom.position.y
            aviso.orientacion = orientacion_de(self.odom.orientation)
        else:
            self.get_logger().warning('Sin /odom: aviso el lugar con posicion 0, 0')
        self.pub_lugar.publish(aviso)
        if self.modo_testeo and self.en_manual and self.timer_reinicio is None:
            self.timer_reinicio = self.create_timer(self.reinicio_testeo_s,
                                                    self.reiniciar_por_testeo)
        self.get_logger().info(
            f'LUGAR: {tipo} {salidas} en ({aviso.x:.2f}, {aviso.y:.2f}) '
            f'F0={self.detector.frente_inicial_cm} F1={self.detector.frente_final_cm}')


def numero_o_menos_uno(cm):
    """F0 y F1 van en el aviso como numero; -1 si no hay (por ejemplo, F1 en una esquina)."""
    return float(cm) if cm is not None and math.isfinite(cm) else -1.0


def main():
    rclpy.init()
    nodo = NodoDetectorLugares()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
