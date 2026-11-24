#!/usr/bin/env python3
"""Mira TODOS los topics que hay dando vueltas y le manda un resumen al panel.

Cada buscar_cada_s busca topics nuevos y se suscribe a cada uno, solo para leer (no
publica nada en ellos). Cada periodo_s publica en /monitor/topicos un texto JSON:

    {"hora": 1732400000.0,
     "topics": [{"nombre": "/prometeo/distancias", "tipo": "std_msgs/msg/Float32MultiArray",
                 "hz": 10.0, "edad_s": 0.05,
                 "campos": {"data[0]": 12.5, "data[1]": 13.0, "data[2]": 400.0}}, ...]}

hz: cuantos mensajes por segundo llegaron en los ultimos ventana_s segundos.
edad_s: hace cuanto llego el ultimo (null = nunca llego ninguno).
campos: el ultimo mensaje, aplanado ("pose.position.x": 0.82). Las listas largas (una
imagen, por ejemplo) se resumen como "[230400 valores]" para no mandar todo.

Asi el panel ve todo sin tener que abrir cada topic en rosbridge.
"""

import array
import json
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

TOPIC_RESUMEN = '/monitor/topicos'


def aplanar(valor, nombre, salida, max_elementos, max_texto, max_campos):
    """Pasa un mensaje de ROS a un diccionario plano {"pose.position.x": 0.82}."""
    if len(salida) >= max_campos:
        return
    if hasattr(valor, 'get_fields_and_field_types'):   # un mensaje adentro de otro
        for campo in valor.get_fields_and_field_types():
            hijo = f'{nombre}.{campo}' if nombre else campo
            aplanar(getattr(valor, campo), hijo, salida, max_elementos, max_texto, max_campos)
        return
    if isinstance(valor, (list, tuple, array.array, bytes, bytearray)) or hasattr(valor, 'tolist'):
        n = len(valor)
        primitivo = n == 0 or not hasattr(valor[0], 'get_fields_and_field_types')
        if primitivo and n > max_elementos:
            salida[nombre] = f'[{n} valores]'
            return
        for i, elemento in enumerate(list(valor)[:max_elementos]):
            aplanar(elemento, f'{nombre}[{i}]', salida, max_elementos, max_texto, max_campos)
        if n > max_elementos:
            salida[f'{nombre}[…]'] = f'{n - max_elementos} más'
        return
    if isinstance(valor, float):
        salida[nombre] = round(valor, 4)
    elif isinstance(valor, (bool, int)):
        salida[nombre] = valor
    else:
        texto = str(valor)
        salida[nombre] = texto if len(texto) <= max_texto else texto[:max_texto] + '…'


class Vigilado:
    """Lo que se sabe de un topic."""

    def __init__(self, tipo, clase):
        self.tipo = tipo
        self.clase = clase
        self.horas = deque()      # cuando llego cada mensaje (de los ultimos ventana_s)
        self.ultimo = None        # el ultimo mensaje, todavia sin abrir (bytes)
        self.hora_ultimo = None
        self.campos = {}          # el ultimo mensaje ya abierto
        self.abierto = True       # si campos ya corresponde a ultimo


class MonitorTopicos(Node):

    def __init__(self):
        super().__init__('monitor_topicos')
        p = {nombre: self.declare_parameter(nombre, valor).value for nombre, valor in [
            ('periodo_s', 1.0), ('buscar_cada_s', 2.0), ('ventana_s', 3.0),
            ('max_elementos', 8), ('max_texto', 120), ('max_campos', 40),
            ('ignorar', [TOPIC_RESUMEN, '/rosout', '/parameter_events'])]}
        self.p = p
        self.vigilados = {}     # nombre -> Vigilado
        self.suscripciones = []
        self.pub = self.create_publisher(String, TOPIC_RESUMEN, 10)
        self.create_timer(p['buscar_cada_s'], self.buscar)
        self.create_timer(p['periodo_s'], self.publicar)
        self.buscar()

    def qos_para(self, nombre):
        """Si todos los que publican guardan el ultimo mensaje (como la libreta), pedirlo.

        Si no, best effort: sirve tanto para los que publican reliable como best effort.
        """
        infos = self.get_publishers_info_by_topic(nombre)
        guardan = infos and all(
            i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL and
            i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
        if guardan:
            return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        return QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)

    def buscar(self):
        for nombre, tipos in self.get_topic_names_and_types():
            if nombre in self.vigilados or nombre in self.p['ignorar'] or not tipos:
                continue
            try:
                clase = get_message(tipos[0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                self.get_logger().warning(f'{nombre}: no conozco el tipo {tipos[0]}, no lo miro')
                self.vigilados[nombre] = Vigilado(tipos[0], None)
                continue
            self.vigilados[nombre] = Vigilado(tipos[0], clase)
            # raw=True: llegan los bytes sin abrir. Se abren solo al publicar el resumen,
            # asi una imagen a 12 Hz no cuesta casi nada.
            self.suscripciones.append(self.create_subscription(
                clase, nombre, lambda datos, n=nombre: self.llego(n, datos),
                self.qos_para(nombre), raw=True))

    def llego(self, nombre, datos):
        v = self.vigilados[nombre]
        ahora = time.monotonic()
        v.horas.append(ahora)
        v.ultimo = datos
        v.hora_ultimo = ahora
        v.abierto = False

    def publicar(self):
        ahora = time.monotonic()
        ventana = self.p['ventana_s']
        lista = []
        for nombre in sorted(self.vigilados):
            v = self.vigilados[nombre]
            while v.horas and ahora - v.horas[0] > ventana:
                v.horas.popleft()
            if not v.abierto and v.clase is not None:
                campos = {}
                try:
                    aplanar(deserialize_message(v.ultimo, v.clase), '', campos,
                            self.p['max_elementos'], self.p['max_texto'], self.p['max_campos'])
                except Exception as error:   # un mensaje roto no tiene que tirar el monitor
                    campos = {'error': str(error)}
                v.campos = campos
                v.abierto = True
            lista.append({
                'nombre': nombre, 'tipo': v.tipo, 'hz': round(len(v.horas) / ventana, 1),
                'edad_s': None if v.hora_ultimo is None else round(ahora - v.hora_ultimo, 2),
                'campos': v.campos})
        self.pub.publish(String(data=json.dumps({'hora': time.time(), 'topics': lista})))


def main():
    rclpy.init()
    nodo = MonitorTopicos()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
