"""Modelo del HC-SR04: que mediria el sensor real a partir de los rayos de Gazebo.

Este archivo no usa ROS, asi se puede probar sin abrir Gazebo. Tiene dos partes:

  1. El sonido: un ultrasonido no mide "lo que tiene adelante". Manda un cono de
     sonido, y el sonido rebota en una pared lisa como una pelota: solo vuelve al
     sensor si pego casi de frente. Si la pared esta muy inclinada, el eco se va
     para otro lado y el sensor cree que no hay nada.

  2. El firmware: las mismas cuentas que hcsr04.c y sensores.c del ESP32.

Los valores de Parametros son estimados: hay que ajustarlos midiendo con el robot.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

# Valores de micro_ros/drivers/hcsr04/hcsr04.c y micro_ros/main/sensores.h
US_POR_CM = 58.0        # distancia(cm) = tiempo_eco(us) / 58
ECO_MIN_US = 116        # 2 cm * 58
ECO_MAX_US = 23200      # 400 cm * 58
DIST_LEJOS_CM = 400.0   # el sensor no ve nada
DIST_SIN_DATO = -1.0    # el sensor no contesta
VENTANA_MEDIANA = 3

# Dos tramos de pared forman una esquina si sus direcciones difieren en mas que esto
GIRO_MIN_ESQUINA_RAD = math.radians(60.0)


@dataclass
class Parametros:
    angulo_max_rebote_deg: float = 20.0  # mas inclinado que esto, el eco no vuelve
    ruido_base_cm: float = 0.3           # ruido a cualquier distancia
    ruido_proporcional: float = 0.002    # ruido extra: 0.002 es 0,2 % de la distancia
    prob_eco_perdido: float = 0.01       # chance de perder una lectura
    temperatura_c: float = 20.0          # cambia la velocidad del sonido
    sesgo_cm: float = 0.0                # error fijo propio de cada modulo


def _direccion(desde, hasta):
    """Direccion en radianes de la linea que va de un punto a otro."""
    delta = hasta - desde
    return math.atan2(delta[1], delta[0])


def _diferencia_de_lineas(a, b):
    """Angulo entre dos lineas, de 0 a 90 grados (una linea no tiene sentido de ida)."""
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def _cruce_de_lineas(p1, p2, p3, p4):
    """Punto donde se cruza la recta p1-p2 con la recta p3-p4, o None si son paralelas."""
    d1 = p2 - p1
    d2 = p4 - p3
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-9:
        return None
    t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / det
    return p1 + t * d1


def _eco_de_pared(rangos, angulos, puntos, i, angulo_max):
    """Dice si el rayo i rebota en una pared lo bastante de frente como para volver.

    La inclinacion de la pared sale de la linea entre este punto y el vecino. Si el
    vecino cae en otra pared, esa linea queda casi paralela al rayo y se descarta sola.
    """
    for vecino in (i - 1, i + 1):
        if vecino < 0 or vecino >= len(rangos) or not np.isfinite(rangos[vecino]):
            continue
        pared = _direccion(puntos[i], puntos[vecino])
        normal = pared + math.pi / 2
        if _diferencia_de_lineas(angulos[i], normal) <= angulo_max:
            return True
    return False


def _eco_de_esquina(rangos, angulos, puntos, i):
    """Busca una esquina interior entre los rayos i e i+1.

    Una esquina interior devuelve el sonido hacia donde vino, como un espejo doble,
    aunque ninguna de las dos paredes este de frente. Vista desde el sensor es el punto
    mas lejano de las dos paredes. Devuelve su distancia, o None si no hay esquina.
    """
    a, b, c, d = i - 1, i, i + 1, i + 2
    if a < 0 or d >= len(rangos) or not np.all(np.isfinite(rangos[[a, b, c, d]])):
        return None

    giro = _diferencia_de_lineas(_direccion(puntos[a], puntos[b]),
                                 _direccion(puntos[c], puntos[d]))
    if giro < GIRO_MIN_ESQUINA_RAD:
        return None

    # En una esquina interior la distancia crece hacia la esquina de los dos lados.
    # En una punta hacia afuera, como la de la isla, pasa lo contrario.
    if rangos[a] > rangos[b] or rangos[d] > rangos[c]:
        return None

    # El cruce tiene que caer entre los dos rayos. Si cae afuera, son dos paredes que
    # no se tocan: una tapa a la otra y no hay esquina.
    cruce = _cruce_de_lineas(puntos[a], puntos[b], puntos[c], puntos[d])
    if cruce is None:
        return None
    angulo = math.atan2(cruce[1], cruce[0])
    if not angulos[b] <= angulo <= angulos[c]:
        return None
    return float(np.hypot(cruce[0], cruce[1]))


def eco_mas_cercano(rangos, angulos, angulo_max_rebote_deg):
    """Distancia en metros del primer eco que vuelve, o None si no vuelve ninguno.

    rangos y angulos son los rayos del abanico, ordenados por angulo y vistos desde
    el sensor. Un rayo que no toco nada tiene rango infinito.
    """
    rangos = np.asarray(rangos, dtype=float)
    angulos = np.asarray(angulos, dtype=float)
    con_dato = np.where(np.isfinite(rangos), rangos, 0.0)
    puntos = np.column_stack((con_dato * np.cos(angulos), con_dato * np.sin(angulos)))
    angulo_max = math.radians(angulo_max_rebote_deg)

    ecos = []
    for i in range(len(rangos)):
        if not np.isfinite(rangos[i]):
            continue
        if _eco_de_pared(rangos, angulos, puntos, i, angulo_max):
            ecos.append(rangos[i])
        esquina = _eco_de_esquina(rangos, angulos, puntos, i)
        if esquina is not None:
            ecos.append(esquina)

    return float(min(ecos)) if ecos else None


def velocidad_sonido(temperatura_c):
    """Velocidad del sonido en m/s."""
    return 331.3 + 0.606 * temperatura_c


def eco_a_cm(eco_us):
    """Convierte el tiempo de eco en cm, como finish() en hcsr04.c y sensores_task().

    Fuera de rango da "lejos". Por eso, un sensor pegado a la pared tambien da 400.
    """
    if eco_us < ECO_MIN_US or eco_us > ECO_MAX_US:
        return DIST_LEJOS_CM
    return eco_us / US_POR_CM


def lectura_cm(rangos, angulos, parametros, generador):
    """Una medicion del sensor en cm, sin mediana."""
    distancia = eco_mas_cercano(rangos, angulos, parametros.angulo_max_rebote_deg)
    if distancia is None or generador.random() < parametros.prob_eco_perdido:
        return DIST_LEJOS_CM

    ruido = parametros.ruido_base_cm + parametros.ruido_proporcional * distancia * 100.0
    medida_m = distancia + (parametros.sesgo_cm + generador.normal(0.0, ruido)) / 100.0

    # El sensor mide tiempo y el firmware lo convierte suponiendo 58 us/cm. Si la
    # temperatura real es otra, queda un error pequeno, igual que en el robot.
    eco_us = round(max(medida_m, 0.0) * 2.0 / velocidad_sonido(parametros.temperatura_c) * 1e6)
    return eco_a_cm(eco_us)


class Mediana:
    """Mediana de las ultimas lecturas, igual que historial_t en sensores.c."""

    def __init__(self):
        self.lecturas = deque(maxlen=VENTANA_MEDIANA)

    def agregar(self, cm):
        self.lecturas.append(cm)
        return self.valor()

    def valor(self):
        ordenadas = sorted(self.lecturas)
        return ordenadas[len(ordenadas) // 2]
