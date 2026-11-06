# Las cuentas de /mover (T2): cuanto girar segun la salida, a que velocidad, y
# hasta donde retroceder en una puerta.
#
# Son solo cuentas, sin ROS, igual que lugares.py: se prueban sin robot.
# Los angulos van en radianes. Positivo = a la izquierda (como en ROS y en /odom).

import math

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar

# Cuanto girar para cada salida
GIRO_DE_SALIDA = {
    Lugar.IZQUIERDA: math.pi / 2,
    Lugar.ADELANTE: 0.0,
    Lugar.DERECHA: -math.pi / 2,
    Mover.Goal.VUELTA: math.pi,
}


def normalizar_angulo(angulo):
    """Lleva un angulo al rango -pi a pi.

    Hace falta porque /odom pasa de 3,14 a -3,14 al dar la vuelta: sin esto, para
    ir de 3,1 a -3,1 (casi lo mismo) el robot creeria que tiene que girar casi una vuelta.
    """
    return math.atan2(math.sin(angulo), math.cos(angulo))


def angulo_objetivo(orientacion_actual, salida):
    """Hacia donde tiene que quedar mirando el robot para tomar esa salida.

    None si la salida no existe (un texto mal escrito).
    """
    giro = GIRO_DE_SALIDA.get(salida)
    if giro is None:
        return None
    return normalizar_angulo(orientacion_actual + giro)


def velocidad_de_giro(error, ganancia, giro_min, giro_max, tolerancia):
    """La velocidad de giro (rad/s) para corregir el error (rad). 0 = ya llego.

    Proporcional: gira rapido cuando falta mucho y despacio al final, para no
    pasarse. Nunca menos que giro_min (por debajo de eso los motores no mueven al
    robot, la zona muerta) ni mas que giro_max.
    """
    if abs(error) <= tolerancia:
        return 0.0
    velocidad = min(max(ganancia * abs(error), giro_min), giro_max)
    return math.copysign(velocidad, error)


def frente_en_el_medio_de_la_puerta(frente_inicial, frente_final):
    """Lo que mide el frente con el robot en el medio de la puerta (cm), o None.

    La puerta empezo con el frente en F0 y termino en F1 (F1 es menor: el robot
    avanzo). El medio esta en (F0 + F1) / 2. Con -1 (no hay dato) devuelve None.
    """
    if frente_inicial is None or frente_final is None:
        return None
    if frente_inicial < 0 or frente_final < 0:
        return None
    return (frente_inicial + frente_final) / 2


def es_puerta(lugar):
    """True si el lugar fue una puerta: tiene F1 (el costado se volvio a cerrar)."""
    return lugar is not None and lugar.frente_final >= 0
