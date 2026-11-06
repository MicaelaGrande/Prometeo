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


def giro_de_salida(salida):
    """Cuanto girar (radianes, + izquierda) para tomar esa salida. None si no existe."""
    return GIRO_DE_SALIDA.get(salida)


def frente_abierto(frente_cm, umbral_cm):
    """True si adelante hay un pasillo (mide umbral_cm o mas, o sin eco). None sin dato.

    Sirve para confirmar el giro: despues de doblar, adelante tiene que estar el
    pasillo nuevo. Si sigue la pared, el robot giro de menos (las ruedas patinaron).
    """
    if frente_cm is None:
        return None
    return frente_cm >= umbral_cm


# Como se confirma un giro (siempre con el robot QUIETO: girando, una pared en
# diagonal no devuelve bien el eco y el sensor marca cualquier cosa).
CON_COSTADO = 'con_costado'      # esquina: la pared de adelante pasa a estar al costado
CON_FRENTE = 'con_frente'        # media vuelta: adelante tiene que quedar el pasillo
SIN_CONFIRMAR = 'sin_confirmar'  # puerta: antes no habia pared adelante, no hay con que


def como_confirmar(salida, frente_antes_cm, umbral_cm):
    """Elige como confirmar el giro, segun la salida y lo que media el frente antes."""
    if salida == Mover.Goal.VUELTA:
        return CON_FRENTE
    if salida in (Lugar.IZQUIERDA, Lugar.DERECHA) and frente_abierto(
            frente_antes_cm, umbral_cm) is False:
        return CON_COSTADO
    return SIN_CONFIRMAR


def giro_confirmado(como, salida, izquierda_cm, derecha_cm, frente_cm,
                    frente_antes_cm, tolerancia_cm, umbral_cm):
    """True si esta lectura (con el robot quieto) dice que el giro quedo bien.

    CON_COSTADO, dos cosas a la vez:
      - la pared que estaba adelante tiene que medir lo mismo desde el costado
        contrario al giro (doblo a la izquierda: queda a la derecha). El sensor solo
        da esa medida si mira la pared de frente, o sea si el robot quedo derecho;
      - y el frente tiene que haber cambiado: si sigue midiendo lo mismo que antes,
        no giro (la pared del pasillo de al lado puede medir parecido).
    CON_FRENTE: adelante tiene que estar el pasillo.
    None si falta el dato. SIN_CONFIRMAR siempre es True.
    """
    if como == SIN_CONFIRMAR:
        return True
    if como == CON_FRENTE:
        return frente_abierto(frente_cm, umbral_cm)
    costado = derecha_cm if salida == Lugar.IZQUIERDA else izquierda_cm
    if costado is None or frente_cm is None:
        return None
    pared_al_costado = abs(costado - frente_antes_cm) <= tolerancia_cm
    frente_cambio = abs(frente_cm - frente_antes_cm) > tolerancia_cm   # inf tambien cambia
    return pared_al_costado and frente_cambio


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


class PosicionLaberinto:
    """Donde esta el robot, sin encoders ni /odom: la "posicion de laberinto".

    Las ruedas patinan, asi que no se usa lo que dicen. En cambio:
    - la direccion sale de los giros que /mover CONFIRMO con los ultrasonidos: cada
      uno es un cuarto de vuelta justo (en el laberinto todo va a 90 grados);
    - la distancia sale del tiempo avanzando por la velocidad que se mando.
    No es exacta, pero alcanza para que Mapeo no mezcle lugares distintos.
    Orientacion 0 = hacia donde miraba al arrancar (el "norte" de la libreta).
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.cuartos = 0   # cuartos de vuelta a la izquierda desde el arranque (0 a 3)

    def orientacion(self):
        """En radianes, de -pi a pi, como /odom."""
        return normalizar_angulo(self.cuartos * math.pi / 2)

    def avanzar(self, metros):
        """Suma lo avanzado en la direccion en que mira (negativo = retrocede)."""
        self.x += metros * math.cos(self.orientacion())
        self.y += metros * math.sin(self.orientacion())

    def girar(self, giro_rad):
        """Suma un giro confirmado, redondeado a cuartos de vuelta."""
        self.cuartos = (self.cuartos + round(giro_rad / (math.pi / 2))) % 4

    def poner(self, x, y, orientacion):
        """La corrige (por ejemplo, el operador dice en que lugar lo dejo)."""
        self.x, self.y = x, y
        self.cuartos = round(orientacion / (math.pi / 2)) % 4
