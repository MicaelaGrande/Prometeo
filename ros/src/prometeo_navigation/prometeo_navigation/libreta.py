# La libreta del recorrido: anota por donde paso el robot y que caminos probo.
#
# Funciona como un mapa de subte: importan las estaciones (lugares) y las vias
# que las unen (salidas), no el dibujo exacto del laberinto.
#
# Es Python comun, sin ROS, asi se puede probar sin robot y sin simulador.

import math
from dataclasses import dataclass, field
from typing import List, Optional

# El aviso del detector se llama igual que la hoja de la libreta (Lugar): aca se lo
# trae con otro nombre para no confundirlos.
from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar as AvisoLugar

# Los dos estados posibles de una salida.
# Se guardan en constantes para no escribir el texto a mano cada vez:
# un error de tipeo en una constante da error, y en un texto pasa de largo.
PENDIENTE = 'pendiente'  # todavia no la recorrimos
PROBADA = 'probada'      # ya pasamos por ahi

# ===================== Puntos cardinales =====================
# La libreta anota las salidas en norte, sur, este y oeste, no en izquierda o derecha:
# "izquierda" cambia segun hacia donde mira el robot, y "norte" no. Asi reconoce el
# mismo lugar aunque llegue desde otro lado.
#
# No es el norte de verdad (TICKETS_NAVEGACION.md, T3): norte = hacia donde miraba el
# robot al arrancar (orientacion 0 en /odom). Oeste es 90 grados (a su izquierda),
# sur 180 y este 270.
ESTE = 'este'
NORTE = 'norte'
OESTE = 'oeste'
SUR = 'sur'
# En el orden en que se gira a la izquierda: cada uno esta 90 grados mas que el anterior
RUMBOS = (NORTE, OESTE, SUR, ESTE)

# Cuantos cuartos de vuelta a la izquierda queda cada salida respecto del frente.
# VUELTA (de Mover) es "atras": por donde vino.
CUARTOS_DE_SALIDA = {
    AvisoLugar.ADELANTE: 0,
    AvisoLugar.IZQUIERDA: 1,
    Mover.Goal.VUELTA: 2,
    AvisoLugar.DERECHA: 3,
}


def rumbo_de(orientacion):
    """Hacia que punto cardinal mira el robot, con la orientacion de /odom (radianes).

    Redondea al mas cercano: en el laberinto todo va a 90 grados, y un robot que
    mira a 80 grados (girado de mas o de menos) igual va hacia el oeste.
    """
    cuartos = round(orientacion / (math.pi / 2)) % 4
    return RUMBOS[cuartos]


def direccion_de_salida(orientacion, salida):
    """Pasa una salida vista desde el robot (izquierda...) a un punto cardinal.

    Por ejemplo, mirando al norte (como al arrancar), la izquierda es el oeste.
    None si la salida no existe (un texto mal escrito).
    """
    cuartos = CUARTOS_DE_SALIDA.get(salida)
    if cuartos is None:
        return None
    return RUMBOS[(RUMBOS.index(rumbo_de(orientacion)) + cuartos) % 4]


def salida_hacia(orientacion, direccion):
    """Al reves: que salida (izquierda, adelante...) lleva hacia ese punto cardinal.

    La van a usar Exploracion y /volver para pedirle a /mover una salida de la libreta.
    """
    if direccion not in RUMBOS:
        return None
    cuartos = (RUMBOS.index(direccion) - RUMBOS.index(rumbo_de(orientacion))) % 4
    for salida, c in CUARTOS_DE_SALIDA.items():
        if c == cuartos:
            return salida


# @dataclass arma una clase que solo guarda datos, sin escribir todo el codigo
# que hace falta para crearla: alcanza con listar sus campos.
@dataclass
class Salida:
    """Un camino que sale de un lugar del laberinto."""

    # Hacia donde va: 'norte', 'sur', 'este' u 'oeste'.
    direccion: str

    # Si ya la recorrimos. Toda salida nueva arranca pendiente.
    estado: str = PENDIENTE

    # El numero del lugar al que lleva. None significa que todavia no lo sabemos.
    lleva_a: Optional[int] = None


@dataclass
class Lugar:
    """Un punto del laberinto que vale la pena recordar. Es una hoja de la libreta."""

    # Su identificacion: 1, 2, 3... Es lo que se guarda en Salida.lleva_a.
    numero: int

    # Donde esta, en metros, medido desde donde arranco el robot. Sale de /odom.
    x: float
    y: float

    # Que clase de lugar es: 'curva', 'bifurcacion' o 'callejon'.
    tipo: str

    # Los caminos que salen de aca. Es una lista de objetos Salida.
    # field(default_factory=list) le da a cada Lugar su propia lista vacia.
    # Con un simple "= []", todos los lugares compartirian la misma.
    salidas: List[Salida] = field(default_factory=list)


# ===================== La libreta =====================

class Libreta:
    """Todas las hojas (Lugar) del recorrido. Empieza vacia en cada ronda.

    anotar() recibe cada aviso del detector y hace una de dos cosas:
    - si el robot ya estuvo ahi, devuelve esa misma hoja (no la repite);
    - si no, agrega una hoja nueva.
    """

    def __init__(self, margen_m=0.30):
        # Cuanto se puede equivocar /odom: dos avisos a menos de esto pueden ser el
        # mismo lugar. Se ajusta probando (el robot real patina al girar).
        self.margen_m = margen_m
        self.lugares = []

    def anotar(self, tipo, x, y, orientacion, salidas):
        """Recibe un aviso del detector. Devuelve (hoja, es_nuevo).

        salidas: como las manda el detector (izquierda, adelante, derecha). Se pasan
        a puntos cardinales. Tambien se anota por donde vino (atras), ya PROBADA:
        es una salida mas de ese lugar, y por ahi se vuelve.
        """
        direcciones = [direccion_de_salida(orientacion, s) for s in salidas]
        de_donde_vino = direccion_de_salida(orientacion, Mover.Goal.VUELTA)

        repetido = self.buscar(x, y, set(direcciones) | {de_donde_vino})
        if repetido is not None:
            return repetido, False

        hoja = Lugar(len(self.lugares) + 1, x, y, tipo,
                     [Salida(d) for d in direcciones] + [Salida(de_donde_vino, PROBADA)])
        self.lugares.append(hoja)
        return hoja, True

    def buscar(self, x, y, direcciones):
        """La hoja de un lugar ya anotado que coincide, o None.

        Tienen que coincidir dos cosas (TICKETS_NAVEGACION.md, T3):
        - la posicion, con el margen, porque /odom acumula error;
        - las direcciones de todas sus salidas (contando por donde vino). Asi no se
          confunde con otro lugar cercano que tiene otra forma.
        Si hay varios, el mas cercano.
        """
        candidatos = [
            hoja for hoja in self.lugares
            if math.hypot(hoja.x - x, hoja.y - y) <= self.margen_m
            and {s.direccion for s in hoja.salidas} == direcciones]
        if not candidatos:
            return None
        return min(candidatos, key=lambda hoja: math.hypot(hoja.x - x, hoja.y - y))
