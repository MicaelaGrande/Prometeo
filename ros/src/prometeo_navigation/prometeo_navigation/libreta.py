# La libreta del recorrido: anota por donde paso el robot y que caminos probo.
#
# Funciona como un mapa de subte: importan las estaciones (lugares) y las vias
# que las unen (salidas), no el dibujo exacto del laberinto.
#
# Es Python comun, sin ROS, asi se puede probar sin robot y sin simulador.

from dataclasses import dataclass, field
from typing import List, Optional

# Los dos estados posibles de una salida.
# Se guardan en constantes para no escribir el texto a mano cada vez:
# un error de tipeo en una constante da error, y en un texto pasa de largo.
PENDIENTE = 'pendiente'  # todavia no la recorrimos
PROBADA = 'probada'      # ya pasamos por ahi


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
