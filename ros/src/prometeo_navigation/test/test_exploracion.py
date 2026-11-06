# Pruebas de Exploracion (exploracion.py). Sin robot y sin simulador.
# Correr con: source install/setup.bash && python3 -m pytest src/prometeo_navigation/test

import math

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.exploracion import elegir_salida
from prometeo_navigation.libreta import Libreta

NORTE, OESTE, SUR, ESTE = 0.0, math.pi / 2, math.pi, -math.pi / 2   # orientaciones


def test_sin_lugar_actual_no_elige():
    assert elegir_salida(Libreta(), NORTE) is None


def test_elige_una_salida_sin_probar():
    libreta = Libreta()
    libreta.anotar('bifurcacion', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    assert elegir_salida(libreta, NORTE) == AvisoLugar.DERECHA


def test_no_elige_una_salida_ya_probada():
    libreta = Libreta()
    libreta.anotar('bifurcacion', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    libreta.tomar_salida(NORTE, AvisoLugar.DERECHA)
    libreta.actual = libreta.lugares[0]          # volvio a estar ahi
    assert elegir_salida(libreta, NORTE) == AvisoLugar.IZQUIERDA


def test_en_un_callejon_da_media_vuelta_hacia_la_bifurcacion():
    libreta = Libreta()
    libreta.anotar('bifurcacion', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    libreta.tomar_salida(NORTE, AvisoLugar.DERECHA)                 # al este
    libreta.anotar('callejon', 1.0, 0.0, ESTE, [])
    assert elegir_salida(libreta, ESTE) == Mover.Goal.VUELTA


def test_al_volver_a_la_bifurcacion_prueba_la_otra():
    libreta = Libreta()
    libreta.anotar('bifurcacion', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    libreta.tomar_salida(NORTE, AvisoLugar.DERECHA)                 # al este
    libreta.anotar('callejon', 1.0, 0.0, ESTE, [])
    libreta.tomar_salida(ESTE, Mover.Goal.VUELTA)                   # vuelve al oeste
    # Llega a la bifurcacion mirando al oeste: ve adelante (oeste) y izquierda (sur)
    libreta.anotar('bifurcacion', 0.05, 0.0, OESTE, [AvisoLugar.ADELANTE, AvisoLugar.IZQUIERDA])
    assert len(libreta.lugares) == 2               # la reconocio
    assert elegir_salida(libreta, OESTE) == AvisoLugar.ADELANTE    # el oeste, sin probar


def test_isla_no_repite_la_vuelta_y_vuelve_al_camino_pendiente():
    # A (0,0) salidas norte y oeste. B (0,1) curva. C (-1,1) es una T con una salida
    # extra al norte que queda pendiente. D (-1,0) curva. Vuelve a A desde el oeste.
    libreta = Libreta()
    libreta.anotar('bifurcacion', 0.0, 0.0, NORTE, [AvisoLugar.ADELANTE, AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(NORTE, AvisoLugar.ADELANTE)
    libreta.anotar('curva', 0.0, 1.0, NORTE, [AvisoLugar.IZQUIERDA])            # B
    libreta.tomar_salida(NORTE, AvisoLugar.IZQUIERDA)
    libreta.anotar('bifurcacion', -1.0, 1.0, OESTE,
                   [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])                 # C: sur y norte
    libreta.tomar_salida(OESTE, AvisoLugar.IZQUIERDA)                           # al sur
    libreta.anotar('curva', -1.0, 0.0, SUR, [AvisoLugar.IZQUIERDA])             # D
    libreta.tomar_salida(SUR, AvisoLugar.IZQUIERDA)                             # al este
    a, es_nuevo = libreta.anotar('bifurcacion', 0.0, 0.0, ESTE,
                                 [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    assert not es_nuevo
    # En A ya probo todo. El lugar con pendientes mas cercano es C (norte sin probar).
    # Por la libreta, A -> D -> C es por el oeste, y A -> B -> C por el norte: los dos a
    # 2 pasos. Elige uno de los dos caminos conocidos, nunca una salida nueva de A.
    eleccion = elegir_salida(libreta, ESTE)
    assert eleccion in (Mover.Goal.VUELTA, AvisoLugar.IZQUIERDA)   # oeste o norte


def test_sin_nada_pendiente_en_toda_la_libreta_da_media_vuelta():
    libreta = Libreta()
    libreta.anotar('curva', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(NORTE, AvisoLugar.IZQUIERDA)
    libreta.anotar('callejon', -1.0, 0.0, OESTE, [])
    assert elegir_salida(libreta, OESTE) == Mover.Goal.VUELTA
