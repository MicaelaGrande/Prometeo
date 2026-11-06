# Pruebas de la vuelta al inicio (vuelta.py). Sin robot y sin simulador.

import math

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.libreta import LARGADA, Libreta
from prometeo_navigation.vuelta import camino_a_la_largada

NORTE, OESTE, SUR, ESTE = 0.0, math.pi / 2, math.pi, -math.pi / 2


def test_el_primer_lugar_lleva_a_la_largada():
    libreta = Libreta()
    a, _ = libreta.anotar('curva', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA])
    sur = [s for s in a.salidas if s.direccion == 'sur'][0]
    assert sur.lleva_a == LARGADA


def test_mide_cuanto_tardo_cada_tramo():
    libreta = Libreta()
    a, _ = libreta.anotar('curva', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA], ahora_s=10.0)
    libreta.tomar_salida(NORTE, AvisoLugar.IZQUIERDA, ahora_s=12.0)
    b, _ = libreta.anotar('curva', -1.0, 0.0, OESTE, [AvisoLugar.DERECHA], ahora_s=19.5)
    oeste = [s for s in a.salidas if s.direccion == 'oeste'][0]
    este = [s for s in b.salidas if s.direccion == 'este'][0]
    assert oeste.segundos == 7.5 and este.segundos == 7.5


def test_desde_el_primer_lugar_sale_derecho_a_la_largada():
    libreta = Libreta()
    a, _ = libreta.anotar('curva', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA])
    assert camino_a_la_largada(libreta, a) == [(1, 'sur')]


def test_vuelve_por_los_lugares_que_paso():
    libreta = Libreta()
    libreta.anotar('curva', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(NORTE, AvisoLugar.IZQUIERDA)
    c, _ = libreta.anotar('curva', -1.0, 0.0, OESTE, [AvisoLugar.DERECHA])
    assert camino_a_la_largada(libreta, c) == [(2, 'este'), (1, 'sur')]


def test_elige_el_camino_mas_rapido_y_no_el_de_menos_lugares():
    # Desde A hay dos caminos a D: directo pero lento (20 s), o por B y C (3 + 3 + 3 s).
    #        B ─── C
    #        │     │
    #  ini ─ A ─── D
    libreta = Libreta()
    a, _ = libreta.anotar('bifurcacion', 0.0, 0.0, NORTE,
                          [AvisoLugar.ADELANTE, AvisoLugar.DERECHA], ahora_s=0.0)
    libreta.tomar_salida(NORTE, AvisoLugar.DERECHA, ahora_s=0.0)           # este, lento
    d, _ = libreta.anotar('bifurcacion', 2.0, 0.0, ESTE,
                          [AvisoLugar.IZQUIERDA], ahora_s=20.0)            # D: norte
    libreta.tomar_salida(ESTE, AvisoLugar.IZQUIERDA, ahora_s=20.0)         # norte
    libreta.anotar('curva', 2.0, 1.0, NORTE, [AvisoLugar.IZQUIERDA], ahora_s=23.0)  # C
    libreta.tomar_salida(NORTE, AvisoLugar.IZQUIERDA, ahora_s=23.0)        # oeste
    libreta.anotar('curva', 0.0, 1.0, OESTE, [AvisoLugar.IZQUIERDA], ahora_s=26.0)  # B
    libreta.tomar_salida(OESTE, AvisoLugar.IZQUIERDA, ahora_s=26.0)        # sur
    otra_vez, es_nuevo = libreta.anotar(
        'bifurcacion', 0.0, 0.05, SUR,
        [AvisoLugar.ADELANTE, AvisoLugar.IZQUIERDA], ahora_s=29.0)       # A otra vez
    assert not es_nuevo and otra_vez is a
    pasos = camino_a_la_largada(libreta, d)
    # D -> C -> B -> A -> largada: 3 + 3 + 3 = 9 s, mejor que D -> A directo (20 s)
    assert [n for n, _ in pasos] == [2, 3, 4, 1]
    assert pasos[-1] == (1, 'sur')


def test_libreta_partida_no_tiene_camino():
    libreta = Libreta()
    libreta.anotar('curva', 0.0, 0.0, NORTE, [AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(NORTE, AvisoLugar.IZQUIERDA)
    libreta.pasar_a_manual()
    c, _ = libreta.anotar('curva', -3.0, 2.0, OESTE, [AvisoLugar.DERECHA])
    assert camino_a_la_largada(libreta, c) is None


def test_no_entra_a_un_callejon_conocido():
    libreta = Libreta()
    libreta.anotar('bifurcacion', 0.0, 0.0, NORTE,
                   [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    libreta.tomar_salida(NORTE, AvisoLugar.DERECHA)                         # al este
    libreta.anotar('callejon', 1.0, 0.0, ESTE, [])
    libreta.tomar_salida(ESTE, Mover.Goal.VUELTA)
    libreta.anotar('bifurcacion', 0.05, 0.0, OESTE,
                   [AvisoLugar.ADELANTE, AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(OESTE, AvisoLugar.ADELANTE)                        # al oeste
    e, _ = libreta.anotar('curva', -1.0, 0.0, OESTE, [AvisoLugar.IZQUIERDA])
    assert camino_a_la_largada(libreta, e) == [(3, 'este'), (1, 'sur')]
