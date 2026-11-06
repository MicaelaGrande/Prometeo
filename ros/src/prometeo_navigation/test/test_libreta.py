# Pruebas de la libreta. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   source ../../install/setup.bash && python3 -m pytest test/test_libreta.py -v

import math

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.libreta import (
    ESTE, Lugar, NORTE, OESTE, PENDIENTE, Salida, SUR, direccion_de_salida, rumbo_de,
    salida_hacia)


def test_una_salida_nueva_arranca_pendiente():
    salida = Salida('norte')

    assert salida.direccion == 'norte'
    assert salida.estado == PENDIENTE
    assert salida.lleva_a is None  # todavia no sabemos adonde lleva


def test_un_lugar_guarda_su_posicion_y_sus_salidas():
    lugar = Lugar(1, 1.20, 0.40, 'bifurcacion', [Salida('norte'), Salida('este')])

    assert lugar.numero == 1
    assert lugar.x == 1.20
    assert lugar.y == 0.40
    assert lugar.tipo == 'bifurcacion'
    assert len(lugar.salidas) == 2  # len cuenta los elementos de una lista
    assert lugar.salidas[0].direccion == 'norte'  # la primera de la lista es la 0


def test_cada_lugar_tiene_su_propia_lista_de_salidas():
    lugar_1 = Lugar(1, 0.0, 0.0, 'curva')
    lugar_2 = Lugar(2, 1.0, 0.0, 'curva')

    lugar_1.salidas.append(Salida('norte'))  # append agrega al final de la lista

    assert len(lugar_1.salidas) == 1
    assert len(lugar_2.salidas) == 0


# ===== Puntos cardinales =====
# Orientacion de /odom: 0 = norte (como arranco), pi/2 = oeste, pi = sur, -pi/2 = este.

def test_al_arrancar_mira_al_norte():
    assert rumbo_de(0.0) == NORTE


def test_rumbo_de_los_cuatro_lados():
    assert rumbo_de(math.pi / 2) == OESTE
    assert rumbo_de(math.pi) == SUR
    assert rumbo_de(-math.pi) == SUR
    assert rumbo_de(-math.pi / 2) == ESTE


def test_rumbo_redondea_un_giro_de_mas_o_de_menos():
    assert rumbo_de(math.radians(80)) == OESTE
    assert rumbo_de(math.radians(100)) == OESTE
    assert rumbo_de(math.radians(-170)) == SUR
    assert rumbo_de(math.radians(5)) == NORTE


def test_mirando_al_norte_la_izquierda_es_el_oeste():
    assert direccion_de_salida(0.0, AvisoLugar.IZQUIERDA) == OESTE
    assert direccion_de_salida(0.0, AvisoLugar.DERECHA) == ESTE
    assert direccion_de_salida(0.0, AvisoLugar.ADELANTE) == NORTE


def test_mirando_al_oeste_la_izquierda_es_el_sur():
    assert direccion_de_salida(math.pi / 2, AvisoLugar.IZQUIERDA) == SUR
    assert direccion_de_salida(math.pi / 2, AvisoLugar.DERECHA) == NORTE


def test_atras_es_por_donde_vino():
    assert direccion_de_salida(0.0, Mover.Goal.VUELTA) == SUR


def test_salida_mal_escrita_no_tiene_direccion():
    assert direccion_de_salida(0.0, 'izqierda') is None


def test_salida_hacia_es_lo_contrario():
    assert salida_hacia(0.0, OESTE) == AvisoLugar.IZQUIERDA
    assert salida_hacia(0.0, NORTE) == AvisoLugar.ADELANTE
    assert salida_hacia(0.0, SUR) == Mover.Goal.VUELTA
    assert salida_hacia(math.pi / 2, NORTE) == AvisoLugar.DERECHA


def test_ida_y_vuelta_da_lo_mismo():
    for orientacion in (0.0, 1.5, 3.1, -1.6):
        for salida in (AvisoLugar.IZQUIERDA, AvisoLugar.ADELANTE, AvisoLugar.DERECHA):
            direccion = direccion_de_salida(orientacion, salida)
            assert salida_hacia(orientacion, direccion) == salida


def test_salida_hacia_un_rumbo_mal_escrito_es_none():
    assert salida_hacia(0.0, 'nrote') is None
