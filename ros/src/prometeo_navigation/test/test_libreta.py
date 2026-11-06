# Pruebas de la libreta. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   source ../../install/setup.bash && python3 -m pytest test/test_libreta.py -v

import math

from prometeo_interfaces.action import Mover
from prometeo_interfaces.msg import Lugar as AvisoLugar
from prometeo_navigation.libreta import (
    ESTE, Libreta, Lugar, NORTE, OESTE, PENDIENTE, PROBADA, Salida, SUR, direccion_de_salida,
    rumbo_de,
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


# ===== La libreta: anotar un lugar nuevo o reconocer uno repetido =====
# Robot mirando al norte (orientacion 0), como al arrancar.

def test_la_libreta_empieza_vacia():
    assert Libreta().lugares == []


def test_anota_un_lugar_nuevo_con_sus_salidas_en_puntos_cardinales():
    libreta = Libreta()
    hoja, es_nuevo = libreta.anotar('curva', 1.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    assert es_nuevo
    assert hoja.numero == 1
    assert (hoja.x, hoja.y, hoja.tipo) == (1.0, 0.0, 'curva')
    assert [(s.direccion, s.estado) for s in hoja.salidas] == [
        (OESTE, PENDIENTE), (SUR, PROBADA)]   # izquierda y por donde vino


def test_un_callejon_solo_tiene_por_donde_vino():
    hoja, _ = Libreta().anotar('callejon', 0.0, 0.0, 0.0, [])
    assert [(s.direccion, s.estado) for s in hoja.salidas] == [(SUR, PROBADA)]


def test_los_lugares_se_numeran_en_orden():
    libreta = Libreta()
    libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    hoja, _ = libreta.anotar('curva', 2.0, 0.0, 0.0, [AvisoLugar.DERECHA])
    assert hoja.numero == 2
    assert len(libreta.lugares) == 2


def test_reconoce_un_lugar_al_volver_desde_otro_lado():
    # Una T: salidas al oeste, este y sur. Primero llega desde el sur (mirando al
    # norte); despues desde el oeste, mirando al este (orientacion -pi/2).
    libreta = Libreta()
    primera, _ = libreta.anotar('bifurcacion', 1.0, 1.0, 0.0,
                                [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    otra_vez, es_nuevo = libreta.anotar('bifurcacion', 1.1, 0.95, -math.pi / 2,
                                        [AvisoLugar.ADELANTE, AvisoLugar.DERECHA])
    assert not es_nuevo
    assert otra_vez is primera
    assert len(libreta.lugares) == 1


def test_mismo_lugar_pero_lejos_es_otro():
    libreta = Libreta(margen_m=0.30)
    libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    _, es_nuevo = libreta.anotar('curva', 0.5, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    assert es_nuevo


def test_cerca_pero_con_otra_forma_es_otro():
    libreta = Libreta()
    libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    _, es_nuevo = libreta.anotar('curva', 0.1, 0.0, 0.0, [AvisoLugar.DERECHA])
    assert es_nuevo


def test_justo_en_el_margen_es_el_mismo():
    libreta = Libreta(margen_m=0.30)
    libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    _, es_nuevo = libreta.anotar('curva', 0.30, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    assert not es_nuevo


def test_si_hay_dos_parecidos_elige_el_mas_cercano():
    libreta = Libreta(margen_m=0.30)
    lejos, _ = libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    libreta.lugares.append(Lugar(2, 0.4, 0.0, 'curva', list(lejos.salidas)))  # otro igual
    hoja, _ = libreta.anotar('curva', 0.35, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    assert hoja.numero == 2


# ===== Tomar una salida y unir los lugares =====

def test_tomar_salida_la_marca_probada():
    libreta = Libreta()
    hoja, _ = libreta.anotar('bifurcacion', 0.0, 0.0, 0.0,
                             [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    camino = libreta.tomar_salida(0.0, AvisoLugar.IZQUIERDA)
    assert camino.direccion == OESTE
    assert camino.estado == PROBADA
    este = [s for s in hoja.salidas if s.direccion == ESTE][0]
    assert este.estado == PENDIENTE   # la otra sigue sin probar


def test_al_llegar_al_siguiente_quedan_unidos_en_los_dos_sentidos():
    libreta = Libreta()
    a, _ = libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(0.0, AvisoLugar.IZQUIERDA)        # sale hacia el oeste
    # Ahora mira al oeste (pi/2) y llega a B, una curva a la derecha
    b, _ = libreta.anotar('curva', -1.0, 0.0, math.pi / 2, [AvisoLugar.DERECHA])
    oeste_de_a = [s for s in a.salidas if s.direccion == OESTE][0]
    este_de_b = [s for s in b.salidas if s.direccion == ESTE][0]   # por donde llego
    assert oeste_de_a.lleva_a == b.numero
    assert este_de_b.lleva_a == a.numero
    assert este_de_b.estado == PROBADA


def test_el_primer_lugar_no_sabe_de_donde_vino():
    hoja, _ = Libreta().anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    sur = [s for s in hoja.salidas if s.direccion == SUR][0]
    assert sur.lleva_a is None   # vino de la largada


def test_tomar_salida_sin_lugar_actual_no_hace_nada():
    assert Libreta().tomar_salida(0.0, AvisoLugar.IZQUIERDA) is None


def test_tomar_una_salida_que_no_existe_no_hace_nada():
    libreta = Libreta()
    libreta.anotar('curva', 0.0, 0.0, 0.0, [AvisoLugar.IZQUIERDA])
    assert libreta.tomar_salida(0.0, AvisoLugar.DERECHA) is None


def test_rodear_una_isla_marca_probado_el_camino_de_vuelta():
    # A tiene salidas al norte y al oeste. Sale por el norte, rodea la isla y
    # vuelve a A por el oeste: esa salida tiene que quedar probada (no repetir la vuelta).
    libreta = Libreta()
    a, _ = libreta.anotar('bifurcacion', 0.0, 0.0, 0.0,
                          [AvisoLugar.ADELANTE, AvisoLugar.IZQUIERDA])
    libreta.tomar_salida(0.0, AvisoLugar.ADELANTE)                      # norte
    libreta.anotar('curva', 0.0, 1.0, 0.0, [AvisoLugar.IZQUIERDA])      # B
    libreta.tomar_salida(0.0, AvisoLugar.IZQUIERDA)                     # oeste
    libreta.anotar('curva', -1.0, 1.0, math.pi / 2, [AvisoLugar.IZQUIERDA])  # C
    libreta.tomar_salida(math.pi / 2, AvisoLugar.IZQUIERDA)             # sur
    libreta.anotar('curva', -1.0, 0.0, math.pi, [AvisoLugar.IZQUIERDA])  # D
    libreta.tomar_salida(math.pi, AvisoLugar.IZQUIERDA)                 # este
    # Llega a A desde el oeste, mirando al este: ve izquierda = norte, derecha = sur
    # y atras = oeste. Son las mismas salidas de A: es A otra vez.
    otra_vez, es_nuevo = libreta.anotar('bifurcacion', 0.05, 0.0, -math.pi / 2,
                                        [AvisoLugar.IZQUIERDA, AvisoLugar.DERECHA])
    assert not es_nuevo and otra_vez is a
    assert all(s.estado == PROBADA for s in a.salidas)
    oeste = [s for s in a.salidas if s.direccion == OESTE][0]
    assert oeste.lleva_a == 4   # D
