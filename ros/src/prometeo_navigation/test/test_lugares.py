# Pruebas de la deteccion de lugares. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_lugares.py -v

import math

from prometeo_interfaces.msg import Lugar
from prometeo_navigation.lugares import (
    EN_PASILLO, MIDIENDO_LUGAR, Confirmador, DetectorDeLugares, Promedio,
    armar_lugar, costado_abierto, entro_al_pasillo, frente_muy_cerca, lectura_vigente)


def test_entro_si_las_dos_paredes_estan_parecidas():
    assert entro_al_pasillo(18, 22, margen_cm=5)  # diferencia 4


def test_no_entro_si_una_pared_esta_mucho_mas_lejos():
    assert not entro_al_pasillo(10, 30, margen_cm=5)  # diferencia 20


def test_justo_en_el_margen_cuenta_como_entro():
    assert entro_al_pasillo(20, 25, margen_cm=5)  # diferencia 5, igual al margen


def test_no_importa_de_que_lado_este_mas_cerca():
    assert entro_al_pasillo(22, 18, margen_cm=5)


def test_sin_dato_de_un_sensor_todavia_no_sabemos():
    assert not entro_al_pasillo(None, 20, margen_cm=5)
    assert not entro_al_pasillo(20, None, margen_cm=5)


def test_costado_cerrado_si_mide_parecido_al_promedio():
    assert costado_abierto(22, promedio_cm=20, salto_cm=15) is False  # salto de 2


def test_costado_abierto_si_pega_un_salto():
    assert costado_abierto(45, promedio_cm=20, salto_cm=15) is True  # salto de 25


def test_justo_en_el_salto_cuenta_como_abierto():
    assert costado_abierto(35, promedio_cm=20, salto_cm=15) is True  # salto de 15


def test_sin_eco_cuenta_como_abierto():
    # inf: el sensor midio pero no volvio eco, no hay nada cerca
    assert costado_abierto(math.inf, promedio_cm=20, salto_cm=15) is True


def test_sin_dato_no_se_sabe():
    assert costado_abierto(None, promedio_cm=20, salto_cm=15) is None


def test_lectura_reciente_se_usa():
    assert lectura_vigente(20, hora_s=10.0, ahora_s=10.1, edad_maxima_s=0.5) == 20


def test_lectura_vieja_no_se_sabe():
    assert lectura_vigente(20, hora_s=10.0, ahora_s=11.0, edad_maxima_s=0.5) is None


def test_lectura_que_nunca_llego_no_se_sabe():
    assert lectura_vigente(None, hora_s=None, ahora_s=10.0, edad_maxima_s=0.5) is None


def test_confirma_despues_de_n_abiertos_seguidos():
    confirmador = Confirmador(lecturas_necesarias=3)

    assert confirmador.agregar(True) is False
    assert confirmador.agregar(True) is False
    assert confirmador.agregar(True) is True


def test_un_cerrado_corta_la_seguidilla():
    confirmador = Confirmador(lecturas_necesarias=3)
    confirmador.agregar(True)
    confirmador.agregar(True)

    assert confirmador.agregar(False) is False
    assert confirmador.cuenta == 0  # hay que empezar de nuevo


def test_no_se_sabe_deja_la_cuenta_igual():
    # el ejemplo acordado: abierto, abierto, no se sabe, abierto -> confirma
    confirmador = Confirmador(lecturas_necesarias=3)
    confirmador.agregar(True)
    confirmador.agregar(True)

    assert confirmador.agregar(None) is False
    assert confirmador.cuenta == 2
    assert confirmador.agregar(True) is True


def test_frente_lejos_no_esta_muy_cerca():
    assert frente_muy_cerca(80, distancia_cm=15) is False


def test_frente_cerca_esta_muy_cerca():
    assert frente_muy_cerca(10, distancia_cm=15) is True


def test_frente_justo_en_la_distancia_cuenta_como_cerca():
    assert frente_muy_cerca(15, distancia_cm=15) is True


def test_frente_sin_eco_no_esta_cerca():
    assert frente_muy_cerca(math.inf, distancia_cm=15) is False


def test_frente_sin_dato_no_se_sabe():
    assert frente_muy_cerca(None, distancia_cm=15) is None


def test_promedio_sin_lecturas_no_se_sabe():
    assert Promedio().valor() is None


def test_promedio_de_varias_lecturas():
    promedio = Promedio()
    promedio.agregar(18)
    promedio.agregar(20)
    promedio.agregar(22)

    assert promedio.valor() == 20


def test_promedio_ignora_sin_dato_y_sin_eco():
    promedio = Promedio()
    promedio.agregar(20)
    promedio.agregar(None)
    promedio.agregar(math.inf)  # si lo sumara, el promedio quedaria infinito

    assert promedio.valor() == 20
    assert promedio.cantidad == 1


def test_promedio_reiniciar_borra_todo():
    promedio = Promedio()
    promedio.agregar(60)
    promedio.reiniciar()

    assert promedio.valor() is None


# --- Detector de lugares ---
# Todas usan salto de 15 cm, 3 lecturas para confirmar y frente muy cerca a 15 cm.

def nuevo_detector():
    return DetectorDeLugares(salto_cm=15, lecturas_necesarias=3, distancia_frente_cm=15)


def test_el_detector_arranca_en_el_pasillo():
    assert nuevo_detector().etapa == EN_PASILLO


def test_pasillo_normal_sigue_en_el_pasillo():
    detector = nuevo_detector()
    for izquierda, derecha in [(18, 22), (19, 21), (20, 20), (18, 22), (19, 21)]:
        detector.actualizar(izquierda, derecha, frente_cm=100)

    assert detector.etapa == EN_PASILLO


def test_salto_repetido_pasa_a_midiendo_y_anota_f0():
    detector = nuevo_detector()
    detector.actualizar(20, 20, frente_cm=120)
    detector.actualizar(20, 20, frente_cm=118)
    detector.actualizar(45, 20, frente_cm=116)  # izquierda abierta 1
    detector.actualizar(45, 20, frente_cm=114)  # 2
    assert detector.etapa == EN_PASILLO
    detector.actualizar(45, 20, frente_cm=112)  # 3: confirmado

    assert detector.etapa == MIDIENDO_LUGAR
    assert detector.frente_inicial_cm == 112


def test_un_salto_suelto_no_confirma():
    detector = nuevo_detector()
    for izquierda in [20, 20, 45, 20, 45, 20]:
        detector.actualizar(izquierda, 20, frente_cm=100)

    assert detector.etapa == EN_PASILLO


def test_con_el_frente_muy_cerca_confirma_sin_esperar():
    detector = nuevo_detector()
    detector.actualizar(20, 20, frente_cm=30)
    detector.actualizar(45, 20, frente_cm=12)  # una sola lectura abierta, frente cerca

    assert detector.etapa == MIDIENDO_LUGAR


def test_bajon_reinicia_el_promedio():
    # El ejemplo acordado: despues de girar, la izquierda mira al pasillo de donde vino.
    detector = nuevo_detector()
    detector.actualizar(60, 20, frente_cm=100)
    detector.actualizar(60, 20, frente_cm=100)
    detector.actualizar(20, 20, frente_cm=100)  # bajon: aparecio la pared

    assert detector.promedio_izquierda.valor() == 20
    for _ in range(3):
        detector.actualizar(45, 20, frente_cm=100)  # salida de verdad
    assert detector.etapa == MIDIENDO_LUGAR


def test_sin_eco_al_principio_espera_a_ver_una_pared():
    detector = nuevo_detector()
    detector.actualizar(math.inf, 20, frente_cm=100)  # mira al pasillo de atras, sin eco
    detector.actualizar(math.inf, 20, frente_cm=100)
    detector.actualizar(math.inf, 20, frente_cm=100)

    assert detector.etapa == EN_PASILLO  # sin promedio no puede decir que se abrio
    assert detector.promedio_izquierda.valor() is None


def test_no_anota_f0_sin_un_frente_que_sirva():
    detector = nuevo_detector()
    detector.actualizar(20, 20, frente_cm=100)
    for _ in range(3):
        detector.actualizar(45, 20, frente_cm=math.inf)  # confirmado, pero frente sin eco
    assert detector.etapa == EN_PASILLO

    detector.actualizar(45, 20, frente_cm=None)
    assert detector.etapa == EN_PASILLO

    detector.actualizar(45, 20, frente_cm=90)  # ahora si hay un numero
    assert detector.etapa == MIDIENDO_LUGAR
    assert detector.frente_inicial_cm == 90


# --- armar_lugar: el traductor para Decision (una prueba por fila de la tabla) ---
# armar_lugar(izquierda_abierta, adelante_abierto, derecha_abierta)

def test_esquina_a_la_izquierda_es_curva():
    assert armar_lugar(True, False, False) == (Lugar.CURVA, [Lugar.IZQUIERDA])


def test_esquina_a_la_derecha_es_curva():
    assert armar_lugar(False, False, True) == (Lugar.CURVA, [Lugar.DERECHA])


def test_esquina_con_los_dos_costados_abiertos_es_bifurcacion():  # una T
    assert armar_lugar(True, False, True) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.DERECHA])


def test_puerta_a_la_izquierda_es_bifurcacion():
    assert armar_lugar(True, True, False) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.ADELANTE])


def test_puerta_a_la_derecha_es_bifurcacion():
    assert armar_lugar(False, True, True) == (
        Lugar.BIFURCACION, [Lugar.ADELANTE, Lugar.DERECHA])


def test_cruce_es_bifurcacion_con_tres_salidas():
    assert armar_lugar(True, True, True) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.ADELANTE, Lugar.DERECHA])


def test_sin_salidas_es_callejon():
    assert armar_lugar(False, False, False) == (Lugar.CALLEJON, [])
