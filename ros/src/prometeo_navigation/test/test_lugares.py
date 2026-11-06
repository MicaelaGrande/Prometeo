# Pruebas de la deteccion de lugares. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_lugares.py -v

import math

from prometeo_interfaces.msg import Lugar
from prometeo_navigation.lugares import (
    AVISADO, EN_PASILLO, MIDIENDO_LUGAR, MIRANDO_ALREDEDOR, Confirmador, DetectorDeLugares,
    Promedio,
    armar_lugar, costado_abierto, distancia_a_lectura, entro_al_pasillo, frente_muy_cerca,
    lectura_vigente)


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


# --- MIDIENDO_LUGAR: la esquina ---------------------------------------------

def llevar_a_midiendo(detector, izquierda=45, derecha=20, f0=80):
    """Pasillo de 20 cm de cada lado y despues 3 lecturas con la salida abierta.

    Termina en MIDIENDO_LUGAR con F0 = f0.
    """
    detector.actualizar(20, 20, frente_cm=f0 + 10)
    detector.actualizar(20, 20, frente_cm=f0 + 8)
    detector.actualizar(izquierda, derecha, frente_cm=f0 + 4)
    detector.actualizar(izquierda, derecha, frente_cm=f0 + 2)
    detector.actualizar(izquierda, derecha, frente_cm=f0)
    assert detector.etapa == MIDIENDO_LUGAR
    assert detector.frente_inicial_cm == f0


def test_esquina_avisa_curva_al_llegar_a_la_mitad_de_f0():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, f0=80)

    assert detector.actualizar(45, 20, frente_cm=60) is None   # todavia no llego
    assert detector.actualizar(45, 20, frente_cm=41) is None
    lugar = detector.actualizar(45, 20, frente_cm=40)           # 80 / 2

    assert lugar == (Lugar.CURVA, [Lugar.IZQUIERDA])
    assert detector.etapa == AVISADO


def test_esquina_a_la_derecha():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=20, derecha=45, f0=80)

    assert detector.actualizar(20, 45, frente_cm=38) == (Lugar.CURVA, [Lugar.DERECHA])


def test_t_con_los_dos_costados_abiertos_es_bifurcacion():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, derecha=45, f0=80)

    assert detector.actualizar(45, 45, frente_cm=40) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.DERECHA])


def test_t_con_el_segundo_costado_abriendose_mas_tarde():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, derecha=20, f0=80)
    for frente in (70, 65, 60):                     # la derecha se abre recien ahora
        assert detector.actualizar(45, 45, frente_cm=frente) is None

    assert detector.actualizar(45, 45, frente_cm=40) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.DERECHA])


def test_frente_sin_dato_o_sin_eco_sigue_midiendo():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, f0=80)

    assert detector.actualizar(45, 20, frente_cm=None) is None
    assert detector.actualizar(45, 20, frente_cm=math.inf) is None
    assert detector.etapa == MIDIENDO_LUGAR
    assert detector.actualizar(45, 20, frente_cm=40) == (Lugar.CURVA, [Lugar.IZQUIERDA])


def test_despues_de_avisar_no_vuelve_a_avisar():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, f0=80)
    assert detector.actualizar(45, 20, frente_cm=40) is not None

    for frente in (35, 30, 20, 10):                 # el timbre ya se toco
        assert detector.actualizar(45, 20, frente_cm=frente) is None
    assert detector.etapa == AVISADO


def test_en_el_pasillo_actualizar_no_avisa_nada():
    detector = nuevo_detector()
    for _ in range(5):
        assert detector.actualizar(20, 20, frente_cm=100) is None


# --- MIDIENDO_LUGAR: la puerta ----------------------------------------------

def test_puerta_a_la_izquierda_avisa_bifurcacion_con_adelante():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, f0=150)
    assert detector.actualizar(45, 20, frente_cm=140) is None    # pasando la abertura
    assert detector.actualizar(20, 20, frente_cm=130) is None    # vuelve la pared: 1
    assert detector.actualizar(20, 20, frente_cm=128) is None    # 2
    lugar = detector.actualizar(20, 20, frente_cm=126)           # 3: puerta confirmada

    assert lugar == (Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.ADELANTE])
    assert detector.etapa == AVISADO
    assert detector.frente_final_cm == 126                       # F1


def test_puerta_a_la_derecha():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=20, derecha=45, f0=150)
    for frente in (130, 128):
        assert detector.actualizar(20, 20, frente_cm=frente) is None

    assert detector.actualizar(20, 20, frente_cm=126) == (
        Lugar.BIFURCACION, [Lugar.ADELANTE, Lugar.DERECHA])


def test_una_sola_lectura_cerrada_no_confirma_la_puerta():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, f0=150)
    for izquierda in (20, 45, 20, 45, 20, 45):                   # el sensor tiembla
        assert detector.actualizar(izquierda, 20, frente_cm=130) is None

    assert detector.etapa == MIDIENDO_LUGAR
    assert not detector.puerta_confirmada


def test_puerta_con_el_frente_sin_eco_espera_un_numero_para_f1():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, f0=150)
    for _ in range(3):                                           # confirmada, sin eco
        assert detector.actualizar(20, 20, frente_cm=math.inf) is None
    assert detector.puerta_confirmada
    assert detector.actualizar(20, 20, frente_cm=None) is None

    assert detector.actualizar(20, 20, frente_cm=120) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.ADELANTE])
    assert detector.frente_final_cm == 120


def test_t_con_un_lado_que_se_cierra_avisa_un_cruce():
    # A mirar en Gazebo: puerta de un lado y esquina del otro da un cruce.
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, derecha=45, f0=150)
    for frente in (130, 128):
        assert detector.actualizar(45, 20, frente_cm=frente) is None   # se cierra la derecha

    assert detector.actualizar(45, 20, frente_cm=126) == (
        Lugar.BIFURCACION, [Lugar.IZQUIERDA, Lugar.ADELANTE, Lugar.DERECHA])


# --- Callejon ------------------------------------------------------------------

def test_callejon_frente_muy_cerca_sin_costados_abiertos():
    detector = nuevo_detector()                      # distancia_frente_cm = 15
    detector.actualizar(20, 20, frente_cm=40)
    assert detector.actualizar(20, 20, frente_cm=14) is None     # 1
    assert detector.actualizar(20, 20, frente_cm=13) is None     # 2
    lugar = detector.actualizar(20, 20, frente_cm=12)            # 3: confirmado

    assert lugar == (Lugar.CALLEJON, [])
    assert detector.etapa == AVISADO


def test_una_lectura_cerca_suelta_no_es_callejon():
    detector = nuevo_detector()
    for frente in (40, 12, 40, 12, 40, 12):          # el sensor tiembla
        assert detector.actualizar(20, 20, frente_cm=frente) is None

    assert detector.etapa == EN_PASILLO


def test_frente_sin_dato_no_corta_la_cuenta_del_callejon():
    detector = nuevo_detector()
    detector.actualizar(20, 20, frente_cm=40)
    detector.actualizar(20, 20, frente_cm=12)
    detector.actualizar(20, 20, frente_cm=None)      # no se sabe: la cuenta queda
    detector.actualizar(20, 20, frente_cm=12)

    assert detector.actualizar(20, 20, frente_cm=12) == (Lugar.CALLEJON, [])


def test_frente_cerca_con_un_costado_abierto_no_es_callejon():
    detector = nuevo_detector()
    detector.actualizar(20, 20, frente_cm=30)
    detector.actualizar(45, 20, frente_cm=12)        # costado abierto con frente cerca

    assert detector.etapa == MIDIENDO_LUGAR          # es un lugar con salida, no callejon


# --- Despues de avisar: reiniciar -------------------------------------------------

def test_reiniciar_vuelve_al_pasillo_y_olvida_todo():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, f0=80)
    detector.actualizar(45, 20, frente_cm=40)        # avisa la esquina
    detector.reiniciar()

    assert detector.etapa == EN_PASILLO
    assert detector.promedio_izquierda.valor() is None
    assert detector.promedio_derecha.valor() is None
    assert detector.frente_inicial_cm is None
    assert detector.frente_final_cm is None
    assert not detector.izquierda_abierta and not detector.derecha_abierta
    assert not detector.puerta_confirmada


def test_despues_de_reiniciar_detecta_el_lugar_siguiente():
    detector = nuevo_detector()
    llevar_a_midiendo(detector, izquierda=45, f0=80)
    assert detector.actualizar(45, 20, frente_cm=40) == (Lugar.CURVA, [Lugar.IZQUIERDA])
    detector.reiniciar()                             # termino de girar

    llevar_a_midiendo(detector, izquierda=20, derecha=45, f0=90)
    assert detector.actualizar(20, 45, frente_cm=45) == (Lugar.CURVA, [Lugar.DERECHA])


def test_despues_de_girar_el_lado_que_mira_atras_no_inventa_una_salida():
    # Giro a la izquierda: ahora la izquierda mira al pasillo de donde vino (60 cm).
    detector = nuevo_detector()
    detector.reiniciar()
    for _ in range(4):
        assert detector.actualizar(60, 20, frente_cm=100) is None
    assert detector.etapa == EN_PASILLO              # 60 es su primera "pared"

    detector.actualizar(20, 20, frente_cm=95)        # aparece la pared: bajon
    assert detector.promedio_izquierda.valor() == 20


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


# --- distancia_a_lectura: lo que llega por /distancias ---------------------------

def test_distancia_normal_queda_igual():
    assert distancia_a_lectura(23.5) == 23.5


def test_distancia_negativa_es_sensor_fallado():
    assert distancia_a_lectura(-1.0) is None


def test_distancia_nan_no_se_sabe():
    assert distancia_a_lectura(math.nan) is None


def test_400_o_mas_es_sin_eco():
    assert distancia_a_lectura(400.0) == math.inf
    assert distancia_a_lectura(650.0) == math.inf


# ===== MIRANDO_ALREDEDOR: al arrancar o al volver de manual (ancho de pasillo 40 cm) =====

def mirando():
    detector = nuevo_detector()
    detector.mirar_alrededor()
    return detector


def test_mirar_alrededor_pasa_a_esa_etapa():
    assert mirando().etapa == MIRANDO_ALREDEDOR


def test_arranca_en_un_pasillo_pasa_a_en_pasillo_sin_avisar():
    detector = mirando()
    avisos = [detector.actualizar(12, 12, 100) for _ in range(3)]
    assert avisos == [None, None, None]
    assert detector.etapa == EN_PASILLO


def test_arranca_en_una_esquina_avisa_curva():
    detector = mirando()
    avisos = [detector.actualizar(math.inf, 12, 10) for _ in range(3)]
    assert avisos == [None, None, (Lugar.CURVA, [Lugar.IZQUIERDA])]
    assert detector.etapa == AVISADO


def test_arranca_en_una_puerta_avisa_bifurcacion_con_adelante():
    detector = mirando()
    for _ in range(2):
        assert detector.actualizar(12, 60, 150) is None
    assert detector.actualizar(12, 60, 150) == (
        Lugar.BIFURCACION, [Lugar.ADELANTE, Lugar.DERECHA])


def test_alrededor_una_lectura_mala_no_inventa_una_salida():
    detector = mirando()
    detector.actualizar(12, 12, 100)
    detector.actualizar(80, 12, 100)   # lectura mala a la izquierda: corta la cuenta
    detector.actualizar(12, 12, 100)
    assert detector.etapa == MIRANDO_ALREDEDOR
    detector.actualizar(12, 12, 100)
    detector.actualizar(12, 12, 100)
    assert detector.etapa == EN_PASILLO


def test_alrededor_sin_dato_espera():
    detector = mirando()
    for _ in range(5):
        assert detector.actualizar(None, 12, 100) is None
    assert detector.etapa == MIRANDO_ALREDEDOR


def test_alrededor_con_salida_espera_a_saber_el_frente():
    detector = mirando()
    for _ in range(3):
        assert detector.actualizar(math.inf, 12, None) is None
    assert detector.etapa == MIRANDO_ALREDEDOR
    detector.actualizar(math.inf, 12, 10)
    detector.actualizar(math.inf, 12, 10)
    assert detector.actualizar(math.inf, 12, 10) == (Lugar.CURVA, [Lugar.IZQUIERDA])


def test_despues_de_mirar_alrededor_el_pasillo_detecta_normal():
    detector = mirando()
    for _ in range(3):
        detector.actualizar(12, 12, 100)
    llevar_a_midiendo(detector)
    assert detector.etapa == MIDIENDO_LUGAR


def test_mirar_alrededor_olvida_lo_de_antes():
    detector = nuevo_detector()
    llevar_a_midiendo(detector)
    detector.mirar_alrededor()
    assert detector.etapa == MIRANDO_ALREDEDOR
    assert detector.frente_inicial_cm is None
    assert detector.promedio_izquierda.valor() is None
