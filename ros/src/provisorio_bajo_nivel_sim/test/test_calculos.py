# PROVISORIO: pruebas de calculos.py. Se borran junto con el paquete (ver LEEME_BORRAR.md).
#
# pytest corre cada funcion que empieza con test_. assert significa "esto tiene que ser
# verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/provisorio_bajo_nivel_sim con:
#   python3 -m pytest test/test_calculos.py -v

from provisorio_bajo_nivel_sim.calculos import (
    FrenoEmergencia,
    debe_centrarse,
    giro_para_centrarse,
    hay_peligro,
    umbral_freno_cm,
)

# Los mismos valores que micro_ros/main/seguridad.c
T_REACCION_S = 0.14
T_FRENADO_S = 0.08
MARGEN_CM = 1.0
UMBRAL_MIN_CM = 3.0


def umbral(velocidad_m_s):
    return umbral_freno_cm(velocidad_m_s, T_REACCION_S, T_FRENADO_S, MARGEN_CM, UMBRAL_MIN_CM)


# ===== Freno =====

def test_umbral_da_lo_mismo_que_la_esp32():
    # seguridad.c: avance 1.0 (70 cm/s) da unos 16,4 cm. Aca 0,70 m/s es lo mismo.
    assert abs(umbral(0.70) - 16.4) < 0.01


def test_umbral_nunca_baja_del_minimo():
    assert umbral(0.0) == UMBRAL_MIN_CM


def test_frena_si_la_pared_esta_mas_cerca_que_el_umbral():
    assert hay_peligro(0.2, 4.0, umbral(0.2))


def test_no_frena_si_la_pared_esta_lejos():
    assert not hay_peligro(0.2, 50.0, umbral(0.2))


def test_avanzar_sin_ver_el_frente_es_peligro():
    assert hay_peligro(0.2, None, umbral(0.2))


def test_marcha_atras_nunca_es_peligro():
    # El sensor esta adelante: aunque la pared este pegada, retroceder se permite
    assert not hay_peligro(-0.2, 1.0, umbral(-0.2))


def test_el_freno_tarda_en_soltar():
    freno = FrenoEmergencia(histeresis_cm=2.0, lecturas_para_liberar=3)
    u = umbral(0.2)

    assert freno.actualizar(0.2, 4.0, u)       # pared cerca: frena
    assert freno.actualizar(0.2, 50.0, u)      # 1 lectura libre: sigue frenado
    assert freno.actualizar(0.2, 50.0, u)      # 2
    assert not freno.actualizar(0.2, 50.0, u)  # 3: suelta


def test_una_lectura_cerca_reinicia_la_cuenta():
    freno = FrenoEmergencia(histeresis_cm=2.0, lecturas_para_liberar=3)
    u = umbral(0.2)

    freno.actualizar(0.2, 4.0, u)
    freno.actualizar(0.2, 50.0, u)
    freno.actualizar(0.2, 50.0, u)
    freno.actualizar(0.2, 4.0, u)              # vuelve a estar cerca
    assert freno.actualizar(0.2, 50.0, u)      # empieza de cero: sigue frenado


def test_en_modo_recoger_no_frena():
    freno = FrenoEmergencia(histeresis_cm=2.0, lecturas_para_liberar=3)
    freno.habilitado = False

    assert not freno.actualizar(0.2, 1.0, umbral(0.2))


# ===== Centrado =====

def test_se_centra_solo_si_avanza_derecho():
    assert debe_centrarse(0.2, 0.0, giro_minimo_rad_s=0.05)


def test_respeta_el_giro_de_quien_manda():
    # T2 esta doblando: no se corrige
    assert not debe_centrarse(0.2, 0.8, giro_minimo_rad_s=0.05)


def test_no_se_centra_si_esta_quieto():
    assert not debe_centrarse(0.0, 0.0, giro_minimo_rad_s=0.05)


def centrar(izq_cm, der_cm):
    return giro_para_centrarse(izq_cm, der_cm, ganancia=2.0, giro_max_rad_s=0.5,
                               distancia_salida_cm=50.0)


def test_en_el_medio_no_gira():
    assert centrar(12.0, 12.0) == 0.0


def test_mas_lugar_a_la_izquierda_gira_a_la_izquierda():
    assert centrar(20.0, 5.0) > 0.0  # positivo = izquierda en ROS


def test_mas_lugar_a_la_derecha_gira_a_la_derecha():
    assert centrar(5.0, 20.0) < 0.0


def test_funciona_en_pasillos_de_distinto_ancho():
    # Mismo corrimiento en un pasillo angosto y en uno ancho: misma correccion
    assert centrar(10.0, 14.0) == centrar(30.0, 34.0)


def test_el_giro_tiene_un_limite():
    assert centrar(49.0, 1.0) == 0.5


def test_si_un_costado_se_abre_sigue_derecho():
    # Hay una salida a la izquierda: no se tira hacia el hueco
    assert centrar(120.0, 12.0) == 0.0


def test_sin_dato_de_un_costado_sigue_derecho():
    assert centrar(None, 12.0) == 0.0
