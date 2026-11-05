# Pruebas de la deteccion de lugares. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_lugares.py -v

import math

from prometeo_navigation.lugares import (
    Confirmador, costado_abierto, entro_al_pasillo, lectura_vigente)


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
