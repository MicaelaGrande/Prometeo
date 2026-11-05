# Pruebas de los botones del joystick que cambian de modo (T11).
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_botones_joystick.py -v

from prometeo_navigation.botones_joystick import BotonesJoystick

PARAR, SOLTAR, MANUAL, AUTO = 1, 3, 4, 6


def nuevos():
    return BotonesJoystick({PARAR: 'parar', MANUAL: 'manual', AUTO: 'auto'},
                           boton_soltar=SOLTAR, tiempo_soltar_s=1.0)


def apretados(*numeros):
    """Lista de 11 botones con los pedidos en 1."""
    return [1 if i in numeros else 0 for i in range(11)]


def test_sin_botones_no_pide_nada():
    assert nuevos().actualizar(apretados(), ahora_s=0.0) == []


def test_apretar_parar_pide_parar_una_sola_vez():
    botones = nuevos()

    assert botones.actualizar(apretados(PARAR), ahora_s=0.0) == ['parar']
    assert botones.actualizar(apretados(PARAR), ahora_s=0.1) == []  # sigue apretado
    assert botones.actualizar(apretados(), ahora_s=0.2) == []
    assert botones.actualizar(apretados(PARAR), ahora_s=0.3) == ['parar']  # otra vez


def test_manual_y_auto():
    botones = nuevos()

    assert botones.actualizar(apretados(MANUAL), ahora_s=0.0) == ['manual']
    assert botones.actualizar(apretados(), ahora_s=0.1) == []
    assert botones.actualizar(apretados(AUTO), ahora_s=0.2) == ['auto']


def test_soltar_hay_que_mantenerlo():
    botones = nuevos()

    assert botones.actualizar(apretados(SOLTAR), ahora_s=0.0) == []
    assert botones.actualizar(apretados(SOLTAR), ahora_s=0.5) == []
    assert botones.actualizar(apretados(SOLTAR), ahora_s=1.0) == ['soltar']
    assert botones.actualizar(apretados(SOLTAR), ahora_s=2.0) == []  # una sola vez


def test_soltar_apretado_corto_no_hace_nada():
    botones = nuevos()
    botones.actualizar(apretados(SOLTAR), ahora_s=0.0)
    botones.actualizar(apretados(), ahora_s=0.5)  # lo solto antes del segundo

    assert botones.actualizar(apretados(SOLTAR), ahora_s=0.9) == []  # empieza de nuevo


def test_joystick_con_menos_botones_no_falla():
    assert nuevos().actualizar([0, 1], ahora_s=0.0) == ['parar']
