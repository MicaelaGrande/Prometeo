# Pruebas de los botones del joystick que le hablan a la llave de modo.
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_botones_joystick.py -v

from prometeo_navigation.botones_joystick import BotonesJoystick
from prometeo_navigation.modos import PEDIR_AUTO, PEDIR_PARAR

PARAR, AUTO = 1, 6


def nuevos():
    return BotonesJoystick({PARAR: PEDIR_PARAR}, boton_mantener=AUTO,
                           pedido_mantener=PEDIR_AUTO, tiempo_mantener_s=1.0)


def apretados(*numeros):
    """Lista de 11 botones con los pedidos en 1."""
    return [1 if i in numeros else 0 for i in range(11)]


def test_sin_botones_no_pide_nada():
    assert nuevos().actualizar(apretados(), ahora_s=0.0) == []


def test_apretar_parar_pide_parar_una_sola_vez():
    botones = nuevos()

    assert botones.actualizar(apretados(PARAR), ahora_s=0.0) == [PEDIR_PARAR]
    # Sigue apretado: no lo repite
    assert botones.actualizar(apretados(PARAR), ahora_s=0.1) == []
    # Lo suelta y lo vuelve a apretar: lo pide de nuevo
    assert botones.actualizar(apretados(), ahora_s=0.2) == []
    assert botones.actualizar(apretados(PARAR), ahora_s=0.3) == [PEDIR_PARAR]


def test_automatico_hay_que_mantenerlo():
    botones = nuevos()

    assert botones.actualizar(apretados(AUTO), ahora_s=0.0) == []
    assert botones.actualizar(apretados(AUTO), ahora_s=0.5) == []
    assert botones.actualizar(apretados(AUTO), ahora_s=1.0) == [PEDIR_AUTO]
    # Sigue apretado: no lo repite
    assert botones.actualizar(apretados(AUTO), ahora_s=2.0) == []


def test_automatico_apretado_corto_no_hace_nada():
    botones = nuevos()

    assert botones.actualizar(apretados(AUTO), ahora_s=0.0) == []
    assert botones.actualizar(apretados(), ahora_s=0.5) == []       # lo solto antes
    assert botones.actualizar(apretados(AUTO), ahora_s=0.6) == []   # empieza de cero
    assert botones.actualizar(apretados(AUTO), ahora_s=1.2) == []


def test_joystick_con_menos_botones_no_falla():
    botones = nuevos()

    assert botones.actualizar([1, 1], ahora_s=0.0) == [PEDIR_PARAR]
