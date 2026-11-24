# Pruebas de la llave de modo (una sola via: manual -> automatico -> terminado).
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_modos.py -v

from prometeo_navigation.modos import (
    AUTO, COMPU, JOYSTICK, Llave, MANUAL, NADIE, NAVEGACION, PEDIR_AUTO, PEDIR_PARAR,
    TERMINADO)

# Las ordenes reales son mensajes Twist de ROS. Aca alcanza con cualquier cosa
# que se pueda reconocer, porque la llave no mira adentro de la orden.
ADELANTE = 'adelante'
GIRAR = 'girar'
ATRAS = 'atras'


def nueva():
    return Llave(edad_maxima_s=0.6)


# --- El camino de una sola via ---

def test_arranca_en_manual_y_quieto():
    llave = nueva()

    assert llave.modo == MANUAL
    assert llave.orden_a_publicar(ahora_s=0.0) is None  # None = quieto
    assert llave.fuente(ahora_s=0.0) == NADIE


def test_de_manual_pasa_a_automatico():
    llave = nueva()

    assert llave.pedir(PEDIR_AUTO) is True
    assert llave.modo == AUTO


def test_de_automatico_no_se_puede_volver_a_manual():
    llave = nueva()
    llave.pedir(PEDIR_AUTO)

    assert llave.pedir(MANUAL) is False
    assert llave.modo == AUTO


def test_pedir_automatico_dos_veces_no_cambia_nada():
    llave = nueva()
    llave.pedir(PEDIR_AUTO)

    assert llave.pedir(PEDIR_AUTO) is False
    assert llave.modo == AUTO


def test_parar_desde_automatico_termina():
    llave = nueva()
    llave.pedir(PEDIR_AUTO)

    assert llave.pedir(PEDIR_PARAR) is True
    assert llave.modo == TERMINADO


def test_parar_desde_manual_tambien_termina():
    llave = nueva()

    assert llave.pedir(PEDIR_PARAR) is True
    assert llave.modo == TERMINADO


def test_terminado_es_para_siempre():
    llave = nueva()
    llave.pedir(PEDIR_PARAR)

    assert llave.pedir(PEDIR_AUTO) is False
    assert llave.pedir(MANUAL) is False
    assert llave.pedir(PEDIR_PARAR) is False
    assert llave.modo == TERMINADO


def test_un_pedido_desconocido_se_ignora():
    # Puede llegar del panel, que escribe los textos a mano
    llave = nueva()

    assert llave.pedir('autu') is False
    assert llave.modo == MANUAL


# --- Que orden pasa en cada modo ---

def test_en_manual_pasa_la_compu():
    llave = nueva()
    llave.guardar_orden(COMPU, ADELANTE, ahora_s=0.0)

    assert llave.orden_a_publicar(ahora_s=0.1) == ADELANTE
    assert llave.fuente(ahora_s=0.1) == COMPU


def test_en_manual_no_pasa_la_navegacion():
    llave = nueva()
    llave.guardar_orden(NAVEGACION, ADELANTE, ahora_s=0.0)

    assert llave.orden_a_publicar(ahora_s=0.1) is None


def test_en_manual_gana_el_que_mando_ultimo():
    llave = nueva()
    llave.guardar_orden(COMPU, ADELANTE, ahora_s=0.0)
    llave.guardar_orden(JOYSTICK, GIRAR, ahora_s=0.1)

    assert llave.orden_a_publicar(ahora_s=0.2) == GIRAR
    assert llave.fuente(ahora_s=0.2) == JOYSTICK


def test_en_automatico_pasa_solo_la_navegacion():
    llave = nueva()
    llave.pedir(PEDIR_AUTO)
    llave.guardar_orden(NAVEGACION, ADELANTE, ahora_s=0.0)
    llave.guardar_orden(JOYSTICK, ATRAS, ahora_s=0.1)   # tocaron el joystick
    llave.guardar_orden(COMPU, GIRAR, ahora_s=0.1)      # y el teclado

    assert llave.orden_a_publicar(ahora_s=0.2) == ADELANTE
    assert llave.fuente(ahora_s=0.2) == NAVEGACION


def test_al_pasar_a_automatico_no_repite_la_orden_manual():
    llave = nueva()
    llave.guardar_orden(COMPU, ADELANTE, ahora_s=0.0)
    llave.pedir(PEDIR_AUTO)

    # Hasta que la navegacion mande algo, quieto
    assert llave.orden_a_publicar(ahora_s=0.1) is None


def test_terminado_queda_quieto_aunque_manden_ordenes():
    llave = nueva()
    llave.pedir(PEDIR_AUTO)
    llave.pedir(PEDIR_PARAR)
    llave.guardar_orden(NAVEGACION, ADELANTE, ahora_s=0.0)
    llave.guardar_orden(COMPU, GIRAR, ahora_s=0.0)
    llave.guardar_orden(JOYSTICK, ATRAS, ahora_s=0.0)

    assert llave.orden_a_publicar(ahora_s=0.1) is None
    assert llave.fuente(ahora_s=0.1) == NADIE


# --- Ordenes viejas ---

def test_una_orden_vieja_no_pasa():
    llave = nueva()
    llave.guardar_orden(COMPU, ADELANTE, ahora_s=0.0)

    assert llave.orden_a_publicar(ahora_s=0.5) == ADELANTE   # todavia vale
    assert llave.orden_a_publicar(ahora_s=0.7) is None       # mas de 0,6 s: quieto


def test_si_la_navegacion_deja_de_mandar_queda_quieto():
    llave = nueva()
    llave.pedir(PEDIR_AUTO)
    llave.guardar_orden(NAVEGACION, ADELANTE, ahora_s=0.0)

    assert llave.orden_a_publicar(ahora_s=1.0) is None
    assert llave.fuente(ahora_s=1.0) == NADIE
