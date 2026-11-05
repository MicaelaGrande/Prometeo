# Pruebas de la logica del selector de modo (T11).
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_modos.py -v

from prometeo_navigation.modos import (
    AUTO, COMPU, FRENO, JOYSTICK, MANUAL, NADIE, Selector)

# Las ordenes reales son mensajes Twist de ROS. Aca alcanza con cualquier cosa
# que se pueda reconocer, porque el selector no mira adentro de la orden.
ADELANTE = 'adelante'
GIRAR = 'girar'
ATRAS = 'atras'


def nuevo():
    return Selector(edad_maxima_s=0.6)


# --- Modos ---

def test_arranca_en_manual_sin_freno_y_quieto():
    selector = nuevo()

    assert selector.modo == MANUAL
    assert selector.freno_puesto is False
    assert selector.orden_a_publicar(ahora_s=0.0) is None  # None = parar
    assert selector.fuente(ahora_s=0.0) == NADIE


def test_en_manual_pasa_la_orden_de_la_compu():
    selector = nuevo()
    selector.guardar_orden(COMPU, ADELANTE, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) == ADELANTE
    assert selector.fuente(ahora_s=10.1) == COMPU


def test_en_manual_pasa_la_orden_del_joystick():
    selector = nuevo()
    selector.guardar_orden(JOYSTICK, ATRAS, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) == ATRAS
    assert selector.fuente(ahora_s=10.1) == JOYSTICK


def test_en_manual_si_mandan_los_dos_gana_el_ultimo():
    selector = nuevo()
    selector.guardar_orden(COMPU, ADELANTE, ahora_s=10.0)
    selector.guardar_orden(JOYSTICK, ATRAS, ahora_s=10.2)

    assert selector.orden_a_publicar(ahora_s=10.3) == ATRAS
    assert selector.fuente(ahora_s=10.3) == JOYSTICK


def test_en_manual_no_pasa_la_orden_automatica():
    selector = nuevo()
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) is None


def test_en_auto_pasa_la_automatica_y_no_las_manuales():
    selector = nuevo()
    selector.pedir('auto')
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)
    selector.guardar_orden(COMPU, ADELANTE, ahora_s=10.0)
    selector.guardar_orden(JOYSTICK, ATRAS, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) == GIRAR
    assert selector.fuente(ahora_s=10.1) == AUTO


def test_si_dejan_de_llegar_ordenes_para():
    # se corto el teclado: la ultima orden queda vieja
    selector = nuevo()
    selector.guardar_orden(COMPU, ADELANTE, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.5) == ADELANTE  # todavia vale
    assert selector.orden_a_publicar(ahora_s=11.0) is None      # vieja: parar
    assert selector.fuente(ahora_s=11.0) == NADIE


def test_al_cambiar_de_modo_arranca_quieto():
    selector = nuevo()
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)  # llego antes del cambio
    selector.pedir('auto')

    assert selector.orden_a_publicar(ahora_s=10.1) is None  # no repite la vieja


def test_acepta_mayusculas_y_espacios():
    selector = nuevo()

    assert selector.pedir('  AUTO ') is True
    assert selector.modo == AUTO


def test_ignora_un_pedido_desconocido():
    selector = nuevo()

    assert selector.pedir('automatico') is False
    assert selector.modo == MANUAL


def test_pedir_el_mismo_modo_no_cambia_nada():
    selector = nuevo()
    selector.guardar_orden(COMPU, ADELANTE, ahora_s=10.0)

    assert selector.pedir('manual') is False
    assert selector.orden_a_publicar(ahora_s=10.1) == ADELANTE  # no se borro


# --- Freno de emergencia ---

def test_parar_frena_y_no_cambia_el_modo():
    selector = nuevo()
    selector.pedir('auto')
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)

    assert selector.pedir('parar') is True
    assert selector.modo == AUTO  # sigue en automatico
    assert selector.freno_puesto is True
    assert selector.orden_a_publicar(ahora_s=10.1) is None
    assert selector.fuente(ahora_s=10.1) == FRENO


def test_con_el_freno_puesto_no_pasa_ninguna_orden():
    selector = nuevo()
    selector.pedir('parar')
    selector.guardar_orden(COMPU, ADELANTE, ahora_s=10.0)
    selector.guardar_orden(JOYSTICK, ATRAS, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) is None


def test_el_freno_queda_puesto_aunque_se_cambie_de_modo():
    selector = nuevo()
    selector.pedir('parar')
    selector.pedir('auto')
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)

    assert selector.freno_puesto is True
    assert selector.orden_a_publicar(ahora_s=10.1) is None


def test_soltar_el_freno_sigue_en_el_mismo_modo_y_quieto():
    selector = nuevo()
    selector.pedir('auto')
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)
    selector.pedir('parar')

    assert selector.pedir('soltar') is True
    assert selector.modo == AUTO
    assert selector.orden_a_publicar(ahora_s=10.1) is None  # espera una orden nueva
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.2)
    assert selector.orden_a_publicar(ahora_s=10.3) == GIRAR


def test_parar_dos_veces_o_soltar_sin_freno_no_cambia_nada():
    selector = nuevo()

    assert selector.pedir('soltar') is False
    assert selector.pedir('parar') is True
    assert selector.pedir('parar') is False
