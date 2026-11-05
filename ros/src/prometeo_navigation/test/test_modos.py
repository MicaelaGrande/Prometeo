# Pruebas de la logica del selector de modo (T11).
#
# Se corren desde la carpeta ros/src/prometeo_navigation con:
#   python3 -m pytest test/test_modos.py -v

from prometeo_navigation.modos import AUTO, MANUAL, Selector

# Las ordenes reales son mensajes Twist de ROS. Aca alcanza con cualquier cosa
# que se pueda reconocer, porque el selector no mira adentro de la orden.
ADELANTE = 'adelante'
GIRAR = 'girar'


def test_arranca_en_manual_y_quieto():
    selector = Selector(edad_maxima_s=0.6)

    assert selector.modo == MANUAL
    assert selector.orden_a_publicar(ahora_s=0.0) is None  # None = parar


def test_en_manual_pasa_la_orden_manual():
    selector = Selector(edad_maxima_s=0.6)
    selector.guardar_orden(MANUAL, ADELANTE, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) == ADELANTE


def test_en_manual_no_pasa_la_orden_automatica():
    selector = Selector(edad_maxima_s=0.6)
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) is None


def test_en_auto_pasa_la_automatica_y_no_la_manual():
    selector = Selector(edad_maxima_s=0.6)
    selector.cambiar_modo('auto')
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)
    selector.guardar_orden(MANUAL, ADELANTE, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.1) == GIRAR


def test_si_dejan_de_llegar_ordenes_para():
    # se corto el teclado: la ultima orden queda vieja
    selector = Selector(edad_maxima_s=0.6)
    selector.guardar_orden(MANUAL, ADELANTE, ahora_s=10.0)

    assert selector.orden_a_publicar(ahora_s=10.5) == ADELANTE  # todavia vale
    assert selector.orden_a_publicar(ahora_s=11.0) is None      # vieja: parar


def test_al_cambiar_de_modo_arranca_quieto():
    selector = Selector(edad_maxima_s=0.6)
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)  # llego antes del cambio
    selector.cambiar_modo('auto')

    assert selector.orden_a_publicar(ahora_s=10.1) is None  # no repite la vieja


def test_acepta_mayusculas_y_espacios():
    selector = Selector(edad_maxima_s=0.6)

    assert selector.cambiar_modo('  AUTO ') is True
    assert selector.modo == AUTO


def test_ignora_un_modo_desconocido():
    selector = Selector(edad_maxima_s=0.6)

    assert selector.cambiar_modo('automatico') is False
    assert selector.modo == MANUAL


def test_pedir_el_mismo_modo_no_cambia_nada():
    selector = Selector(edad_maxima_s=0.6)
    selector.guardar_orden(MANUAL, ADELANTE, ahora_s=10.0)

    assert selector.cambiar_modo('manual') is False
    assert selector.orden_a_publicar(ahora_s=10.1) == ADELANTE  # no se borro


def test_parar_desde_auto_pasa_a_manual_quieto():
    selector = Selector(edad_maxima_s=0.6)
    selector.cambiar_modo('auto')
    selector.guardar_orden(AUTO, GIRAR, ahora_s=10.0)

    assert selector.cambiar_modo('parar') is True
    assert selector.modo == MANUAL
    assert selector.orden_a_publicar(ahora_s=10.1) is None


def test_parar_en_manual_frena_la_orden_que_venia():
    # ya estaba en manual y alguien apretaba adelante: el panico igual frena
    selector = Selector(edad_maxima_s=0.6)
    selector.guardar_orden(MANUAL, ADELANTE, ahora_s=10.0)

    assert selector.cambiar_modo('parar') is True
    assert selector.orden_a_publicar(ahora_s=10.1) is None
