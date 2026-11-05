# Pruebas de la logica de Decision. pytest corre cada funcion que empieza con test_.
# assert significa "esto tiene que ser verdad": si no lo es, la prueba falla.
#
# Se corren desde la carpeta ros/src/prometeo_decision con:
#   python3 -m pytest test/test_logica.py -v

from types import SimpleNamespace

from prometeo_decision.logica import (
    ACERCANDO, ACERCARSE, BIFURCACION, BUSCANDO_COLOR, CALLEJON, CURVA,
    DERECHA, Decision, EXPLORAR, IR_A_SALIDA, IZQUIERDA, ADELANTE,
    SIGUIENDO, TERMINADO, VOLVIENDO,
)


# --- Ayudas: arman detecciones "de mentira", como las que manda la camara ---

def flecha(color, direccion='left', largo='normal', confianza=0.9):
    return SimpleNamespace(kind='arrow', color=color, direction=direccion,
                           length_class=largo, confidence=confianza)


def cubo(color, confianza=0.9):
    return SimpleNamespace(kind='cube', color=color, direction='',
                           length_class='', confidence=confianza)


# --- 1. Elegir color ---------------------------------------------------------

def test_la_primera_flecha_elige_el_color():
    d = Decision()
    d.ver([flecha('red')])

    assert d.color_actual == 'red'
    assert d.estado == SIGUIENDO


def test_una_flecha_dudosa_no_elige_color():
    d = Decision()
    d.ver([flecha('red', confianza=0.3)])  # menos que confianza_minima (0.6)

    assert d.color_actual is None
    assert d.estado == BUSCANDO_COLOR


# --- 2. Ignorar los otros colores -------------------------------------------

def test_siguiendo_rojo_ignora_la_flecha_verde():
    d = Decision()
    d.ver([flecha('red', 'left')])
    d.ver([flecha('green', 'right')])

    assert d.color_actual == 'red'  # no cambio de color
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, DERECHA])
    assert orden.salida == IZQUIERDA  # siguio la roja, no la verde


# --- 3. Seguir la flecha en la bifurcacion ----------------------------------

def test_flecha_roja_izquierda_manda_a_la_izquierda():
    d = Decision()
    d.ver([flecha('red', 'left', 'far')])
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, ADELANTE])

    assert orden.accion == IR_A_SALIDA
    assert orden.salida == IZQUIERDA
    assert orden.length_class == 'far'  # le pasa el largo a Navegacion


def test_flecha_para_arriba_manda_derecho():
    d = Decision()
    d.ver([flecha('red', 'up')])
    orden = d.llegar_a_lugar(BIFURCACION, [ADELANTE, DERECHA])

    assert orden.salida == ADELANTE


# --- 4. Cuando decide Exploracion -------------------------------------------

def test_bifurcacion_sin_flecha_de_mi_color_pide_explorar():
    d = Decision()
    d.ver([flecha('red')])
    d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, ADELANTE])  # usa la flecha roja
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, ADELANTE])  # ya no hay flecha

    assert orden.accion == EXPLORAR


def test_flecha_sin_direccion_clara_pide_explorar():
    d = Decision()
    d.ver([flecha('red', 'UNK')])
    orden = d.llegar_a_lugar(BIFURCACION, [IZQUIERDA, DERECHA])

    assert orden.accion == EXPLORAR


def test_callejon_pide_explorar_y_curva_no_pide_nada():
    d = Decision()
    d.ver([flecha('red')])

    assert d.llegar_a_lugar(CALLEJON, []).accion == EXPLORAR
    assert d.llegar_a_lugar(CURVA, [IZQUIERDA]) is None


# --- 5. Cubos ----------------------------------------------------------------

def test_siguiendo_rojo_ignora_el_cubo_verde():
    d = Decision()
    d.ver([flecha('red')])
    orden = d.ver([cubo('green')])

    assert orden is None
    assert d.estado == SIGUIENDO


def test_ve_el_cubo_rojo_y_se_acerca():
    d = Decision()
    d.ver([flecha('red')])
    orden = d.ver([cubo('red')])

    assert orden.accion == ACERCARSE
    assert d.estado == ACERCANDO


# --- 6. Agarrar, depositar y terminar ---------------------------------------

def test_depositar_libera_el_color():
    d = Decision()
    d.ver([flecha('red')])
    d.ver([cubo('red')])
    d.cubo_agarrado()
    assert d.estado == VOLVIENDO
    d.cubo_depositado()

    assert d.cubos_faltantes == {'green', 'blue'}
    assert d.color_actual is None
    assert d.estado == BUSCANDO_COLOR


def test_despues_de_los_3_cubos_termina():
    d = Decision()
    for color in ['red', 'green', 'blue']:
        d.ver([flecha(color)])
        d.ver([cubo(color)])
        d.cubo_agarrado()
        d.cubo_depositado()

    assert d.cubos_faltantes == set()
    assert d.estado == TERMINADO